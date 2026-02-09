"""
Roomba WiFi server with protocol support, error handling, and safety features
"""

import network
from picozero import pico_led
from time import sleep, time
import rp2
import sys
import socket
import serial
import json

# Import protocol
try:
    from protocol import Message, Commands, Responses, CMD_DRIVE, CMD_STOP, CMD_MODE, CMD_LED, CMD_SENSORS, CMD_SONG, CMD_PLAY_SONG, CMD_PING, CMD_STATUS
except ImportError:
    print("Warning: protocol.py not found, using fallback")
    Message = None

# Configuration
WATCHDOG_TIMEOUT = 2.0  # Stop robot if no command received in 2 seconds
MAX_BUFFER_SIZE = 2048
KEEPALIVE_INTERVAL = 1.0

# State
last_command_time = 0
watchdog_enabled = True

with open(f"wifi_creds.json", 'r') as f:
    creds = json.load(f)
    ssid = creds["ssid"]
    password = creds["password"]

def connect():
    """Connect to WiFi with retry logic"""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    retry_count = 0
    max_retries = 10
    
    while not wlan.isconnected() and retry_count < max_retries:
        if rp2.bootsel_button():
            print("BOOTSEL pressed, exiting...")
            sys.exit()
        
        print(f'Connecting to {ssid}... (attempt {retry_count + 1}/{max_retries})')
        wlan.connect(ssid, password)
        
        # Wait up to 10 seconds for connection
        for i in range(20):
            if wlan.isconnected():
                break
            pico_led.toggle()
            sleep(0.5)
        
        retry_count += 1
    
    if not wlan.isconnected():
        print("Failed to connect to WiFi")
        pico_led.off()
        return None
    
    ip = wlan.ifconfig()[0]
    print(f'Connected on {ip}')
    pico_led.on()
    return ip

def open_socket(ip):
    """Create server socket"""
    address = (ip, 80)
    connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    connection.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    connection.bind(address)
    connection.listen(1)
    connection.setblocking(False)  # Non-blocking for watchdog
    print(f"Server listening on {address}")
    return connection

def handle_command(msg):
    """
    Process incoming command message
    Returns: response bytes to send back
    """
    global last_command_time
    last_command_time = time()
    
    try:
        cmd_type = msg.get("type")
        
        if cmd_type == CMD_DRIVE:
            velocity = msg.get("velocity", 0)
            radius = msg.get("radius", 0)
            serial.drive(velocity, radius)
            return Responses.ok(f"Driving: v={velocity}, r={radius}")
        
        elif cmd_type == CMD_STOP:
            serial.stop()
            return Responses.ok("Stopped")
        
        elif cmd_type == CMD_MODE:
            mode = msg.get("mode", "safe")
            serial.set_mode(mode)
            return Responses.ok(f"Mode set to {mode}")
        
        elif cmd_type == CMD_LED:
            serial.set_leds(
                debris=msg.get("debris", False),
                spot=msg.get("spot", False),
                dock=msg.get("dock", False),
                check_robot=msg.get("check_robot", False),
                power_color=msg.get("power_color", 0),
                power_intensity=msg.get("power_intensity", 255)
            )
            return Responses.ok("LEDs updated")
        
        elif cmd_type == CMD_SENSORS:
            packet_ids = msg.get("packet_ids", [])
            sensor_data = {}
            for pid in packet_ids:
                data = serial.read_sensors(pid)
                if data:
                    sensor_data[pid] = list(data)
            return Message.encode("sensor_data", data=sensor_data)
        
        elif cmd_type == CMD_SONG:
            song_num = msg.get("song_num", 0)
            notes = msg.get("notes", [])
            serial.define_song(song_num, notes)
            return Responses.ok(f"Song {song_num} defined")
        
        elif cmd_type == CMD_PLAY_SONG:
            song_num = msg.get("song_num", 0)
            serial.play_song(song_num)
            return Responses.ok(f"Playing song {song_num}")
        
        elif cmd_type == CMD_PING:
            return Responses.pong()
        
        elif cmd_type == CMD_STATUS:
            battery = serial.get_battery_status()
            if battery:
                return Responses.status(
                    battery_voltage=battery["voltage"],
                    battery_charge=battery["charge"],
                    battery_capacity=battery["capacity"],
                    mode=serial.current_mode
                )
            else:
                return Responses.status(mode=serial.current_mode)
        
        else:
            return Responses.error(f"Unknown command: {cmd_type}")
    
    except Exception as e:
        print(f"Error handling command: {e}")
        return Responses.error(str(e))

def watchdog_check():
    """Safety watchdog - stop robot if no recent commands"""
    global last_command_time, watchdog_enabled
    
    if not watchdog_enabled:
        return
    
    elapsed = time() - last_command_time
    if elapsed > WATCHDOG_TIMEOUT:
        try:
            serial.stop()
            print(f"Watchdog triggered! Stopped robot (no command for {elapsed:.1f}s)")
            watchdog_enabled = False  # Disable until next command
        except:
            pass

def serve(connection):
    """Main server loop with robust error handling"""
    global last_command_time, watchdog_enabled
    
    # Initialize Roomba
    print("Initializing Roomba...")
    if not serial.init_serial_oi(mode="safe"):
        print("Failed to initialize Roomba!")
        return
    
    print("Server started. Waiting for connections...")
    last_command_time = time()
    
    client = None
    client_addr = None
    receive_buffer = b""
    
    while True:
        try:
            # Check watchdog
            watchdog_check()
            
            # Try to accept new connection if none active
            if client is None:
                try:
                    client, client_addr = connection.accept()
                    client.setblocking(False)
                    print(f"Client connected from {client_addr}")
                    
                    # Send welcome message
                    welcome = Responses.ok("Connected to Roomba Server")
                    client.send(welcome)
                    
                    last_command_time = time()
                    watchdog_enabled = True
                    receive_buffer = b""
                    
                except OSError:
                    # No connection yet, continue
                    sleep(0.1)
                    continue
            
            # Handle existing connection
            try:
                # Receive data (non-blocking)
                chunk = client.recv(1024)
                
                if not chunk:
                    # Client disconnected
                    print("Client disconnected")
                    client.close()
                    client = None
                    serial.stop()
                    watchdog_enabled = False
                    continue
                
                # Add to buffer
                receive_buffer += chunk
                
                # Process complete messages (newline-delimited)
                while b'\n' in receive_buffer:
                    line, receive_buffer = receive_buffer.split(b'\n', 1)
                    
                    if not line:
                        continue
                    
                    # Parse message
                    if Message:
                        msg = Message.decode(line)
                    else:
                        # Fallback: parse simple text protocol
                        msg = parse_simple_protocol(line)
                    
                    print(f"Received: {msg}")
                    
                    # Handle command
                    response = handle_command(msg)
                    
                    # Send response
                    try:
                        client.send(response)
                    except OSError:
                        print("Failed to send response")
                
                # Clear buffer if too large
                if len(receive_buffer) > MAX_BUFFER_SIZE:
                    print("Buffer overflow, clearing")
                    receive_buffer = b""
            
            except OSError as e:
                # No data available (non-blocking), continue
                if e.args[0] == 11:  # EAGAIN
                    sleep(0.01)
                else:
                    print(f"Socket error: {e}")
                    if client:
                        client.close()
                        client = None
                    serial.stop()
        
        except KeyboardInterrupt:
            print("\nShutting down...")
            break
        
        except Exception as e:
            print(f"Unexpected error: {e}")
            if client:
                try:
                    client.close()
                except:
                    pass
                client = None
            serial.stop()
            sleep(1)
    
    # Cleanup
    print("Server shutting down...")
    serial.shutdown()
    if client:
        client.close()
    connection.close()

def parse_simple_protocol(line):
    """Fallback parser for simple text protocol"""
    try:
        parts = line.decode('utf-8').strip().split()
        
        if len(parts) >= 3 and parts[0] == 'drive':
            return {
                "type": CMD_DRIVE,
                "velocity": int(parts[1]),
                "radius": int(parts[2])
            }
        elif parts[0] == 'stop':
            return {"type": CMD_STOP}
        else:
            return {"type": "unknown"}
    except:
        return {"type": "error", "message": "Parse failed"}