from machine import Pin, PWM
import network
from time import sleep, time
from picozero import pico_led
from protocol import Message, Responses

import rp2
import sys
import socket
import json
    
# Constants
PWM_FREQ = 50          # Hz
PERIOD_US = 20000      # 20 ms
MIN_US = 600           # lower bound
MAX_US = 2800          # upper bound

MAX_BUFFER_SIZE = 2048

#Define servo
servo = PWM(Pin(15))
servo.freq(PWM_FREQ)

with open(f"wifi_creds.json", 'r') as f:
    creds = json.load(f)
    ssid = creds["ssid"]
    password = creds["password"]

def angle_to_duty_u16(angle):
    angle = max(0, min(180, angle))  # clamp
    pulse_us = MIN_US + (angle / 180) * (MAX_US - MIN_US)
    duty = int((pulse_us / PERIOD_US) * 65535)
    return duty

def set_servo_angle(servo, angle):
    servo.duty_u16(angle_to_duty_u16(angle))

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

def parse_simple_protocol(line):
    """Fallback parser for simple text protocol"""
    try:
        parts = line.decode('utf-8').strip().split()
        
        return {
            "type": parts[0],
            "velocity": int(parts[1]),
            "radius": int(parts[2])
        }

    except:
        return {"type": "error", "message": "Parse failed"}

def handle_command(msg):
    global servo
    """
    Process incoming message
    """
    try:
        cmd_type = msg.get("type")
        print(cmd_type)
        
        if cmd_type == "dispense":
            pico_led.on()
            set_servo_angle(servo, 96)
            sleep(3)
            set_servo_angle(servo, 10)
            sleep(3)
            pico_led.off()
            return Responses.ok("Dispensed")
        return Responses.ok("Unknown command")
    except Exception as e:
        print(f"Error handling command: {e}")
        return Responses.error(str(e))
            
def serve(connection):
    """Main server loop"""
    client = None
    client_addr = None
    receive_buffer = b""
    
    while True:
        try:
            if client is None:
                try:
                    client, client_addr = connection.accept()
                    client.setblocking(False)
                    print(f"client_connected from {client_addr}")
                    client.send(Responses.ok("Connected to dispenser server"))
                except OSError:
                    # No connection yet, continue
                    sleep(0.1)
                    continue
                
            try:
                #Recieve data
                chunk = client.recv(1024)
                
                if not chunk:
                    #Client disconnected
                    print("client disconnected")
                    client.close()
                    client = None
                
                receive_buffer += chunk
                while b'\n' in receive_buffer:
                    line, receive_buffer = receive_buffer.split(b'\n', 1)
                    
                    if not line:
                        continue
                    
                    if Message:
                        msg = Message.decode(line)
                    else:
                        msg = parse_simple_protocol(line)
                        
                    print(f"Recieved: {msg}")
                    
                    response = handle_command(msg)
                    
                    #Send Response
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
        
        except Exception as e:
            print(f"Unexpected error: {e}")
            if client:
                try:
                    client.close()
                except:
                    pass
                client = None
            sleep(1)
    
    # Cleanup
    print("Server shutting down...")
    if client:
        client.close()
    connection.close()
        


