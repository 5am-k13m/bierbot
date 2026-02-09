"""
Roomba serial control with full OI command support
"""

import machine
import time
from picozero import pico_led
import struct

ROOMBA_BAUD_RATE = 115200

# Roomba OpCodes
OPCODE_START = 128
OPCODE_BAUD = 129
OPCODE_SAFE = 131
OPCODE_FULL = 132
OPCODE_POWER = 133
OPCODE_SPOT = 134
OPCODE_CLEAN = 135
OPCODE_MAX = 136
OPCODE_DRIVE = 137
OPCODE_MOTORS = 138
OPCODE_LEDS = 139
OPCODE_SONG = 140
OPCODE_PLAY = 141
OPCODE_SENSORS = 142
OPCODE_SEEK_DOCK = 143
OPCODE_STOP = 173

# Initialize UART
uart = machine.UART(0, baudrate=ROOMBA_BAUD_RATE, 
                    tx=machine.Pin(12), rx=machine.Pin(13), 
                    bits=8, parity=None, stop=1, timeout=1000)

device_detect_pin = machine.Pin(14, machine.Pin.OUT)

# State tracking
current_mode = "off"
last_command_time = 0
COMMAND_TIMEOUT_MS = 100  # Minimum time between commands

print(f"UART initialized at {ROOMBA_BAUD_RATE} baud.")

class RoombaError(Exception):
    """Custom exception for Roomba errors"""
    pass

def send_command(command_byte, data_bytes=None):
    """
    Sends a command to the Roomba with rate limiting
    Raises RoombaError on failure
    """
    global last_command_time
    
    # Rate limiting
    now = time.ticks_ms()
    elapsed = time.ticks_diff(now, last_command_time)
    if elapsed < COMMAND_TIMEOUT_MS:
        time.sleep_ms(COMMAND_TIMEOUT_MS - elapsed)
    
    try:
        # Clear RX buffer
        while uart.any():
            uart.read()
        
        uart.write(bytes([command_byte]))
        if data_bytes:
            uart.write(bytes(data_bytes))
        
        last_command_time = time.ticks_ms()
        
        print(f"Sent: 0x{command_byte:02x}", end='')
        if data_bytes:
            print(f" + {[f'0x{b:02x}' for b in data_bytes]}")
        else:
            print()
        
        return True
        
    except Exception as e:
        raise RoombaError(f"Command failed: {e}")

def init_serial_oi(mode="safe"):
    """
    Initialize Roomba OI
    mode: "safe" or "full"
    Returns: True on success, False on failure
    """
    global current_mode
    
    try:
        send_command(OPCODE_START)
        time.sleep_ms(500)
        
        if mode == "full":
            send_command(OPCODE_FULL)
            current_mode = "full"
        else:
            send_command(OPCODE_SAFE)
            current_mode = "safe"
        
        time.sleep_ms(100)
        
        pico_led.on()
        print(f"Roomba initialized in {current_mode} mode")
        return True
        
    except RoombaError as e:
        print(f"Init failed: {e}")
        pico_led.off()
        return False

def set_mode(mode):
    """Change Roomba mode: 'safe', 'full', or 'passive'"""
    global current_mode
    
    if mode == "safe":
        send_command(OPCODE_SAFE)
        current_mode = "safe"
    elif mode == "full":
        send_command(OPCODE_FULL)
        current_mode = "full"
    elif mode == "passive":
        send_command(OPCODE_START)
        current_mode = "passive"
    else:
        raise RoombaError(f"Invalid mode: {mode}")
    
    time.sleep_ms(100)

def drive(velocity, radius):
    """
    Drive command with validation
    velocity: -500 to 500 mm/s
    radius: -2000 to 2000 mm (32767=straight, 1=CCW spin, -1=CW spin)
    """
    # Clamp values to valid ranges
    velocity = max(-500, min(500, velocity))
    radius = max(-2000, min(2000, radius)) if abs(radius) < 10000 else radius
    
    velocity_bytes = struct.pack('>h', velocity)
    radius_bytes = struct.pack('>h', radius)
    
    send_command(OPCODE_DRIVE, list(velocity_bytes) + list(radius_bytes))
    
    # Visual feedback
    if velocity == 0:
        pico_led.on()
    else:
        pico_led.off()

def drive_direct(left_velocity, right_velocity):
    """
    Direct wheel control
    left_velocity, right_velocity: -500 to 500 mm/s
    """
    left_velocity = max(-500, min(500, left_velocity))
    right_velocity = max(-500, min(500, right_velocity))
    
    right_bytes = struct.pack('>h', right_velocity)
    left_bytes = struct.pack('>h', left_velocity)
    
    send_command(145, list(right_bytes) + list(left_bytes))

def stop():
    """Emergency stop"""
    drive(0, 0)

def set_leds(debris=False, spot=False, dock=False, check_robot=False,
             power_color=0, power_intensity=255):
    """
    Control Roomba LEDs
    power_color: 0=green, 255=red
    power_intensity: 0=off, 255=full
    """
    led_bits = 0
    if debris:
        led_bits |= 0x01
    if spot:
        led_bits |= 0x02
    if dock:
        led_bits |= 0x04
    if check_robot:
        led_bits |= 0x08
    
    send_command(OPCODE_LEDS, [led_bits, power_color, power_intensity])

def define_song(song_number, notes):
    """
    Define a song
    song_number: 0-4
    notes: list of tuples [(note_num, duration), ...]
           note_num: 31-127 (MIDI), duration: 1/64ths of second
    """
    if not 0 <= song_number <= 4:
        raise RoombaError("Song number must be 0-4")
    
    if len(notes) > 16:
        raise RoombaError("Max 16 notes per song")
    
    data = [song_number, len(notes)]
    for note, duration in notes:
        data.extend([note, duration])
    
    send_command(OPCODE_SONG, data)

def play_song(song_number):
    """Play a previously defined song"""
    if not 0 <= song_number <= 4:
        raise RoombaError("Song number must be 0-4")
    
    send_command(OPCODE_PLAY, [song_number])

def read_sensors(packet_id):
    """
    Request sensor data
    packet_id: 0-58 or 100-107 (see manual)
    Returns: raw bytes received
    """
    # Clear buffer
    while uart.any():
        uart.read()
    
    send_command(OPCODE_SENSORS, [packet_id])
    time.sleep_ms(50)  # Wait for response
    
    if uart.any():
        return uart.read()
    return None

def get_battery_status():
    """
    Get battery voltage, charge, and capacity
    Returns: dict with voltage (mV), charge (mAh), capacity (mAh)
    """
    # Request packet group 3 (battery info)
    data = read_sensors(3)
    
    if data and len(data) >= 10:
        # Parse the packet (see manual page 22-27)
        # Packet 3 contains: charging_state, voltage, current, temp, charge, capacity
        voltage = struct.unpack('>H', data[1:3])[0]
        charge = struct.unpack('>H', data[7:9])[0]
        capacity = struct.unpack('>H', data[9:11])[0] if len(data) >= 11 else 0
        
        return {
            "voltage": voltage,
            "charge": charge,
            "capacity": capacity
        }
    
    return None

def seek_dock():
    """Command Roomba to return to dock"""
    send_command(OPCODE_SEEK_DOCK)

def clean():
    """Start default cleaning cycle"""
    send_command(OPCODE_CLEAN)

def spot_clean():
    """Start spot cleaning"""
    send_command(OPCODE_SPOT)

def shutdown():
    """Safely shut down (stops motors, returns to passive)"""
    try:
        stop()
        time.sleep_ms(100)
        send_command(OPCODE_STOP)
        pico_led.off()
        print("Roomba safely shut down")
    except:
        pass