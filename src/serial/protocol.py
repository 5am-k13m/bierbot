"""
Shared protocol definitions for Roomba wireless control
Used on both Pico (MicroPython) and laptop (Python)
"""

import json

# Command types
CMD_DRIVE = "drive"
CMD_STOP = "stop"
CMD_MODE = "mode"
CMD_LED = "led"
CMD_SENSORS = "sensors"
CMD_SONG = "song"
CMD_PLAY_SONG = "play_song"
CMD_PING = "ping"
CMD_STATUS = "status"

# Response types
RESP_OK = "ok"
RESP_ERROR = "error"
RESP_SENSOR_DATA = "sensor_data"
RESP_STATUS = "status"
RESP_PONG = "pong"

# Mode values
MODE_SAFE = "safe"
MODE_FULL = "full"
MODE_PASSIVE = "passive"

class Message:
    """Base class for protocol messages"""
    
    @staticmethod
    def encode(msg_type, **kwargs):
        """Encode a message to JSON bytes"""
        message = {"type": msg_type, **kwargs}
        return (json.dumps(message) + "\n").encode('utf-8')
    
    @staticmethod
    def decode(data):
        """Decode JSON bytes to message dict"""
        try:
            return json.loads(data.decode('utf-8').strip())
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            return {"type": "error", "message": f"Decode error: {e}"}

# Command constructors
class Commands:
    @staticmethod
    def drive(velocity, radius):
        return Message.encode(CMD_DRIVE, velocity=velocity, radius=radius)
    
    @staticmethod
    def stop():
        return Message.encode(CMD_STOP)
    
    @staticmethod
    def set_mode(mode):
        return Message.encode(CMD_MODE, mode=mode)
    
    @staticmethod
    def set_led(debris=False, spot=False, dock=False, check_robot=False, 
                power_color=0, power_intensity=255):
        return Message.encode(CMD_LED, 
                            debris=debris, spot=spot, dock=dock, 
                            check_robot=check_robot,
                            power_color=power_color, 
                            power_intensity=power_intensity)
    
    @staticmethod
    def request_sensors(packet_ids):
        """Request sensor data. packet_ids is a list like [7, 8, 9]"""
        return Message.encode(CMD_SENSORS, packet_ids=packet_ids)
    
    @staticmethod
    def define_song(song_num, notes):
        """notes is list of tuples: [(note_num, duration), ...]"""
        return Message.encode(CMD_SONG, song_num=song_num, notes=notes)
    
    @staticmethod
    def play_song(song_num):
        return Message.encode(CMD_PLAY_SONG, song_num=song_num)
    
    @staticmethod
    def ping():
        return Message.encode(CMD_PING)
    
    @staticmethod
    def request_status():
        return Message.encode(CMD_STATUS)

# Response constructors
class Responses:
    @staticmethod
    def ok(message=""):
        return Message.encode(RESP_OK, message=message)
    
    @staticmethod
    def error(message):
        return Message.encode(RESP_ERROR, message=message)
    
    @staticmethod
    def sensor_data(packet_id, value):
        return Message.encode(RESP_SENSOR_DATA, packet_id=packet_id, value=value)
    
    @staticmethod
    def status(battery_voltage=None, battery_charge=None, 
               battery_capacity=None, mode=None):
        return Message.encode(RESP_STATUS,
                            battery_voltage=battery_voltage,
                            battery_charge=battery_charge,
                            battery_capacity=battery_capacity,
                            mode=mode)
    
    @staticmethod
    def pong():
        return Message.encode(RESP_PONG)