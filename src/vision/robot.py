"""
Class which handles moving the robot along a given path
"""

import numpy as np
import time
import socket
import threading
import time
from serial.protocol import Commands, Message

# Configuration
SERVER_IP = "192.168.1.170"
SERVER_PORT = 80
RECONNECT_DELAY = 3.0

# Control settings
FORWARD_SPEED = 200  # mm/s
TURN_RADIUS = 1      # 1 = turn in place
STRAIGHT_RADIUS = 32767

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY = (128, 128, 128)

class RoombaController:
    def __init__(self, server_ip, server_port):
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        self.connected = False
        self.running = True
        self.in_motion = False

        # Drive state
        self.velocity = 0
        self.radius = STRAIGHT_RADIUS

        self.receive_thread = None
        self.connect()

    def connect(self):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.server_ip, self.server_port))
            self.socket.settimeout(1.0)

            data = self.socket.recv(1024)
            msg = Message.decode(data)
            print(f"Connected: {msg}")

            self.connected = True

            self.receive_thread = threading.Thread(
                target=self._receive_loop, daemon=True
            )
            self.receive_thread.start()

            return True

        except Exception as e:
            print(f"Connection failed: {e}")
            self.connected = False
            return False

    def _receive_loop(self):
        buffer = b""

        while self.running and self.connected:
            try:
                chunk = self.socket.recv(1024)
                if not chunk:
                    print("Server disconnected")
                    self.connected = False
                    break

                buffer += chunk
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    msg = Message.decode(line)

                    # Only log unexpected responses
                    if msg.get("type") not in ["ok", "pong"]:
                        print(f"Response: {msg}")

            except socket.timeout:
                continue
            except Exception as e:
                print(f"Receive error: {e}")
                self.connected = False
                break

    def send_command(self, command):
        if not self.connected:
            return False
        try:
            self.socket.sendall(command)
            return True
        except Exception as e:
            print(f"Send error: {e}")
            self.connected = False
            return False

    def drive(self, velocity, radius):
        self.velocity = velocity
        self.radius = radius
        return self.send_command(Commands.drive(velocity, radius))

    def stop(self):
        self.velocity = 0
        return self.send_command(Commands.stop())

    def set_mode(self, mode):
        return self.send_command(Commands.set_mode(mode))

    def set_led(self, **kwargs):
        return self.send_command(Commands.set_led(**kwargs))

    def play_beep(self):
        self.send_command(Commands.define_song(0, [(72, 16)]))
        time.sleep(0.05)
        self.send_command(Commands.play_song(0))

    def disconnect(self):
        self.running = False
        self.stop()
        time.sleep(0.1)

        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except:
                pass

        self.connected = False

class Robot():
    def __init__(self, velocity=100, reconnect_time = 3.0):
        self.position = None
        self.facing_direction = None
        self.velocity = velocity

        self.controller = RoombaController(server_ip="192.168.1.170",
                                           server_port=80)
        self.reconnect_time = reconnect_time

    def reconnect(self):
        last_reconnect = time.time()
        while not self.controller.connected and time.time() - last_reconnect > self.reconnect_time:
            print("Attempting reconnect...")
            self.controller.connect()
            last_reconnect = time.time()
    
    def calibrate_roomba_facing_direction(self, position_1, position_2):
        direction = (position_2 - position_1)
        self.facing_direction = direction / np.linalg.norm(direction)

    def get_rotation_time(self, theta):
        """
        theta: radians
        velocity: mm/s
        """
        WHEEL_BASE = 235.0  # mm
        return (theta * WHEEL_BASE) / (2.0 * self.velocity)
    
    def point_roomba_toward_point(self, point, error=(np.pi * 5) / 180):
        centered_point = point - self.position
        point_norm = centered_point / np.linalg.norm(centered_point)

        angle = np.arccos(
            np.clip(np.dot(self.facing_direction, point_norm), -1.0, 1.0)
        )

        if angle < error:
            return

        rot_time = self.get_rotation_time(angle)
        print(f"angle(rad)={angle:.3f}, time(s)={rot_time:.2f}")

        # Spin in place
        self.controller.drive(velocity=self.velocity, radius=1)
        time.sleep(rot_time)
        self.controller.stop()

        self.facing_direction = point_norm
        