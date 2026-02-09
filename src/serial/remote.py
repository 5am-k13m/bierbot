"""
Roomba remote control with GUI (no status or sensor checking)
"""

import pygame
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


def draw_ui(screen, controller, font):
    screen.fill(BLACK)

    title = font.render("Roomba Remote Control", True, WHITE)
    screen.blit(title, (20, 20))

    status_color = GREEN if controller.connected else RED
    status_text = "CONNECTED" if controller.connected else "DISCONNECTED"
    status = font.render(f"Status: {status_text}", True, status_color)
    screen.blit(status, (20, 60))

    vel_text = font.render(f"Velocity: {controller.velocity} mm/s", True, WHITE)
    screen.blit(vel_text, (20, 100))

    rad_text = "STRAIGHT" if abs(controller.radius) > 10000 else f"{controller.radius} mm"
    radius_text = font.render(f"Radius: {rad_text}", True, WHITE)
    screen.blit(radius_text, (20, 130))

    controls = [
        "Arrow Keys: Drive",
        "Space: Stop",
        "S: Safe Mode",
        "F: Full Mode",
        "B: Beep",
        "L: Toggle Dock LED",
        "ESC: Quit"
    ]

    y = 200
    for control in controls:
        text = font.render(control, True, GRAY)
        screen.blit(text, (20, y))
        y += 25

    pygame.display.flip()


def main():
    pygame.init()

    screen = pygame.display.set_mode((640, 480))
    pygame.display.set_caption("Roomba Remote Control")

    font = pygame.font.Font(None, 28)
    clock = pygame.time.Clock()

    controller = RoombaController(SERVER_IP, SERVER_PORT)

    keys_pressed = set()
    dock_led = False
    last_reconnect = time.time()

    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                keys_pressed.add(event.key)

                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_SPACE:
                    controller.stop()
                elif event.key == pygame.K_s:
                    controller.set_mode("safe")
                elif event.key == pygame.K_f:
                    controller.set_mode("full")
                elif event.key == pygame.K_b:
                    controller.play_beep()
                elif event.key == pygame.K_l:
                    dock_led = not dock_led
                    controller.set_led(dock=dock_led)

            elif event.type == pygame.KEYUP:
                keys_pressed.discard(event.key)

        velocity = 0
        radius = STRAIGHT_RADIUS

        if pygame.K_UP in keys_pressed:
            velocity = FORWARD_SPEED
        elif pygame.K_DOWN in keys_pressed:
            velocity = -FORWARD_SPEED

        if pygame.K_LEFT in keys_pressed:
            radius = TURN_RADIUS
            if velocity == 0:
                velocity = FORWARD_SPEED // 2
        elif pygame.K_RIGHT in keys_pressed:
            radius = -TURN_RADIUS
            if velocity == 0:
                velocity = FORWARD_SPEED // 2

        if velocity != controller.velocity or radius != controller.radius:
            controller.drive(velocity, radius)

        if not controller.connected and time.time() - last_reconnect > RECONNECT_DELAY:
            print("Attempting reconnect...")
            controller.connect()
            last_reconnect = time.time()

        draw_ui(screen, controller, font)
        clock.tick(30)

    controller.disconnect()
    pygame.quit()


if __name__ == "__main__":
    main()
