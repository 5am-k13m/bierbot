import socket
import threading
import json
import time
import sys

DISPENSER_SERVER_IP = "192.168.1.190"
SERVER_PORT = 80
RECONNECT_DELAY = 3.0

server_socket = None
connected = False
receive_thread = None

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

def connect():
    global connected, server_socket
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.settimeout(5.0)
        server_socket.connect((DISPENSER_SERVER_IP, SERVER_PORT))
        server_socket.settimeout(1.0)

        data = server_socket.recv(1024)
        msg = Message.decode(data)
        print(f"Connected: {msg}")

        connected = True

        receive_thread = threading.Thread(
            target=_receive_loop, daemon=True
        )
        receive_thread.start()

        return True

    except Exception as e:
        print(f"Connection failed: {e}")
        connected = False
        return False
        
def _receive_loop():
    global connected
    buffer = b""

    while connected:
        try:
            chunk = server_socket.recv(1024)
            if not chunk:
                print("Server disconnected")
                connected = False
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
            connected = False
            break
        
def dispense():
    global connected, server_socket

    if not connected:
        return False
    
    try:
        command = Message.encode("dispense")
        server_socket.sendall(command)
        return True
    except Exception as e:
        print(f"Send error: {e}")
        connected = False
        return False 
    
connect()

while True:
    exit = input("Dispense?")
    if exit == 'n':
        running = False
        time.sleep(0.1)

        if server_socket:
            try:
                server_socket.shutdown(socket.SHUT_RDWR)
                server_socket.close()
            except:
                pass

        connected = False
        sys.exit()
    dispense()