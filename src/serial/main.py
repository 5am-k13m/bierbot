from server import *
from picozero import pico_led

# Main execution
try:
    ip = connect()
    if ip:
        connection = open_socket(ip)
        serve(connection)
except Exception as e:
    print(f"Fatal error: {e}")
finally:
    pico_led.off()