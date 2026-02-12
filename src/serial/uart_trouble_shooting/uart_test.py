import machine
import time
from picozero import pico_led

# Initialize UART0 on GP12/GP13
uart = machine.UART(0, baudrate=115200, 
                    tx=machine.Pin(16), rx=machine.Pin(17))

print("=== LOOPBACK TEST ===")
print("Connect GP4 to GP5 with a jumper wire")
input("Press Enter when connected...")

# Clear buffer
while uart.any():
    uart.read()

# Send test message
test_msg = b"HELLO UART!"
print(f"\nSending: {test_msg}")
uart.write(test_msg)
time.sleep_ms(50)

# Check if we received it back
if uart.any():
    received = uart.read()
    print(f"Received: {received}")
    
    if received == test_msg:
        print("\n✓✓✓ UART IS WORKING PERFECTLY! ✓✓✓")
        print("The TX pin IS transmitting data!")
    else:
        print("Data mismatch - possible issue")
else:
    print("No data received - possible issue")

print("\nRemove jumper and reconnect to Roomba")
