import smbus
import time

# Define I2C bus and TI board's slave address
I2C_BUS = 1  # Use bus 1 for Raspberry Pi
SLAVE_ADDRESS = 0x30  # Must match TI's I2C_SOAR value

# Initialize I2C bus
bus = smbus.SMBus(I2C_BUS)

try:
    while True:
        # Read one byte from the TI board
        try:
            received_byte = bus.read_byte(SLAVE_ADDRESS)
            print(f"Received byte from TI Board: 0x{received_byte:02X}")
        except OSError:
            print("Error: No response from the TI Board. Is it connected?")

        time.sleep(1)  # Delay for readability

except KeyboardInterrupt:
    print("\nExiting program.")
    bus.close()
