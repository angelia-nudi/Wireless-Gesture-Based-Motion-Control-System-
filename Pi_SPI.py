import smbus
import time
import math
import struct
import spidev
import RPi.GPIO as GPIO

# === MPU6050 Constants ===
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# === SPI Handshake GPIOs ===
CS_PIN = 8              # Manual CS
CONTROLLER_READY = 27   # Pi → TI
PERIPHERAL_READY = 17   # TI → Pi

# === MPU6050 Setup ===
bus = smbus.SMBus(1)
Device_Address = 0x68

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

def get_gyro_data():
    gx = read_raw_data(GYRO_XOUT_H) / 131.0
    gy = read_raw_data(GYRO_YOUT_H) / 131.0
    gz = read_raw_data(GYRO_ZOUT_H) / 131.0
    return gx, gy, gz

def get_accel_data():
    ax = read_raw_data(ACCEL_XOUT_H) / 16384.0
    ay = read_raw_data(ACCEL_YOUT_H) / 16384.0
    az = read_raw_data(ACCEL_ZOUT_H) / 16384.0
    return ax, ay, az

# === Kalman Filter Class ===
class KalmanFilter:
    def __init__(self, q, r, p, initial_estimate=0):
        self.q = q
        self.r = r
        self.p = p
        self.x = initial_estimate

    def update(self, measurement):
        self.p += self.q
        k = self.p / (self.p + self.r)
        self.x += k * (measurement - self.x)
        self.p *= (1 - k)
        return self.x

# === Init MPU & Filters ===
MPU_Init()
roll_kalman = KalmanFilter(0.001, 0.1, 1, 0)
pitch_kalman = KalmanFilter(0.001, 0.1, 1, 0)
prev_time = time.time()

# === SPI Setup ===
spi = spidev.SpiDev()
spi.open(0, 0)
spi.no_cs = True
spi.mode = 0b01
spi.max_speed_hz = 100000

# === GPIO Setup ===
GPIO.setmode(GPIO.BCM)
GPIO.setup(CS_PIN, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(CONTROLLER_READY, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(PERIPHERAL_READY, GPIO.IN)

# === Initial Handshake ===
print("Waiting for TI to raise PERIPHERAL_READY...")
while GPIO.input(PERIPHERAL_READY) == GPIO.LOW:
    time.sleep(0.1)
print("TI is ready ✔️")

GPIO.output(CONTROLLER_READY, GPIO.HIGH)
time.sleep(0.5)
GPIO.output(CONTROLLER_READY, GPIO.LOW)
print("Handshake complete — entering data loop")

# === MAIN LOOP ===
try:
    while True:
        # === Read MPU and filter ===
        gx, gy, gz = get_gyro_data()
        ax, ay, az = get_accel_data()

        accel_roll = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
        accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))

        curr_time = time.time()
        dt = curr_time - prev_time
        prev_time = curr_time

        gyro_roll = roll_kalman.x + gx * dt
        gyro_pitch = pitch_kalman.x + gy * dt

        roll = roll_kalman.update(accel_roll)
        pitch = pitch_kalman.update(accel_pitch)

        print(f"Roll: {roll:.2f}, Pitch: {pitch:.2f}")

        # === Wait for TI to pull PERIPHERAL_READY LOW ===
        timeout = 2
        start = time.time()
        while GPIO.input(PERIPHERAL_READY) == GPIO.HIGH:
            if time.time() - start > timeout:
                print("Timeout: TI not ready, skipping this sample.")
                break
            time.sleep(0.01)

        if GPIO.input(PERIPHERAL_READY) == GPIO.HIGH:
            continue

        # === Pack and Send ===
        packet = struct.pack('<ff', roll, pitch)
        packet += b'\x00' * (32 - len(packet))

        GPIO.output(CS_PIN, GPIO.LOW)
        time.sleep(0.001)
        response = spi.xfer2(list(packet))
        time.sleep(0.001)
        GPIO.output(CS_PIN, GPIO.HIGH)

        # === Wait for TI to raise PERIPHERAL_READY HIGH again ===
        while GPIO.input(PERIPHERAL_READY) == GPIO.LOW:
            time.sleep(0.01)

        time.sleep(0.1)

finally:
    GPIO.output(CONTROLLER_READY, GPIO.LOW)
    GPIO.output(CS_PIN, GPIO.HIGH)
    spi.close()
    GPIO.cleanup()
    print("Shutdown cleanly.")
