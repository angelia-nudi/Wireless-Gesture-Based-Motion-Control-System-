import smbus
import time
import csv

# MPU6050 Addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
GYRO_XOUT_H = 0x43
ACCEL_XOUT_H = 0x3B
TEMP_OUT_H = 0x41

# Initialize I2C
bus = smbus.SMBus(1)

# Initialize MPU6050
def init_mpu6050():
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# Read raw data
def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = (high << 8) | low
    if value > 32768:
        value -= 65536
    return value

# Get sensor data including internal temperature
def get_sensor_data():
    gx = read_raw_data(GYRO_XOUT_H) / 131.0
    gy = read_raw_data(GYRO_XOUT_H + 2) / 131.0
    gz = read_raw_data(GYRO_XOUT_H + 4) / 131.0
    ax = read_raw_data(ACCEL_XOUT_H) / 16384.0
    ay = read_raw_data(ACCEL_XOUT_H + 2) / 16384.0
    az = read_raw_data(ACCEL_XOUT_H + 4) / 16384.0
    temp_row = read_raw_data(TEMP_OUT_H)
    temp_c = temp_row / 340.0 + 36.53  # According to MPU6050 datasheet
    return gx, gy, gz, ax, ay, az, temp_c

# Initialize MPU6050 sensor
init_mpu6050()

# Data Collection Setup
filename = "imu_labeled_data.csv"

print("IMU Data Collection with Gesture Labels\nPress Ctrl+C to stop anytime.")

with open(filename, mode="w", newline="") as file:
    writer = csv.writer(file)
    writer.writerow(["Time (s)", "Label", "Gyro_X", "Gyro_Y", "Gyro_Z",
                     "Accel_X", "Accel_Y", "Accel_Z", "Temperature_C"])

    print("\nInitializing sensor stabilization (5 seconds)...")
    time.sleep(5)  # Allow sensor stabilization

    start_time = time.time()

    try:
        while True:
            # Ask user for gesture/activity label
            label = input("\nEnter activity label (e.g., SwipeLeft, Circle, Rest): ").strip()
            duration = float(input("Enter recording duration in seconds (e.g., 5): "))
            interval = 0.05  # Sampling at 20 Hz

            print(f"Start performing '{label}' for {duration}s...")

            record_start = time.time()
            while (time.time() - record_start) < duration:
                gx, gy, gz, ax, ay, az, temp_c = get_sensor_data()
                elapsed_time = time.time() - record_start  # Corrected timestamp

                writer.writerow([f"{elapsed_time:.3f}", label, 
                                 f"{gx:.3f}", f"{gy:.3f}", f"{gz:.3f}",
                                 f"{ax:.3f}", f"{ay:.3f}", f"{az:.3f}",
                                 f"{temp_c:.2f}"])

                print(f"[{elapsed_time:.2f}s] {label} | Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f}) | Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) | Temp: {temp_c:.2f}°C")

                time.sleep(interval)

            print(f"Completed recording '{label}'. You can now prepare for the next activity.")

    except KeyboardInterrupt:
        print("\nData collection stopped by user.")

print(f"\nLabeled IMU data saved to '{filename}'.")
