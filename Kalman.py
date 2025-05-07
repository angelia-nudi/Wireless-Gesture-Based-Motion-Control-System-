import smbus
import time
from time import sleep
import math

#some MPU6050 Registers and their Address
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


def MPU_Init():
	#write to sample rate register
	bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
	
	#Write to power management register
	bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
	
	#Write to Configuration register
	bus.write_byte_data(Device_Address, CONFIG, 0)
	
	#Write to Gyro configuration register
	bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
	
	#Write to interrupt enable register
	bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
    
        #concatenate higher and lower value
        value = ((high << 8) | low)
        
        #to get signed value from mpu6050
        if(value > 32768):
                value = value - 65536
        return value
    
def get_gyro_data():
    	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)
	Gx = gyro_x/131.0
	Gy = gyro_y/131.0
	Gz = gyro_z/131.0
	return Gx, Gy, Gz

def get_accel_data():
   	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)
	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0
	return Ax, Ay, Az

# Kalman Filter for Roll and Pitch
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance, estimate_error, initial_estimate=0):
        self.q = process_variance  # Process noise covariance
        self.r = measurement_variance  # Measurement noise covariance
        self.p = estimate_error  # Estimation error covariance
        self.x = initial_estimate  # Initial estimate

    def update(self, measurement):
        # Prediction update
        self.p = self.p + self.q
        
        # Measurement update
        k = self.p / (self.p + self.r)  # Kalman gain
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * self.p
        return self.x	


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

# Kalman filters for roll and pitch angles
roll_kalman = KalmanFilter(0.0001, 0.01, 1, 0)
pitch_kalman = KalmanFilter(0.0001, 0.01 , 1, 0)

prev_time = time.time() # Track time for integration


while True:
    # Read sensor data
    gx, gy, gz = get_gyro_data()
    ax, ay, az = get_accel_data()
    
    # Compute accelerometer roll and pitch
    accel_roll = math.degrees(math.atan2(ay, math.sqrt(ax ** 2 + az ** 2)))
    accel_pitch = math.degrees(math.atan2(-ax, math.sqrt(ay ** 2 + az ** 2)))

    # Time step calculation
    curr_time = time.time()
    dt = (curr_time - prev_time) / 1000.0  # Convert ms to seconds
    prev_time = curr_time

    # Integrate gyroscope data for roll and pitch
    gyro_roll = roll_kalman.x + gx * dt
    gyro_pitch = pitch_kalman.x + gy * dt

    # Apply Kalman Filter to fuse data
    roll = roll_kalman.update(accel_roll)
    pitch = pitch_kalman.update(accel_pitch)

    # Print filtered and unfiltered angles
    print(f"Roll: {roll:.2f}° (Filtered), Roll (Unfiltered): {accel_roll:.2f}°")
    print(f"Pitch: {pitch:.2f}° (Filtered), Pitch (Unfiltered): {accel_pitch:.2f}°")

    time.sleep(0.1)  # Adjust based on your application
