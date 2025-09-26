import smbus2
import time
import math

# I2C
bus = smbus2.SMBus(1)
mpu_address = 0x68
bus.write_byte_data(mpu_address, 0x6B, 0)  


yaw = 0.0
gz_prev = 0.0
yaw_buffer = []
last_time = time.time()
drift_rate = 0.0 


alpha = 0.95  
buffer_size = 10

def read_word(reg):
    high = bus.read_byte_data(mpu_address, reg)
    low = bus.read_byte_data(mpu_address, reg + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

def read_mpu6050():
    ax = read_word(0x3B) / 16384.0
    ay = read_word(0x3D) / 16384.0
    az = read_word(0x3F) / 16384.0
    gz = read_word(0x47) / 131.0
    return ax, ay, az, gz

def calibrate_drift(duration=5.0):
    print(f"\n📢 Don't move your robot {duration} calibration duration...")
    samples = []
    start_time = time.time()
    gz_prev_local = 0.0

    while time.time() - start_time < duration:
        _, _, _, gz = read_mpu6050()
        gz_filtered = alpha * gz_prev_local + (1 - alpha) * gz
        gz_prev_local = gz_filtered
        samples.append(gz_filtered)
        time.sleep(0.01)

    avg_drift = sum(samples) / len(samples)
    print(f"✔️6050 have been calibrated: {avg_drift:.4f} deg/s\n")
    return avg_drift

def get_corrected_yaw():
    global yaw, gz_prev, yaw_buffer, last_time, drift_rate

    _, _, _, gz = read_mpu6050()
    gz_filtered = alpha * gz_prev + (1 - alpha) * gz
    gz_prev = gz_filtered

    current_time = time.time()
    dt = current_time - last_time
    last_time = current_time

    
    corrected_gz = gz_filtered - drift_rate
    yaw += corrected_gz * dt
    yaw %= 360

   
    yaw_buffer.append(yaw)
    if len(yaw_buffer) > buffer_size:
        yaw_buffer.pop(0)
    yaw_smoothed = sum(yaw_buffer) / len(yaw_buffer)

    return yaw_smoothed


drift_rate = calibrate_drift(duration=5.0)


print("wait 5 second to calibrate your mpu6050")
while True:
    filtered_yaw = get_corrected_yaw()
    print(f"Yaw: {filtered_yaw:.2f}°")
    time.sleep(0.001)

