from gpiozero import Motor, AngularServo, RotaryEncoder
import smbus2
import time
import math
import threading

# === Hardware Setup ===
motor = Motor(19, 13)  # Motor control pins
servo = AngularServo(12, min_angle=0, max_angle=180,
                     min_pulse_width=0.0005, max_pulse_width=0.0025)
rotor = RotaryEncoder(17, 27, wrap=True, max_steps=18000)  # Encoder pins

# === Servo Steering Angles ===
mid = 110   # Straight ahead
maxa = 150  # Full left
mina = 70   # Full right

# === Encoder Settings ===
ds = 43  # Encoder steps per centimeter

# === MPU6050 Setup ===
bus = smbus2.SMBus(1)
mpu_address = 0x68
bus.write_byte_data(mpu_address, 0x6B, 0)  # Wake up MPU6050

# === Gyroscope Variables ===
latest_yaw = 0.0
gz_prev = 0.0
last_time = time.time()
drift_rate = 0.0
alpha = 0.95
lock = threading.Lock()

# === PID Settings ===
Kp = 2.0
Ki = 0.0
Kd = 0.5
integral = 0.0
previous_error = 0.0

# === Read 16-bit word from MPU6050 register ===
def read_word(reg):
    high = bus.read_byte_data(mpu_address, reg)
    low = bus.read_byte_data(mpu_address, reg + 1)
    val = (high << 8) + low
    return val - 65536 if val >= 0x8000 else val

# === Read Z-axis gyroscope value ===
def read_gyro():
    gz = read_word(0x47) / 131.0
    return gz

# === Calibrate gyro drift while robot is stationary ===
def calibrate_drift(duration=5.0):
    print(f"\n📢 Hold robot still for {duration} seconds to calibrate drift...")
    samples = []
    gz_prev_local = 0.0
    start_time = time.time()

    while time.time() - start_time < duration:
        gz = read_gyro()
        gz_filtered = alpha * gz_prev_local + (1 - alpha) * gz
        gz_prev_local = gz_filtered
        samples.append(gz_filtered)
        time.sleep(0.01)

    avg_drift = sum(samples) / len(samples)
    print(f"✔️ Drift rate calibrated: {avg_drift:.4f} deg/s\n")
    return avg_drift

# === Gyro thread: updates yaw at 200 Hz ===
def gyro_thread():
    global latest_yaw, gz_prev, last_time
    while True:
        gz = read_gyro()
        gz_filtered = alpha * gz_prev + (1 - alpha) * gz
        gz_prev = gz_filtered

        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        with lock:
            latest_yaw += (gz_filtered - drift_rate) * dt
            latest_yaw %= 360

        time.sleep(0.005)  # 200 Hz

# === Get latest yaw safely ===
def get_latest_yaw():
    with lock:
        return latest_yaw

# === PID correction based on yaw ===
def pid_steering(target_yaw):
    global integral, previous_error
    current_yaw = get_latest_yaw()

    # Circular error correction: result between -180 and +180
    error = (target_yaw - current_yaw + 540) % 360 - 180

    integral += error
    derivative = error - previous_error
    previous_error = error

    output = Kp * error + Ki * integral + Kd * derivative
    angle = mid - output  # Inverted signal for correct steering
    angle = max(min(angle, maxa), mina)
    servo.angle = angle

# === Drive forward 1 meter with PID heading correction ===
def drive(dist):
    rotor.steps = 0
    target_yaw = get_latest_yaw()  # Set current heading as reference
    motor.forward(1)
    while rotor.steps / ds < dist:
        pid_steering(target_yaw)
        time.sleep(0.01)
    motor.stop()
    print("✅ Finished driving 1 meter straight.")
    
def turnleft(speed):
    rotor.steps=0
    servo.angle=mina
    motor.forward(speed)

    while rotor.steps <3900:
       
        continue
    motor.stop()
# === Main Execution ===
drift_rate = calibrate_drift(duration=5.0)
threading.Thread(target=gyro_thread, daemon=True).start()

print("🚗 Starting straight drive...")
drive(100)
for i in range(11):
    turnleft(1)
    drive(120)
