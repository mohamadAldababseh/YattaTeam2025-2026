from gpiozero import Motor, AngularServo, RotaryEncoder
import smbus2
import time
import math
import threading
from colors import colors
from Sensors import DistanceSensor as ultra
# === Hardware Setup ===

a=colors()
threading.Thread(target=a.loop, daemon=True).start()
right=ultra(16,1)
left=ultra(5,6)
forward=ultra(23,24)
motor = Motor(19, 13)  # Motor control pins
servo = AngularServo(12, min_angle=0, max_angle=180,min_pulse_width=0.0005, max_pulse_width=0.0025)
servo.angle=(110)

rotor = RotaryEncoder(17, 27, wrap=True, max_steps=18000)  # Encoder pins

# === Servo Steering Angles ===
mid = 110   # Straight ahead
maxa = 133  # Full left
mina = 62   # Full right

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
Kp = 4.0
Ki = 0.0
Kd = 2
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
    kdd=2
    global integral, previous_error
    current_yaw = get_latest_yaw()
    r_distence=right.getDistance()
    l_distence=left.getDistance()
    distenceerror=l_distence-r_distence
    # Circular error correction: result between -180 and +180
    error = (target_yaw - current_yaw + 540) % 360 - 180
    error=error+(distenceerror)*kdd

    integral += error
    derivative = error - previous_error
    previous_error = error

    output = Kp * error + Ki * integral + Kd * derivative
    angle = mid - output  # Inverted signal for correct steering
    if angle >maxa:
        angle=maxa
    elif angle <mina:
        angle=mina
    
    servo.angle = angle

# === Drive forward 1 meter with PID heading correction ===
def drive(dist):
    rotor.steps = 0
    target_yaw = get_latest_yaw()  # Set current heading as reference
    motor.forward(1)
    while (not (a.orangedetected or a.bluedetected) ) or rotor.steps/ds<dist:
        if rotor.steps/ds<dist:
            a.reset()
        pid_steering(target_yaw)
        time.sleep(0.01)
    motor.stop()
    print("✅ Finished driving 1 meter straight.")
def drive2(dist):
    rotor.steps = 0
    target_yaw = get_latest_yaw()  # Set current heading as reference
    motor.forward(1)
    while rotor.steps / ds < dist:
        pid_steering(target_yaw)
        time.sleep(0.01)
    motor.stop()
def drive3(dist):
    rotor.steps = 0
    target_yaw = get_latest_yaw()  # Set current heading as reference
    motor.forward(1)
    while (not (a.orangedetected or a.bluedetected) )or rotor.steps/ds<dist:
        if roter.steps<1000:
            a.reset()
        pid_steering(target_yaw)
        time.sleep(0.01)
    motor.stop()
    print("✅ Finished driving 1 meter straight.")
    
def turnleft(speed):
    rotor.steps=0
    servo.angle=mina
    motor.forward(speed)

    while rotor.steps <3500:
       
        continue
    motor.stop()
def turnright(speed):
    rotor.steps=0
    servo.angle=maxa
    motor.forward(speed)

    while rotor.steps <4100:
       
        continue
    motor.stop()
    


# === Main Execution ===
drift_rate = calibrate_drift(duration=5.0)
threading.Thread(target=gyro_thread, daemon=True).start()
def cw_drive():
    turnright(1)
    for i in range(11):
        drive(20)
        turnright(1)
        
    drive2(30)


def ccw_drive():
    turnleft(1)
    for i in range(11):
        drive(20)
        turnleft(1)
        
    drive2(30)
    
print(" Starting straight drive...")


drive(20)
if a.orangedetected==True:
    cw_drive()
else:
    cw_drive()
    


