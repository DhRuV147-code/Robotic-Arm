import time
import board
import math
import busio
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo

# Initialize I2C bus and PCA9685 module
i2c = busio.I2C(board.SCL, board.SDA)
pca = PCA9685(i2c)
pca.frequency = 50

px, py, pz = 175, 0, 120
wx, wy, wz = px, py, pz + 150
d1 = 40  # length of base
a2 = 140  # length of shoulder
a3 = 125  # length of elbow
r = math.sqrt(wx**2 + wy**2)
d = math.sqrt(r**2 + wz**2)

# Initialize the six servos on their respective channels
servo_base = servo.Servo(pca.channels[0])
servo_shoulder = servo.Servo(pca.channels[1])
servo_wrist = servo.Servo(pca.channels[2])
servo_gripper_roll = servo.Servo(pca.channels[3])
servo_gripper_pitch = servo.Servo(pca.channels[4])
servo_gripper_open_close = servo.Servo(pca.channels[5])

# Function to smoothly move the servos
def move_servo_smooth(servo_motor, target_angle, step_delay=0.01):
    current_angle = servo_motor.angle
    if current_angle is None:
        current_angle = 90  # Default to 90 degrees if undefined

    step_size = 1 if target_angle > current_angle else -1

    while abs(target_angle - current_angle) > abs(step_size):
        current_angle += step_size
        servo_motor.angle = current_angle
        time.sleep(step_delay)

    servo_motor.angle = target_angle

# Function to move the arm to a specific position using smooth function
def move_arm_to_position(angles):
    move_servo_smooth(servo_base, angles[0])
    move_servo_smooth(servo_shoulder, angles[1])
    move_servo_smooth(servo_wrist, angles[2])
    move_servo_smooth(servo_gripper_roll, angles[3])
    move_servo_smooth(servo_gripper_pitch, angles[4])
    move_servo_smooth(servo_gripper_open_close, angles[5])
    time.sleep(1)  # Small delay after movement

# Calculate q1 using atan2 for safety
q1 = math.atan2(wy, wx)

# Clamp the value for acos to be within the range [-1, 1] for q3
cos_q3 = (a2**2 + a3**2 - d**2) / (2 * a2 * a3)
cos_q3 = max(-1, min(1, cos_q3))  # Clamping
q3 = math.pi - ((math.pi * 51) / 180) - math.acos(cos_q3)

# Clamp the value for acos to be within the range [-1, 1] for q2
cos_q2 = (a2**2 + d**2 - a3**2) / (2 * a2 * d)
cos_q2 = max(-1, min(1, cos_q2))  # Clamping
q2 = math.acos(cos_q2) + math.atan2(wz, d) - ((math.pi * 51) / 180)

# Convert results to degrees
q11 = ((q1 * 180) / math.pi + 80)
q12 = ((q2 * 180) / math.pi + 120)
q13 = ((q3 * 180) / math.pi + 125)

# Angles array: [base, shoulder, wrist, roll, pitch, gripper]

home_position_angles =   [90, 50, 20, 150, 50, 0]
pickup_position_angles = [q11, q12, q13, 150, 180, 0]
place_position_angles = [170, 70, 30, 150, 120, 60]

# Function to perform the entire task sequence
def perform_pick_and_place():
    try:
        print("Moving to home position...")
        move_arm_to_position(home_position_angles)

        time.sleep(1)

        move_servo_smooth(servo_gripper_open_close, 70)
        move_servo_smooth(servo_wrist, 40)

        print("Moving to the pickup position...")
        move_arm_to_position(pickup_position_angles)


        print("Closing gripper to pick up object...")
        move_servo_smooth(servo_gripper_open_close, 0)

        time.sleep(1)

        print("Moving shoulder...")
        move_servo_smooth(servo_shoulder, 50)

        print("Placing the object...")
        move_arm_to_position(place_position_angles)

        time.sleep(1)

        print("Moving to home position...")
        move_arm_to_position(home_position_angles)

    except KeyboardInterrupt:
        print("\nProcess interrupted by the user.")
        move_arm_to_position(home_position_angles)  # Return to home position if interrupted
    except Exception as e:
        print(f"An error occurred: {e}")
        move_arm_to_position(home_position_angles)  # Return to home position in case of an error
    finally:
        pca.deinit()  # Safely deinitialize the PCA9685 module

# Main execution
if __name__ == "__main__":
    perform_pick_and_place()
    print("Pick and place sequence complete.")