import RPi.GPIO as GPIO
import time
import threading

# GPIO Setup
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Servo Configuration
servo_pin = 11
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)
pwm.start(0)

# Stepper Motor Configuration (Updated for proper rotation)
stepper_pins = [37, 36, 31, 29]
for pin in stepper_pins:
    GPIO.setup(pin, GPIO.OUT)

# Updated step sequence for proper rotation
step_sequence = [
    [1, 0, 0, 1],
    [1, 0, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 0, 1]
]

# Color Sensor Configuration (unchanged)
# ... [keep the same color sensor configuration as previous] ...

def set_angle(angle):
    duty = angle / 18 + 2.5
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)

def stepper_thread():
    step_counter = 0
    while stepper_running:
        for pin in range(4):
            GPIO.output(stepper_pins[pin], step_sequence[step_counter][pin])
        step_counter = (step_counter + 1) % 4
        time.sleep(0.002)  # Reduced delay for continuous rotation

# ... [keep the rest of the code unchanged] ...