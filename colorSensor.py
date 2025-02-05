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

# Stepper Motor Configuration
stepper_pins = [37, 36, 31, 29]
for pin in stepper_pins:
    GPIO.setup(pin, GPIO.OUT)

step_sequence = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

# Color Sensor Configuration
s0 = 13
s1 = 15
s2 = 16
s3 = 18
sig = 22
cycles = 10

GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

# Global variables
stepper_running = True
color_angles = {
    "Red": 0,
    "Green": 60,
    "Blue": 120,
    "Yellow": 180
}
rest_angle = 90  # Default resting position

def set_angle(angle):
    duty = angle / 18 + 2.5
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)  # Allow servo to reach position

def stepper_thread():
    step_counter = 0
    while stepper_running:
        for pin in range(4):
            GPIO.output(stepper_pins[pin], step_sequence[step_counter][pin])
        step_counter = (step_counter + 1) % 4
        time.sleep(0.1)  # Adjust speed here (larger = slower)

def measure_frequency():
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    return cycles / duration

def detect_color():
    # Red measurement
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    red = measure_frequency()

    # Blue measurement
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    blue = measure_frequency()

    # Green measurement
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    green = measure_frequency()

    return red, blue, green

def classify_color(red, blue, green):
    if red > green and red > blue:
        if green > 0.6 * red:
            return "Yellow"
        return "Red"
    elif green > red and green > blue:
        return "Green"
    elif blue > red and blue > green:
        return "Blue"
    return "Unknown"

try:
    # Start stepper motor thread
    stepper = threading.Thread(target=stepper_thread)
    stepper.start()

    # Initial position
    set_angle(rest_angle)

    while True:
        red, blue, green = detect_color()
        color = classify_color(red, blue, green)
        
        if color in color_angles:
            print(f"Detected {color} - Sorting...")
            # Move to color-specific position
            set_angle(color_angles[color])
            # Return to resting position
            set_angle(rest_angle)

except KeyboardInterrupt:
    stepper_running = False
    stepper.join()
    pwm.stop()
    GPIO.cleanup()
    print("Program terminated")