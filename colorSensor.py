import RPi.GPIO as GPIO
import time
import threading

# GPIO Pins Setup
# Color Sensor Pins
s0 = 13
s1 = 15
s2 = 16
s3 = 18
sig = 22

# Stepper Motor Pins
stepper_pins = [29, 31, 33, 35]  # Coil pins 1-4

# Servo Motor Pin
servo_pin = 32

# Global Variables
cycles = 10
running = True  # Controls stepper motor thread
color_angles = {
    "Red": 0,
    "Green": 60,
    "Blue": 120,
    "Yellow": 180
}

# Setup GPIO
GPIO.setmode(GPIO.BOARD)

# Color Sensor Setup
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Stepper Motor Setup
for pin in stepper_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Servo Motor Setup
GPIO.setup(servo_pin, GPIO.OUT)
pwm = GPIO.PWM(servo_pin, 50)  # 50 Hz frequency
pwm.start(0)

# Set frequency scaling for color sensor
GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

def measure_frequency():
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    return cycles / duration

def detect_color():
    # Detect red
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    red = measure_frequency()

    # Detect blue
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    blue = measure_frequency()

    # Detect green
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    green = measure_frequency()

    print(f"Red: {red}, Blue: {blue}, Green: {green}")
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

def set_servo_angle(angle):
    duty = angle / 18 + 2.5
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)  # Prevent jitter

def run_stepper():
    step_sequence = [
        (1, 0, 0, 1),
        (1, 0, 0, 0),
        (1, 1, 0, 0),
        (0, 1, 0, 0),
        (0, 1, 1, 0),
        (0, 0, 1, 0),
        (0, 0, 1, 1),
        (0, 0, 0, 1)
    ]
    current_step = 0
    delay = 0.01  # Control speed (larger = slower)

    while running:
        pins = step_sequence[current_step]
        for i in range(len(stepper_pins)):
            GPIO.output(stepper_pins[i], pins[i])
        time.sleep(delay)
        current_step = (current_step + 1) % 8

try:
    # Start stepper motor thread
    stepper_thread = threading.Thread(target=run_stepper)
    stepper_thread.start()

    while True:
        # Detect and classify color
        red, blue, green = detect_color()
        detected_color = classify_color(red, blue, green)
        print(f"Detected Color: {detected_color}")

        # Control servo based on color
        if detected_color in color_angles:
            set_servo_angle(color_angles[detected_color])
            time.sleep(1)  # Allow ball to pass
            set_servo_angle(0)  # Return to rest position

        time.sleep(0.5)  # Short delay between readings

except KeyboardInterrupt:
    running = False
    stepper_thread.join()
    pwm.stop()
    GPIO.cleanup()
    print("Program terminated")