import RPi.GPIO as GPIO
import time
import threading

#########################
# GPIO Setup and Config #
#########################

# Use the BOARD numbering scheme
GPIO.setmode(GPIO.BOARD)

#########################
# Servo Setup           #
#########################

# Servo connected pin (controls the sliding mechanism)
servo_pin = 11
GPIO.setup(servo_pin, GPIO.OUT)
# Create a PWM object for the servo at 50 Hz (typical frequency for servos)
servo_pwm = GPIO.PWM(servo_pin, 50)
servo_pwm.start(0)  # Start at 0% duty cycle (rest position)

# Define servo positions (in degrees) for each color.
# Adjust these values to match the physical positions of your slide.
SERVO_REST   = 0    # Resting position
SERVO_RED    = 45   # Position for Red collection
SERVO_GREEN  = 90   # Position for Green collection
SERVO_BLUE   = 135  # Position for Blue collection
SERVO_YELLOW = 180  # Position for Yellow collection

def set_servo_angle(angle):
    """
    Set the servo to a specific angle.
    The duty cycle is calculated from the angle.
    """
    # This conversion factor may vary depending on your servo.
    duty = angle / 18.0 + 2.5
    servo_pwm.ChangeDutyCycle(duty)
    # Allow time for the servo to begin moving
    time.sleep(0.5)

def drop_ball(color):
    """
    Move the servo to the appropriate position for the detected color,
    wait for the ball to drop, then return the servo to the rest position.
    """
    print(f"Activating servo for {color}")
    
    if color == "Red":
        set_servo_angle(SERVO_RED)
    elif color == "Green":
        set_servo_angle(SERVO_GREEN)
    elif color == "Blue":
        set_servo_angle(SERVO_BLUE)
    elif color == "Yellow":
        set_servo_angle(SERVO_YELLOW)
    else:
        # For any unknown color, do not move the servo (remain at rest)
        print("Unknown color detected, servo remains at rest.")
        return

    # Wait to allow the ball to fall through the slide
    time.sleep(1)
    # Return servo to its rest position
    set_servo_angle(SERVO_REST)
    # Allow a brief pause before handling the next ball
    time.sleep(0.5)

#########################
# Stepper Motor Setup   #
#########################

# Define the GPIO pins for the stepper motor
stepper_pins = [37, 36, 31, 29]
for pin in stepper_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# Define a simple step sequence (one coil on at a time)
step_sequence = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

def step_stepper(step):
    """
    Activate the stepper motor coils for the given step.
    """
    for i in range(4):
        GPIO.output(stepper_pins[i], step_sequence[step][i])

def run_stepper():
    """
    Continuously run the stepper motor slowly.
    This function is intended to run in its own thread.
    """
    step = 0
    try:
        while True:
            step_stepper(step)
            # Delay controls the speed; adjust as needed.
            time.sleep(0.01)
            step = (step + 1) % 4
    except Exception as e:
        print("Stepper motor thread exception:", e)

#########################
# Color Sensor Setup    #
#########################

# GPIO pins for the color sensor
s0 = 13
s1 = 15
s2 = 16
s3 = 18
sig = 22

# Setup pins for the color sensor
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Set frequency scaling (here we use 20% scaling as an example)
GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

cycles = 10  # Number of signal cycles to count for frequency measurement

def measure_frequency():
    """
    Count a fixed number of cycles from the sensor output and return the frequency.
    """
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    if duration == 0:
        return 0
    return cycles / duration

def detect_color():
    """
    Take frequency measurements for red, blue, and green filters.
    Returns a tuple (red, blue, green)
    """
    # Measure red frequency
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    red = measure_frequency()

    # Measure blue frequency
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    blue = measure_frequency()

    # Measure green frequency
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    green = measure_frequency()

    # Debug: Print raw frequency values
    print(f"Sensor frequencies - Red: {red:.2f}, Blue: {blue:.2f}, Green: {green:.2f}")
    return red, blue, green

def classify_color(red, blue, green):
    """
    Classify the detected color based on sensor frequency readings.
    The thresholds and comparisons below are examples – you may need to
    calibrate these values for your sensor and ambient lighting conditions.
    """
    # A simple classification based on relative sensor values.
    if red > green and red > blue:
        # If green is not too low compared to red, it might be yellow.
        if green > 0.6 * red:
            return "Yellow"
        return "Red"
    elif green > red and green > blue:
        return "Green"
    elif blue > red and blue > green:
        return "Blue"
    else:
        # Return "Unknown" if no clear majority is detected.
        return "Unknown"

#########################
# Main Program          #
#########################

def main():
    # Start the stepper motor in a separate thread.
    stepper_thread = threading.Thread(target=run_stepper, daemon=True)
    stepper_thread.start()

    # Initialize servo to its rest position at startup.
    set_servo_angle(SERVO_REST)

    try:
        while True:
            # Read the color sensor
            red_val, blue_val, green_val = detect_color()
            detected = classify_color(red_val, blue_val, green_val)
            print(f"Detected Color: {detected}")

            # Only drop the ball if the detected color is one of the four target colors.
            if detected in ["Red", "Green", "Blue", "Yellow"]:
                drop_ball(detected)
            else:
                # For any other color, do nothing and keep the servo at rest.
                print("No valid color detected. Servo remains at rest.")
                time.sleep(0.5)

            # Small delay before next reading
            time.sleep(0.5)

    except KeyboardInterrupt:
        print("Interrupted by user. Cleaning up GPIO...")

    finally:
        # Stop the servo PWM and clean up GPIO pins.
        servo_pwm.stop()
        GPIO.cleanup()

if __name__ == '__main__':
    main()
