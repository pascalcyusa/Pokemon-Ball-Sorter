import RPi.GPIO as GPIO
import time

# Assign GPIO pin numbers to variables

s0 = 13
s1 = 15
s2 = 16
s3 = 18
sig = 22 #labeled "out" on your board
cycles = 10

# Motor Code
# Define the GPIO pins for the L298N motor driver
OUT1 = 37
OUT2 = 36
OUT3 = 31
OUT4 = 29
#pin for the servo motor
servo_pin = 11

# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Setup Stepper Motor Pins as Output
GPIO.setup(OUT1, GPIO.OUT)
GPIO.setup(OUT2, GPIO.OUT)
GPIO.setup(OUT3, GPIO.OUT)
GPIO.setup(OUT4, GPIO.OUT)

# Initialize all stepper motor outputs to LOW
GPIO.output(OUT1, GPIO.LOW)
GPIO.output(OUT2, GPIO.LOW)
GPIO.output(OUT3, GPIO.LOW)
GPIO.output(OUT4, GPIO.LOW)

# Setup Servo Motor Pin as Output
GPIO.setup(servo_pin, GPIO.OUT)

# Initialize Servo PWM after setting it as an output
servo = GPIO.PWM(servo_pin, 50)  # 50Hz PWM for the servo
servo.start(0)  # Now it is safe to start PWM

# Set frequency scaling for the color sensor
GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

step_sequence = [
    [1, 0, 0, 1],  
    [1, 0, 0, 0],  
    [1, 1, 0, 0],  
    [0, 1, 0, 0],  
    [0, 1, 1, 0],  
    [0, 0, 1, 0],  
    [0, 0, 1, 1],  
    [0, 0, 0, 1] 
]

def measure_frequency():
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    return cycles / duration

def DetectColor():
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    time.sleep(0.1)
    red = measure_frequency()

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    time.sleep(0.1)
    blue = measure_frequency()

    # Detect green values
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

def move_stepper(steps=100, delay=0.003, direction="forward"):
    """
    Moves the stepper motor forward or backward for a given number of steps.
    """
    if direction == "backward":
        sequence = step_sequence[::-1]  # Reverse the sequence for backward movement
    else:
        sequence = step_sequence

    for _ in range(steps):
        for step in sequence:
            GPIO.output(OUT1, step[0])
            GPIO.output(OUT2, step[1])
            GPIO.output(OUT3, step[2])
            GPIO.output(OUT4, step[3])
            time.sleep(delay)
    
    # Turn off all coils to stop motor
    GPIO.output(OUT1, 0)
    GPIO.output(OUT2, 0)
    GPIO.output(OUT3, 0)
    GPIO.output(OUT4, 0)

def set_servo_angle(angle):
    """
    Moves the servo motor to a specified angle.
    """
    duty_cycle = (angle / 18.0) + 2.5  # Convert angle to duty cycle
    servo.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)  # Stop sending signal

def move_servo_based_on_color(detected_color):
    """
    Moves the servo to a specific angle based on detected color.
    """
    if detected_color == "Red":
        set_servo_angle(0)
    elif detected_color == "Blue":
        set_servo_angle(45)
    elif detected_color == "Green":
        set_servo_angle(90)
    elif detected_color == "Yellow":
        set_servo_angle(135)
    else:
        set_servo_angle(0)  # Default position for unknown color

try:
    temp = False  # Toggle direction flag
    while True:
        x = input("Press 't' to toggle stepper direction: ")
        if x.lower() == "t":
            temp = not temp  # Toggle direction

            # Detect color and move the servo accordingly
            red, blue, green = DetectColor()
            detected_color = classify_color(red, blue, green)
            print(f"Detected Color: {detected_color}")

            # Move the stepper in the selected direction
            if temp:
                move_stepper(steps=70, delay=0.003, direction="forward")
            else:
                move_stepper(steps=70, delay=0.003, direction="backward")

            # Move the servo motor based on detected color
            move_servo_based_on_color(detected_color)

        time.sleep(2)  # Small delay before next loop

except KeyboardInterrupt:
    GPIO.cleanup()
    servo.stop()