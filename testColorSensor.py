import RPi.GPIO as GPIO
import time

# Assign GPIO pin numbers for color sensor
s0 = 13
s1 = 15 
s2 = 16
s3 = 18
sig = 22

cycles = 10

# Setup GPIO and color sensor pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(s0, GPIO.OUT)
GPIO.setup(s1, GPIO.OUT)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Set frequency scaling
GPIO.output(s0, GPIO.HIGH)
GPIO.output(s1, GPIO.LOW)

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

try:
    while True:
        # Detect and classify color
        red, blue, green = DetectColor()
        detected_color = classify_color(red, blue, green)
        print(f"Detected Color: {detected_color}")
        time.sleep(1)  # Wait 1 second before next reading

except KeyboardInterrupt:
    GPIO.cleanup()