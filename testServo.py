import RPi.GPIO as GPIO
import time

# Set the GPIO mode to GPIO.BOARD
GPIO.setmode(GPIO.BOARD)

# Define the GPIO pin that the servo is connected to
servo_pin = 11

# Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

# Create a PWM object with a frequency of 50 Hz
pwm = GPIO.PWM(servo_pin, 50)

# Start the PWM with 0% duty cycle
pwm.start(0)

def set_angle(angle):
    """
    Set the angle of the servo motor.

    :param angle: The angle to set the servo to (0-180 degrees)
    """
    duty = angle / 18.0 + 2.5
    pwm.ChangeDutyCycle(duty)

try:
    while True:
        # Set the servo to 0 degrees
        set_angle(0)
        time.sleep(1)

        # Set the servo to 90 degrees
        set_angle(90)
        time.sleep(1)

        # Set the servo to 180 degrees
        set_angle(180)
        time.sleep(1)

except KeyboardInterrupt:
    # If the user presses Ctrl+C, stop the PWM and clean up the GPIO
    pwm.stop()
    GPIO.cleanup()
