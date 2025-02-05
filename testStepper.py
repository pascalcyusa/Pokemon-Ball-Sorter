import RPi.GPIO as GPIO
import time

# Set the GPIO mode to GPIO.BOARD
GPIO.setmode(GPIO.BOARD)

# Define the GPIO pins that the stepper motor is connected to
stepper_pins = [37, 36, 31, 29]

# Set the pins as outputs
for pin in stepper_pins:
    GPIO.setup(pin, GPIO.OUT)

# Define the step sequence for the stepper motor
step_sequence = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

def set_step(step):
    """
    Set the stepper motor to a specific step.

    :param step: The step to set the motor to (0-3)
    """
    for i in range(4):
        GPIO.output(stepper_pins[i], step_sequence[step][i])

try:
    while True:
        for step in range(4):
            set_step(step)
            time.sleep(0.01)  # Adjust the delay to control the speed of the motor

except KeyboardInterrupt:
    # If the user presses Ctrl+C, clean up the GPIO
    GPIO.cleanup()
