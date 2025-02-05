import RPi.GPIO as GPIO
import time

# Assign GPIO pin numbers to variables
s2 = 16
s3 = 18
sig = 22  # labeled "out" on your board
cycles = 10
# Setup GPIO and pins
GPIO.setmode(GPIO.BOARD)
GPIO.setup(s2, GPIO.OUT)
GPIO.setup(s3, GPIO.OUT)
GPIO.setup(sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(31, GPIO.OUT) #forward motor 1
GPIO.setup(33, GPIO.OUT) #forward motor 2
#GPIO.setup(18, GPIO.OUT) #backwards motor 1
#GPIO.set up(36, GPIO.OUT) #backwards motor 2=
# PWM
F_motor1 = GPIO.PWM(31,30) #left motor
F_motor2 = GPIO.PWM(33,30) #Right motor 
#B_motor1 = GPIO.PWM(18,500)
#B_motor2 = GPIO.PWM(36,500)

F_motor1.start(0)
F_motor2.start(0)

def DetectColor():
    # Detect red values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.LOW)
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    red = cycles / duration

    # Detect blue values
    GPIO.output(s2, GPIO.LOW)
    GPIO.output(s3, GPIO.HIGH)
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    blue = cycles / duration

    # Detect green values
    GPIO.output(s2, GPIO.HIGH)
    GPIO.output(s3, GPIO.HIGH)
    start_time = time.time()
    for _ in range(cycles):
        GPIO.wait_for_edge(sig, GPIO.FALLING)
    duration = time.time() - start_time
    green = cycles / duration

    # Returning tuple
    return red, blue, green


def clean_outliers(data):
    data.sort()
    data[:20] = [0] * 20
    data[79:] = [0] * 21
    return data




def calculate_average(data):
    valid_data = [x for x in data if x != 0]
    return sum(valid_data) / len(valid_data)


def adjust_value(value, base, divisor, addition=0):
    return (value + addition - base) / divisor


def detect_color(adjusted_red, adjusted_blue, adjusted_green):

    # if max(red, blue, green) == red and not purple or orange then red
    # take care of secondary colors first
    

    if adjusted_green > adjusted_blue and adjusted_green > adjusted_red:
        return int(adjusted_green)
    # elif abs(adjusted_red - adjusted_blue) < 2:
    #     return "Purple"
    elif adjusted_red > adjusted_blue and adjusted_red > adjusted_green:
        return int(adjusted_red)
    elif adjusted_blue > adjusted_red and adjusted_blue > adjusted_green:
        return  int(adjusted_blue)
    else:
        return int(0)


def Color_Detector_ok():
    red_values = []
    blue_values = []
    green_values = []

    for _ in range(100):
        red_instance, blue_instance, green_instance = DetectColor()
        red_values.append(red_instance)
        blue_values.append(blue_instance)
        green_values.append(green_instance)

    red_values = clean_outliers(red_values)
    blue_values = clean_outliers(blue_values)
    green_values = clean_outliers(green_values)

    average_red = calculate_average(red_values)
    average_blue = calculate_average(blue_values)
    average_green = calculate_average(green_values)

    # adjusted_red = adjust_value(average_red, 0, 1000, addition=1000)
    # adjusted_blue = adjust_value(average_blue, 0, 1000, addition=-3000)
    # adjusted_green = adjust_value(average_green, 0, 1000, addition=1500)
    return int(average_red)
    #detected_color = detect_color(average_red, average_blue, average_green)
    #print(detected_color)
def motor_move():
    # F_motor1.start(15) #left motor
    # F_motor2.start(15)
    current_color = Color_Detector_ok()
    print(current_color)
    if current_color > 8500:  # >9000 = white
        F_motor2.start(15) #left
        F_motor1.start(0)
        print("left")
    elif current_color < 8500 and 5000 < current_color : #9000, 7500
        F_motor2.start(0)
        F_motor1.start(15)
        print("Right")
    elif current_color < 5000: #7500
        F_motor2.start(0)
        F_motor1.start(15) 
        print("purple")
        time.sleep(0.5)
try:
    while True:
        motor_move()
        # red = Color_Detector_ok()
        # print (red)
except KeyboardInterrupt:
    GPIO.cleanup()