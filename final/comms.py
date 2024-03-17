import serial
import struct
import time

PWM_MIN = 102
PWM_MAX = 512
degree_to_PWM = float(PWM_MAX - PWM_MIN)/ 180.0


def degrees_to_pwm(deg : float):
    return PWM_MIN + int( deg * degree_to_PWM )    

def encode_array_to_bytes(array):
    # Define the format string based on the data type of the array elements
    format_string = '{}B'.format(len(array))  # 'i' for integer
    # Pack the array values into bytes
    encoded_bytes = struct.pack(format_string, *array)
    return encoded_bytes


def send_servo_val(servo_num: int, degrees : float):
    servo_val = degrees_to_pwm(degrees)
    return encode_array_to_bytes([0, servo_num, (servo_val & 0xff00) >> 8, servo_val & 0xff ])


with serial.Serial("/dev/ttyUSB1", baudrate=115200, timeout=1) as ser:

    ser.write(send_servo_val(0, 80))
    time.sleep(2)
    ser.write(send_servo_val(0, 100))
    time.sleep(2)
    ser.write(send_servo_val(0, 90))
    time.sleep(2)
    ser.write(send_servo_val(1, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(1, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(1, 90))
    time.sleep(0.5)
    ser.write(send_servo_val(2, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(2, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(2, 90))
    time.sleep(0.5)


    ser.write(send_servo_val(3, 80))
    time.sleep(2)
    ser.write(send_servo_val(3, 100))
    time.sleep(2)
    ser.write(send_servo_val(3, 90))
    time.sleep(2)
    ser.write(send_servo_val(4, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(4, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(4, 90))
    time.sleep(0.5)
    ser.write(send_servo_val(5, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(5, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(5, 90))
    time.sleep(0.5)


    ser.write(send_servo_val(6, 80))
    time.sleep(2)
    ser.write(send_servo_val(6, 100))
    time.sleep(2)
    ser.write(send_servo_val(6, 90))
    time.sleep(2)
    ser.write(send_servo_val(7, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(7, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(7, 90))
    time.sleep(0.5)
    ser.write(send_servo_val(8, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(8, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(8, 90))
    time.sleep(0.5)


    ser.write(send_servo_val(9, 80))
    time.sleep(2)
    ser.write(send_servo_val(9, 100))
    time.sleep(2)
    ser.write(send_servo_val(9, 90))
    time.sleep(2)
    ser.write(send_servo_val(10, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(10, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(10, 90))
    time.sleep(0.5)
    ser.write(send_servo_val(11, 60))
    time.sleep(0.5)
    ser.write(send_servo_val(11, 120))
    time.sleep(0.5)
    ser.write(send_servo_val(11, 90))
    time.sleep(0.5)

