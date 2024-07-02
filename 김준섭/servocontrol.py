import smbus
import RPi.GPIO as GPIO
import time
from picamera import PiCamera
from ftplib import FTP

# MPU6050의 레지스터 주소
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F

# I2C 버스 설정
bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# GPIO 설정
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)  # 서보모터 제어 핀

# PWM 신호 생성
pwm = GPIO.PWM(18, 50)  # 50Hz 주파수 (서보모터에 따라 다를 수 있음)
pwm.start(0)

# 카메라 설정
camera = PiCamera()
camera.resolution = (1024, 768)

# FTP 설정
ftp_server = "your_ftp_server_address"
ftp_user = "your_username"
ftp_password = "your_password"
ftp = FTP(ftp_server)
ftp.login(user=ftp_user, passwd=ftp_password)

def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def get_acceleration():
    acc_x = read_raw_data(ACCEL_XOUT_H)
    acc_y = read_raw_data(ACCEL_YOUT_H)
    acc_z = read_raw_data(ACCEL_ZOUT_H)
    return acc_x, acc_y, acc_z

def get_velocity(acc_x, acc_y, acc_z, dt):
    vel_x = acc_x * dt
    vel_y = acc_y * dt
    vel_z = acc_z * dt
    return vel_x, vel_y, vel_z

def set_servo_angle(angle):
    duty_cycle = (0.05 * 50) + (0.19 * 50 * angle / 180)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)

def capture_and_send_image():
    image_path = "/home/pi/captured_image.jpg"
    camera.capture(image_path)
    with open(image_path, "rb") as file:
        ftp.storbinary("STOR captured_image.jpg", file)

try:
    start_time = time.time()
    duration_above_threshold = 0
    threshold = 2  # m/s
    duration_condition = 3  # seconds
    while True:
        acc_x, acc_y, acc_z = get_acceleration()
        dt = 0.1  # 시간 간격 (예: 0.1초)
        vel_x, vel_y, vel_z = get_velocity(acc_x, acc_y, acc_z, dt)

        if vel_z > threshold:
            duration_above_threshold += dt
        else:
            duration_above_threshold = 0

        if duration_above_threshold >= duration_condition:
            set_servo_angle(0)
            time.sleep(1)
            set_servo_angle(360)
            time.sleep(1)
            set_servo_angle(0)
            time.sleep(1)

            while True:
                capture_and_send_image()
                time.sleep(2)  # 2초마다 이미지 캡처 및 전송

except KeyboardInterrupt:
    pwm.stop()
    GPIO.cleanup()
    ftp.quit()
