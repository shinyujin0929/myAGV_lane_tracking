import RPi.GPIO as GPIO
import time
import socket

IN1 = 7
IN2 = 8
ENA = 9
ENCODER_PIN = 11

encoder_count = 0

HOST = '172.30.1.43'
PORT = 9973

#encoder_detected_flag = False

#pulse_count = 0
#target_pulses = 350

GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(ENA, GPIO.OUT)
GPIO.setup(ENCODER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


pwm = GPIO.PWM(ENA, 50)
pwm.start(0)

sock  = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR,1)
sock.bind(('', PORT))
sock.listen(1)
print("listening")

c_sock, addr = sock.accept()
print("connected")

def forward():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(50)

def stop():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.LOW)
    pwm.ChangeDutyCycle(0)

def encoder_detected(channel):
    global encoder_count
    encoder_count += 1
    print(f"encoder detected {encoder_count} time(s)")
    stop()
    time.sleep(3)


GPIO.add_event_detect(ENCODER_PIN, GPIO.FALLING, callback=encoder_detected, bouncetime=200)

try:
    while True:

        read_data = c_sock.recv(1024)
        
        if not read_data:
            break

        command = read_data.decode("utf-8")

        if command == "start":
            print("start motor")
            forward()
        elif command == "stop":
            print("stop motor")
            stop()
        elif command == "exit":
            print("Exit")


except KeyboardInterrupt:
    pass
    #stop()

finally:
    c_sock.close()
    sock.close()
    GPIO.cleanup()