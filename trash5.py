from picamera2 import Picamera2
import cv2
from ultralytics import YOLO
import time
import RPi.GPIO as GPIO
import os

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

LED = {'can': 5, 'glass': 6, 'plastic': 13}
FULL_LED = {'can': 16, 'glass': 20, 'plastic': 21}
SOUND = {'can': 17, 'glass': 22, 'plastic': 27}
SERVO = {'can': 18, 'glass': 23, 'plastic': 24}
TRIG = {'can': 10, 'glass': 11, 'plastic': 7}
ECHO = {'can': 9, 'glass': 8, 'plastic': 1}

for pin_set in [LED, FULL_LED]:
    for pin in pin_set.values():
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

for pin in SERVO.values():
    GPIO.setup(pin, GPIO.OUT)

for t in TRIG.values():
    GPIO.setup(t, GPIO.OUT)
    GPIO.output(t, GPIO.LOW)
for e in ECHO.values():
    GPIO.setup(e, GPIO.IN)

for pin in SOUND.values():
    GPIO.setup(pin, GPIO.IN)

servo_pwm = {k: GPIO.PWM(S, 50) for k, S in SERVO.items()}
for pwm in servo_pwm.values():
    pwm.start(0)

SERVO_DUTY = {0: 2.5, 100: 8.0}

def set_servo_angle(servo, angle):
    duty = SERVO_DUTY.get(angle, 2.5)
    servo.ChangeDutyCycle(duty)
    time.sleep(0.5)
    servo.ChangeDutyCycle(0)

def get_distance(trig, echo):
    GPIO.output(trig, True)
    time.sleep(0.00001)
    GPIO.output(trig, False)

    timeout = time.time() + 0.05
    while GPIO.input(echo) == 0:
        if time.time() > timeout:
            return 999
    start = time.time()

    timeout = time.time() + 0.05
    while GPIO.input(echo) == 1:
        if time.time() > timeout:
            return 999
    stop = time.time()

    duration = stop - start
    distance = duration * 34300 / 2
    return distance

model = YOLO("/home/raspberrypi/your_model.pt")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()
time.sleep(2)

sound_active_until = {'can': 0, 'glass': 0, 'plastic': 0}
sound_played = {'can': False, 'glass': False, 'plastic': False}
detection_start = {'can': 0, 'glass': 0, 'plastic': 0}
servo_open = {'can': False, 'glass': False, 'plastic': False}

def is_detected_for(label, threshold=3):
    now = time.time()
    if detection_start[label] == 0:
        detection_start[label] = now
        return False
    elif now - detection_start[label] >= threshold:
        return True
    return False

try:
    while True:
        frame = picam2.capture_array()
        frame = frame[:, :, :3]
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        results = model(frame)[0]

        detected = None
        for box in results.boxes:
            cls = int(box.cls[0])
            label = model.names[cls].lower()
            if label in ['can', 'glass', 'plastic']:
                detected = label
                break

        if detected:
            if is_detected_for(detected):
                print(f"[INFO] Detected: {detected}")
                GPIO.output(LED[detected], GPIO.HIGH)
                dist = get_distance(TRIG[detected], ECHO[detected])
                print(f"[DEBUG] {detected} distance: {dist:.1f} cm")
                if dist < 10:
                    GPIO.output(FULL_LED[detected], GPIO.HIGH)
                    print("[WARNING] Bin is full!")
                else:
                    if not servo_open[detected]:
                        set_servo_angle(servo_pwm[detected], 100)
                        servo_open[detected] = True
                        sound_active_until[detected] = time.time() + 20
                        sound_played[detected] = False
            else:
                print(f"[INFO] {detected} detected but waiting for 3 seconds...")
        else:
            detection_start = {'can': 0, 'glass': 0, 'plastic': 0}

        for label in SOUND:
            if time.time() < sound_active_until[label] and not sound_played[label]:
                if GPIO.input(SOUND[label]) == 1:
                    print(f"[INFO] Sound detected on {label} → closing lid")
                    set_servo_angle(servo_pwm[label], 0)
                    servo_open[label] = False
                    time.sleep(0.3)
                    os.system("aplay /home/raspberrypi/correct.wav &")
                    GPIO.output(LED[label], GPIO.LOW)
                    GPIO.output(FULL_LED[label], GPIO.LOW)
                    sound_played[label] = True

        cv2.imshow("Trash Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("프로그램 종료")

finally:
    for pwm in servo_pwm.values():
        pwm.stop()
    GPIO.cleanup()
    picam2.stop()
    cv2.destroyAllWindows()