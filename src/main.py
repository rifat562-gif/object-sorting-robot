#!/usr/bin/env python3
"""
Object Sorting Robot (Raspberry Pi)
Detects white square vs rectangle with OpenCV and sorts using servos + DC motors.
"""

import time
import cv2
import numpy as np

from picamera2 import Picamera2
import RPi.GPIO as GPIO


# ---------- Pins (BCM) ----------
SERVO_BASE = 21
SERVO_ARM  = 24
SERVO_GRIP = 25

M1_EN,  M1_IN1, M1_IN2 = 6,  13, 19
M2_EN,  M2_IN1, M2_IN2 = 12, 16, 26

# ---------- Vision tuning ----------
THRESH = 200
MIN_AREA = 10000
MAX_DISTANCE_FROM_CENTER = 200

SQUARE_AR_LOW, SQUARE_AR_HIGH = 0.90, 1.10
RECT_AR_LOW,   RECT_AR_HIGH   = 0.50, 2.00

# ---------- PWM ----------
MOTOR_PWM_FREQ = 100
SERVO_PWM_FREQ = 50
TURN_SPEED = 100
TURN_DURATION = 1.0


GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup([SERVO_BASE, SERVO_ARM, SERVO_GRIP], GPIO.OUT)
GPIO.setup([M1_EN, M1_IN1, M1_IN2, M2_EN, M2_IN1, M2_IN2], GPIO.OUT)

motor1_pwm = GPIO.PWM(M1_EN, MOTOR_PWM_FREQ)
motor2_pwm = GPIO.PWM(M2_EN, MOTOR_PWM_FREQ)
motor1_pwm.start(0)
motor2_pwm.start(0)

servo_base_pwm = GPIO.PWM(SERVO_BASE, SERVO_PWM_FREQ)
servo_arm_pwm  = GPIO.PWM(SERVO_ARM,  SERVO_PWM_FREQ)
servo_grip_pwm = GPIO.PWM(SERVO_GRIP, SERVO_PWM_FREQ)
servo_base_pwm.start(0)
servo_arm_pwm.start(0)
servo_grip_pwm.start(0)


def set_servo_angle(pwm: GPIO.PWM, angle: float, hold: float = 0.25) -> None:
    """Basic 0–180° servo control (tune mapping if needed)."""
    angle = max(0.0, min(180.0, float(angle)))
    duty = angle / 18.0 + 2.5
    pwm.ChangeDutyCycle(duty)
    time.sleep(hold)
    pwm.ChangeDutyCycle(0)


def normal_position() -> None:
    """Neutral pose (adjust angles for your gripper geometry)."""
    set_servo_angle(servo_base_pwm, 20)
    set_servo_angle(servo_arm_pwm,  20)
    set_servo_angle(servo_grip_pwm, 0)


def grabbing_sequence() -> None:
    """Pick-and-place motion (tune angles/timing for your robot)."""
    set_servo_angle(servo_arm_pwm,  10)
    set_servo_angle(servo_grip_pwm, 40)
    time.sleep(0.8)

    set_servo_angle(servo_base_pwm, 140)
    time.sleep(0.5)

    set_servo_angle(servo_arm_pwm,  20)
    set_servo_angle(servo_grip_pwm, 0)
    time.sleep(0.5)


def _motors_stop() -> None:
    motor1_pwm.ChangeDutyCycle(0)
    motor2_pwm.ChangeDutyCycle(0)


def left_turn(duration: float = TURN_DURATION, speed: int = TURN_SPEED) -> None:
    GPIO.output(M1_IN1, GPIO.LOW)
    GPIO.output(M1_IN2, GPIO.HIGH)
    GPIO.output(M2_IN1, GPIO.HIGH)
    GPIO.output(M2_IN2, GPIO.LOW)

    motor1_pwm.ChangeDutyCycle(int(speed))
    motor2_pwm.ChangeDutyCycle(int(speed))
    time.sleep(duration)
    _motors_stop()


def right_turn(duration: float = TURN_DURATION, speed: int = TURN_SPEED) -> None:
    GPIO.output(M1_IN1, GPIO.HIGH)
    GPIO.output(M1_IN2, GPIO.LOW)
    GPIO.output(M2_IN1, GPIO.LOW)
    GPIO.output(M2_IN2, GPIO.HIGH)

    motor1_pwm.ChangeDutyCycle(int(speed))
    motor2_pwm.ChangeDutyCycle(int(speed))
    time.sleep(duration)
    _motors_stop()


def detect_object(frame_bgr: np.ndarray):
    """Returns (annotated_frame, binary_mask, label, info)."""
    gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, THRESH, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    h, w = frame_bgr.shape[:2]
    cx, cy = w // 2, h // 2

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_AREA:
            continue

        approx = cv2.approxPolyDP(contour, 0.02 * cv2.arcLength(contour, True), True)
        x, y, bw, bh = cv2.boundingRect(approx)
        if bh == 0:
            continue

        obj_cx, obj_cy = x + bw // 2, y + bh // 2
        dist = float(np.hypot(obj_cx - cx, obj_cy - cy))
        if dist > MAX_DISTANCE_FROM_CENTER:
            continue

        ar = float(bw) / float(bh)

        label = None
        if SQUARE_AR_LOW <= ar <= SQUARE_AR_HIGH:
            label = "square"
        elif RECT_AR_LOW <= ar <= RECT_AR_HIGH:
            label = "rectangle"

        if label:
            cv2.drawContours(frame_bgr, [approx], 0, (0, 255, 0), 2)
            cv2.putText(frame_bgr, f"{label} (AR={ar:.2f})", (x, max(20, y - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            info = {"x": x, "y": y, "w": bw, "h": bh, "area": float(area),
                    "aspect_ratio": ar, "distance": dist}
            return frame_bgr, binary, label, info

    return frame_bgr, binary, None, None


def cleanup():
    cv2.destroyAllWindows()
    for pwm in (motor1_pwm, motor2_pwm, servo_base_pwm, servo_arm_pwm, servo_grip_pwm):
        try:
            pwm.stop()
        except Exception:
            pass
    GPIO.cleanup()


def main():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (1280, 720)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    normal_position()
    last_action_time = 0.0
    COOLDOWN = 2.0  # avoid repeated triggers

    try:
        while True:
            frame_rgb = picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

            annotated, binary, label, info = detect_object(frame_bgr)

            cv2.imshow("Detection", annotated)
            cv2.imshow("Binary", binary)

            now = time.time()
            if label and info and (now - last_action_time) > COOLDOWN:
                print(f"{label} | AR={info['aspect_ratio']:.2f} | area={info['area']:.0f} | dist={info['distance']:.1f}")
                grabbing_sequence()

                # Sorting rule (edit if your robot uses opposite directions)
                if label == "square":
                    left_turn()
                    set_servo_angle(servo_base_pwm, 30)
                    right_turn()
                else:
                    right_turn()
                    set_servo_angle(servo_base_pwm, 30)
                    normal_position()
                    left_turn()

                last_action_time = now

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    finally:
        try:
            picam2.stop()
        except Exception:
            pass
        cleanup()


if __name__ == "__main__":
    main()
