import cv2
import cvzone
from cvzone import FPS
from cvzone import ColorModule
import math
import serial
import time

# =================================================================
# 1. SERIAL CONNECTION CONFIGURATION
# =================================================================
# CHANGE 'COM3' to match your actual Arduino port (e.g., 'COM4', '/dev/ttyUSB0')
try:
    arduino = serial.Serial(port='COM5', baudrate=9600, timeout=0.1)
    time.sleep(2)  # Give the Arduino bootloader line time to settle safely
    print(">>> [SUCCESS] Serial communication established with Arduino.")
except Exception as e:
    print(f">>> [WARNING] Serial port connection failed: {e}")
    print(">>> Running in SIMULATION MODE (printing data packets to console only).")
    arduino = None

# =================================================================
# 2. CALIBRATION CONFIGURATION OFFSETS
# =================================================================
# Adjust this offset if your base servo doesn't physically align with the camera axes
BASE_PHYSICAL_OFFSET = 0
INVERT_BASE_DIRECTION = False  # Set to True if your base servo moves away from target


def solve_planar_arm_ik(x_world, z_world, phi_degrees, base_offset_z=5.0, min_z_clearance=3.0, min_x_clearance=5.0):
    """
    Dynamic 3-DOF planar IK solver with Vertical Frame Translation.
    All inputs and dimensions are processed in centimeters (cm).
    """
    L1 = 13.0
    L2 = 8.5
    L3 = 7.0

    x = x_world
    z = z_world - base_offset_z
    phi_rad = math.radians(phi_degrees)

    xw = x - L3 * math.cos(phi_rad)
    zw = z - L3 * math.sin(phi_rad)

    d_squared = xw ** 2 + zw ** 2
    cos_theta2 = (d_squared - L1 ** 2 - L2 ** 2) / (2.0 * L1 * L2)

    if not (-1.0 <= cos_theta2 <= 1.0):
        return None

    alpha = math.atan2(zw, xw)
    valid_solutions = {}

    # CONFIGURATION 1: ELBOW UP
    sin_theta2_down = -math.sqrt(1.0 - cos_theta2 ** 2)
    theta2_down = math.atan2(sin_theta2_down, cos_theta2)
    beta_down = math.atan2(L2 * sin_theta2_down, L1 + L2 * cos_theta2)
    theta1_down = alpha - beta_down
    theta3_down = phi_rad - theta1_down - theta2_down

    x_elbow_down = L1 * math.cos(theta1_down)
    z_elbow_down_world = (L1 * math.sin(theta1_down)) + base_offset_z
    xw_world = xw
    zw_world = zw + base_offset_z

    if (z_world >= min_z_clearance and zw_world >= min_z_clearance and z_elbow_down_world >= min_z_clearance and
            x_world >= min_x_clearance and xw_world >= min_x_clearance and x_elbow_down >= min_x_clearance):
        valid_solutions["Elbow_Up"] = [
            math.degrees(theta1_down),
            math.degrees(theta2_down),
            math.degrees(theta3_down)
        ]

    # CONFIGURATION 2: ELBOW DOWN
    sin_theta2_up = math.sqrt(1.0 - cos_theta2 ** 2)
    theta2_up = math.atan2(sin_theta2_up, cos_theta2)
    beta_up = math.atan2(L2 * sin_theta2_up, L1 + L2 * cos_theta2)
    theta1_up = alpha - beta_up
    theta3_up = phi_rad - theta1_up - theta2_up

    x_elbow_up = L1 * math.cos(theta1_up)
    z_elbow_up_world = (L1 * math.sin(theta1_up)) + base_offset_z

    if (z_world >= min_z_clearance and zw_world >= min_z_clearance and z_elbow_up_world >= min_z_clearance and
            x_world >= min_x_clearance and xw_world >= min_x_clearance and x_elbow_up >= min_x_clearance):
        valid_solutions["Elbow_Down"] = [
            math.degrees(theta1_up),
            math.degrees(theta2_up),
            math.degrees(theta3_up)
        ]

    return valid_solutions if valid_solutions else None


fpsReader = FPS.FPS(avgCount=30)
cap = cv2.VideoCapture(2)
cap.set(cv2.CAP_PROP_FPS, 30)

myColorFinder = ColorModule.ColorFinder(trackBar=False)

hsvRedVals = {'hmin': 0, 'smin': 61, 'vmin': 63, 'hmax': 22, 'smax': 255, 'vmax': 255}
hsvBaseVals = {'hmin': 0, 'smin': 0, 'vmin': 1, 'hmax': 157, 'smax': 66, 'vmax': 49}
hsvYellowVals = {'hmin': 11, 'smin': 60, 'vmin': 84, 'hmax': 67, 'smax': 239, 'vmax': 238}
hsvGreenVals = {'hmin': 33, 'smin': 110, 'vmin': 74, 'hmax': 91, 'smax': 237, 'vmax': 138}

arm_is_busy = False
locked_target_cm = 0.0
locked_target_color = None

print("\n>>> SYSTEM OPERATIONAL.")
print(">>> Press [SPACEBAR] to capture current targets and execute sequence.")
print(">>> Press [R] to manually break busy lockouts. Press [Q] to terminate program.\n")

while True:
    success, img = cap.read()
    if not success:
        print("Failed to grab frame.")
        break

    # ------------------ PHONE CAM STABILIZATION FIX ------------------
    img = cv2.resize(img, (1280, 720))
    ymin, ymax = 30, 720
    xmin, xmax = 100, 1050
    raw_cropped_zone = img[ymin:ymax, xmin:xmax]
    img = cv2.resize(raw_cropped_zone, (640, 480))
    # -----------------------------------------------------------------

    fps, img = fpsReader.update(img, pos=(20, 50),
                                bgColor=(255, 0, 255), textColor=(255, 255, 255),
                                scale=3, thickness=3)

    imgGreen, maskGreen = myColorFinder.update(img, hsvGreenVals)
    imgYellow, maskYellow = myColorFinder.update(img, hsvYellowVals)
    imgBase, maskBase = myColorFinder.update(img, hsvBaseVals)
    imgRed, maskRed = myColorFinder.update(img, hsvRedVals)

    imgContoursGreen, conFoundGreen = cvzone.findContours(img, maskGreen, minArea=200)
    imgContoursYellow, conFoundYellow = cvzone.findContours(img, maskYellow, minArea=1000)
    imgContoursBase, conFoundBase = cvzone.findContours(img, maskBase)
    imgContoursRed, conFoundRed = cvzone.findContours(img, maskRed, minArea=600)

    # Always track base position anchor (Black contour location)
    base_x, base_y = 320, 480
    if conFoundBase:
        base_x = conFoundBase[0]['center'][0]
        base_y = conFoundBase[0]['center'][1]

    # Draw anchor tracking points on frames
    cv2.circle(imgContoursGreen, (base_x, base_y), 7, (255, 0, 255), cv2.FILLED)
    cv2.circle(imgContoursYellow, (base_x, base_y), 7, (255, 0, 255), cv2.FILLED)
    cv2.circle(imgContoursBase, (base_x, base_y), 7, (255, 0, 255), cv2.FILLED)
    cv2.circle(imgContoursRed, (base_x, base_y), 7, (255, 0, 255), cv2.FILLED)

    green_x, green_y = 0, 0
    yellow_x, yellow_y = 0, 0
    red_x, red_y = 0, 0

    if conFoundGreen:
        green_x = conFoundGreen[0]['center'][0]
        green_y = conFoundGreen[0]['center'][1]
    if conFoundYellow:
        yellow_x = conFoundYellow[0]['center'][0]
        yellow_y = conFoundYellow[0]['center'][1]
    if conFoundRed:
        red_x = conFoundRed[0]['center'][0]
        red_y = conFoundRed[0]['center'][1]

    # --- Live Overlay Calculations ---
    if conFoundGreen:
        live_dist_px = ((green_x - base_x) ** 2 + (green_y - base_y) ** 2) ** 0.5
        live_dist_cm = (live_dist_px * 0.0692) - 0.361
        cv2.line(imgContoursGreen, (base_x, base_y), (green_x, green_y), (0, 255, 0), 2)
        cv2.putText(imgContoursGreen, f"{live_dist_cm:.1f} cm",
                    (int((base_x + green_x) / 2), int((base_y + green_y) / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)

    if conFoundYellow:
        live_dist_px = ((yellow_x - base_x) ** 2 + (yellow_y - base_y) ** 2) ** 0.5
        live_dist_cm = (live_dist_px * 0.0692) - 0.361
        cv2.line(imgContoursYellow, (base_x, base_y), (yellow_x, yellow_y), (0, 255, 255), 2)
        cv2.putText(imgContoursYellow, f"{live_dist_cm:.1f} cm",
                    (int((base_x + yellow_x) / 2), int((base_y + yellow_y) / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 2)

    if conFoundRed and conFoundGreen:
        rg_dist_px = ((green_x - red_x) ** 2 + (green_y - red_y) ** 2) ** 0.5
        rg_dist_cm = (rg_dist_px * 0.0692) - 0.361
        cv2.line(imgContoursRed, (red_x, red_y), (green_x, green_y), (0, 255, 0), 2)
        cv2.putText(imgContoursRed, f"R->G: {rg_dist_cm:.1f} cm",
                    (int((red_x + green_x) / 2), int((red_y + green_y) / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 0), 2)

    if conFoundRed and conFoundYellow:
        ry_dist_px = ((yellow_x - red_x) ** 2 + (yellow_y - red_y) ** 2) ** 0.5
        ry_dist_cm = (ry_dist_px * 0.0692) - 0.361
        cv2.line(imgContoursRed, (red_x, red_y), (yellow_x, yellow_y), (0, 255, 255), 2)
        cv2.putText(imgContoursRed, f"R->Y: {ry_dist_cm:.1f} cm",
                    (int((red_x + yellow_x) / 2), int((red_y + yellow_y) / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (0, 255, 255), 2)

    imgStackGreen = cvzone.stackImages([img, imgGreen, maskGreen, imgContoursGreen], 2, 0.7)
    imgStackYellow = cvzone.stackImages([img, imgYellow, maskYellow, imgContoursYellow], 2, 0.7)
    imgStackBase = cvzone.stackImages([img, imgBase, maskBase, imgContoursBase], 2, 0.7)
    imgStackRed = cvzone.stackImages([img, imgRed, maskRed, imgContoursRed], 2, 0.7)

    cv2.imshow('imgGreen', imgStackGreen)
    cv2.imshow('imgYellow', imgStackYellow)
    cv2.imshow('imgBase', imgStackBase)
    cv2.imshow('imgRed', imgStackRed)

    key = cv2.waitKey(1) & 0xFF

    # ----------------=================================================
    # 3. BACKWARDS SERIAL HANDSHAKE RECEIVER
    # ----------------=================================================
    if arm_is_busy and arduino is not None and arduino.in_waiting > 0:
        incoming_line = arduino.readline().decode('utf-8').strip()
        if incoming_line == "DONE":
            arm_is_busy = False
            locked_target_color = None
            print("\n>>> [HANDSHAKE] Arduino finished sorting cycle. Camera UNLOCKED.")

    # ----------------=================================================
    # 4. CAPTURE TRIGGER ENGINE (SPACEBAR)
    # ----------------=================================================
    if key == ord(' ') and not arm_is_busy:
        target_x, target_y = 0, 0

        if conFoundGreen:
            target_x, target_y = green_x, green_y
            locked_target_color = "Green"
            arm_is_busy = True
        elif conFoundYellow:
            target_x, target_y = yellow_x, yellow_y
            locked_target_color = "Yellow"
            arm_is_busy = True
        elif conFoundRed:
            target_x, target_y = red_x, red_y
            locked_target_color = "Red"
            arm_is_busy = True

        if arm_is_busy:
            # A. Calculate Profile Radial Reach from Base Anchor (Black Axis)
            pixel_dist = ((target_x - base_x) ** 2 + (target_y - base_y) ** 2) ** 0.5
            locked_target_cm = (pixel_dist * 0.0692) - 0.361

            # B. Calculate Angular Alignment from Base out to target position
            dx = target_x - base_x
            dy =   target_y - base_y # Correct for inverted pixel frame matrix direction
            base_angle_rad = math.atan2(dy, dx)
            base_out = int(math.degrees(base_angle_rad))
            base_out = 90 - base_out



            # C. Handle Directional Inversions and Custom Calibration Constants
            if INVERT_BASE_DIRECTION:
                base_out = 180 - base_out
            base_out += BASE_PHYSICAL_OFFSET
            base_out = max(0, min(180, base_out))  # Keep values inside safe servo horizons

            print(
                f"\n>>> [CAPTURE] Processing {locked_target_color} at {locked_target_cm:.2f} cm | Base Heading: {base_out}°")

            # D. Choose the corresponding dynamic profile adjustment factor
            if locked_target_cm < 19:
                theta = solve_planar_arm_ik(locked_target_cm - 4, 3, 0)
            elif 19 <= locked_target_cm < 22:
                theta = solve_planar_arm_ik(locked_target_cm - 4 , 3, 0)
            elif 22 <= locked_target_cm < 26:
                theta = solve_planar_arm_ik(locked_target_cm - 4, 3, 0)
            else:
                theta = solve_planar_arm_ik(locked_target_cm - 3.5, 3, 0)

            if theta and "Elbow_Up" in theta:
                t1, t2, t3 = theta["Elbow_Up"]
                shoulder_out = int(t1)
                forearm_out = abs(int(t2))
                thumb_out = int(t3)
                gripper_out = 125  # Clamped holding position

                # E. Pack Data String: "Shoulder,Forearm,Thumb,Gripper,Base,Color\n"
                packet = f"{shoulder_out},{forearm_out},{thumb_out},{gripper_out},{base_out},{locked_target_color}\n"

                if arduino is not None:
                    arduino.write(packet.encode('utf-8'))
                    print(f">>> [TX] Packet transmitted to hardware: {packet.strip()}")

                else:
                    print(f">>> [SIM_TX] Packet data generated: {packet.strip()}")
            else:
                print(">>> [IK ERROR] Unreachable target or boundary bubble breached. Resetting lock.")
                arm_is_busy = False
        else:
            print("\n>>> [WARNING] Trigger pressed but no items present in workspace frame.")

    elif arm_is_busy:
        print(f"[EXECUTING] Robot arm actively sorting the {locked_target_color} box... Waiting for Arduino signal.",
              end="\r")

    if key == ord('r'):
        arm_is_busy = False
        locked_target_color = None
        print("\n>>> [OVERRIDE] Local system busy status flag cleared.")

    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
