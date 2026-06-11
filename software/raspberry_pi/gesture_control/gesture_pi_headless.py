from pypylon import pylon
import cv2
import time
import serial
import math
import mediapipe as mp
from collections import deque, Counter

TARGET_SERIAL = "23967249"

# ===== 串口配置 =====
SERIAL_ENABLED = True
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUDRATE = 115200

ser = None

if SERIAL_ENABLED:
    try:
        ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
        time.sleep(1)
        print(f"串口已打开：{SERIAL_PORT}, 波特率：{SERIAL_BAUDRATE}")
    except Exception as e:
        print("串口打开失败：", e)
        ser = None


CONFIRM_COUNT = 5
NO_HAND_STOP_TIME = 1.0


def angle_between(a, b, c):
    ab_x = a.x - b.x
    ab_y = a.y - b.y
    bc_x = c.x - b.x
    bc_y = c.y - b.y

    dot = ab_x * bc_x + ab_y * bc_y
    norm_ab = math.sqrt(ab_x * ab_x + ab_y * ab_y)
    norm_bc = math.sqrt(bc_x * bc_x + bc_y * bc_y)

    if norm_ab == 0 or norm_bc == 0:
        return 0

    cos_angle = dot / (norm_ab * norm_bc)
    cos_angle = max(-1.0, min(1.0, cos_angle))
    return math.degrees(math.acos(cos_angle))


def finger_is_straight(hand_landmarks, mcp_id, pip_id, dip_id, tip_id):
    lm = hand_landmarks.landmark
    pip_angle = angle_between(lm[mcp_id], lm[pip_id], lm[dip_id])
    dip_angle = angle_between(lm[pip_id], lm[dip_id], lm[tip_id])
    return pip_angle > 145 and dip_angle > 145


def thumb_is_open(hand_landmarks):
    lm = hand_landmarks.landmark

    thumb_angle = angle_between(lm[2], lm[3], lm[4])
    thumb_tip = lm[4]
    index_mcp = lm[5]

    distance = math.sqrt(
        (thumb_tip.x - index_mcp.x) ** 2 +
        (thumb_tip.y - index_mcp.y) ** 2
    )

    return thumb_angle > 140 and distance > 0.08


def get_index_direction(hand_landmarks):
    lm = hand_landmarks.landmark

    index_mcp = lm[5]
    index_tip = lm[8]

    dx = index_tip.x - index_mcp.x
    dy = index_tip.y - index_mcp.y

    threshold = 0.10

    if abs(dx) > abs(dy):
        if dx < -threshold:
            return "LEFT"
        elif dx > threshold:
            return "RIGHT"
    else:
        if dy < -threshold:
            return "GIMBAL_UP"
        elif dy > threshold:
            return "GIMBAL_DOWN"

    return "POINT"


def classify_gesture(hand_landmarks):
    thumb = thumb_is_open(hand_landmarks)

    index = finger_is_straight(hand_landmarks, 5, 6, 7, 8)
    middle = finger_is_straight(hand_landmarks, 9, 10, 11, 12)
    ring = finger_is_straight(hand_landmarks, 13, 14, 15, 16)
    pinky = finger_is_straight(hand_landmarks, 17, 18, 19, 20)

    straight_count = sum([thumb, index, middle, ring, pinky])

    # 双指：钻机启动
    # 食指 + 中指伸出，无名指和小指收回
    if index and middle and not ring and not pinky:
        return "DRILL_ON"

    # 只伸食指：方向控制
    if index and not middle and not ring and not pinky:
        return get_index_direction(hand_landmarks)

    # 握拳：四根长手指都没有伸直
    if not index and not middle and not ring and not pinky:
        return "FORWARD"

    # 张开手掌：全局急停，包括钻机停止
    if straight_count >= 4:
        return "STOP"

    return "UNKNOWN"

class CommandStabilizer:
    """
    慢确认稳定器：
    1. STOP 快速确认，保证安全
    2. 普通方向/机构指令需要连续多帧确认
    3. FORWARD 最严格，防止切换手势时误触发前进
    4. DRILL_ON / DRILL_OFF 也需要较严格确认
    5. UNKNOWN 不改变当前 Stable
    6. 手离开画面超过 no_hand_stop_time 秒后自动 STOP
    """

    def __init__(
        self,
        stop_confirm_count=3,
        normal_confirm_count=8,
        forward_confirm_count=15,
        drill_confirm_count=12,
        no_hand_stop_time=1.0
    ):
        self.stop_confirm_count = stop_confirm_count
        self.normal_confirm_count = normal_confirm_count
        self.forward_confirm_count = forward_confirm_count
        self.drill_confirm_count = drill_confirm_count
        self.no_hand_stop_time = no_hand_stop_time

        self.stable_command = "STOP"
        self.last_printed_command = None

        self.candidate_command = None
        self.candidate_count = 0

        self.last_hand_time = time.time()

    def get_required_count(self, raw_command):
        if raw_command == "STOP":
            return self.stop_confirm_count

        if raw_command == "FORWARD":
            return self.forward_confirm_count

        if raw_command in ["DRILL_ON", "DRILL_OFF"]:
            return self.drill_confirm_count

        return self.normal_confirm_count

    def update(self, raw_command, hand_detected):
        now = time.time()

        if not hand_detected:
            self.candidate_command = None
            self.candidate_count = 0

            if now - self.last_hand_time > self.no_hand_stop_time:
                self.stable_command = "STOP"

            return self.stable_command

        self.last_hand_time = now

        if raw_command in ["UNKNOWN", "NO_HAND"]:
            return self.stable_command

        if raw_command == self.candidate_command:
            self.candidate_count += 1
        else:
            self.candidate_command = raw_command
            self.candidate_count = 1

        required_count = self.get_required_count(raw_command)

        if self.candidate_count >= required_count:
            self.stable_command = raw_command

        return self.stable_command

    def should_print(self):
        if self.stable_command != self.last_printed_command:
            self.last_printed_command = self.stable_command
            return True
        return False

def map_gesture_to_control(command):
    """
    方案A：直接映射控制逻辑

    STOP        -> STOP_ALL
    FORWARD     -> DRIVE_FORWARD
    LEFT        -> DRIVE_LEFT
    RIGHT       -> DRIVE_RIGHT
    GIMBAL_UP   -> BUCKET_UP
    GIMBAL_DOWN -> BUCKET_DOWN
    DRILL_ON    -> DRILL_ON
    DRILL_OFF   -> DRILL_OFF
    """

    if command == "STOP":
        return "STOP_ALL"

    elif command == "FORWARD":
        return "DRIVE_FORWARD"

    elif command == "LEFT":
        return "DRIVE_LEFT"

    elif command == "RIGHT":
        return "DRIVE_RIGHT"

    elif command == "GIMBAL_UP":
        return "BUCKET_UP"

    elif command == "GIMBAL_DOWN":
        return "BUCKET_DOWN"

    elif command == "DRILL_ON":
        return "DRILL_ON"

    elif command == "DRILL_OFF":
        return "DRILL_OFF"

    else:
        return "NO_ACTION"


def send_serial_command(control_cmd):
    """
    通过串口发送控制指令给下位机。
    每条指令以换行符结尾，方便下位机按行解析。
    """

    if control_cmd == "NO_ACTION":
        return

    data = control_cmd + "\n"

    if ser is not None and ser.is_open:
        ser.write(data.encode("utf-8"))
        print("[SERIAL SEND]", control_cmd)
    else:
        print("[SERIAL SIM]", control_cmd)


def print_action_description(control_cmd):
    """
    打印动作说明。
    """

    if control_cmd == "STOP_ALL":
        print("[ACTION] 全部停止：四轮停止，挖斗停止，钻机停止")

    elif control_cmd == "DRIVE_FORWARD":
        print("[ACTION] 四轮前进")

    elif control_cmd == "DRIVE_LEFT":
        print("[ACTION] 四轮左转")

    elif control_cmd == "DRIVE_RIGHT":
        print("[ACTION] 四轮右转")

    elif control_cmd == "BUCKET_UP":
        print("[ACTION] 挖斗举升")

    elif control_cmd == "BUCKET_DOWN":
        print("[ACTION] 挖斗下降")

    elif control_cmd == "DRILL_ON":
        print("[ACTION] 钻机启动")

    elif control_cmd == "DRILL_OFF":
        print("[ACTION] 钻机停止")

    else:
        print("[ACTION] 无有效动作")


def execute_command(command):
    """
    只在 Stable Command 发生变化时调用。
    """

    control_cmd = map_gesture_to_control(command)

    if control_cmd == "NO_ACTION":
        return

    print("[CONTROL]", control_cmd)
    send_serial_command(control_cmd)
    print_action_description(control_cmd)


tl_factory = pylon.TlFactory.GetInstance()
devices = tl_factory.EnumerateDevices()

selected_device = None
for device in devices:
    if device.GetSerialNumber() == TARGET_SERIAL:
        selected_device = device

if selected_device is None:
    raise RuntimeError("没有找到目标 Basler 相机。")

camera = pylon.InstantCamera(tl_factory.CreateDevice(selected_device))
camera.Open()
# ========== 设置曝光和增益 ==========
def try_set_feature(name, value):
    try:
        node = getattr(camera, name)
        node.SetValue(value)
        print(f"{name} 设置为 {value}")
        return True
    except Exception as e:
        print(f"{name} 设置失败：{e}")
        return False


# 关闭自动曝光，改用手动曝光
try_set_feature("ExposureAuto", "Off")

# 老款 Basler 常用 ExposureTimeAbs，新款常用 ExposureTime
if not try_set_feature("ExposureTime", 30000.0):
    try_set_feature("ExposureTimeAbs", 30000.0)

# 增益设置
try_set_feature("GainAuto", "Continuous")

# 如果自动增益无效，可以尝试手动增益
# 老款可能是 GainRaw，新款可能是 Gain
if not try_set_feature("Gain", 6.0):
    try_set_feature("GainRaw", 100)
print("已打开相机：", camera.GetDeviceInfo().GetFriendlyName())

try:
    camera.Width.SetValue(960)
    camera.Height.SetValue(600)
    print("相机采集分辨率设置为 960x600")
except Exception as e:
    print("分辨率设置失败，使用默认分辨率：", e)

camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

converter = pylon.ImageFormatConverter()
converter.OutputPixelFormat = pylon.PixelType_BGR8packed
converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned

mp_hands = mp.solutions.hands

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    model_complexity=0,
    min_detection_confidence=0.6,
    min_tracking_confidence=0.6
)

stabilizer = CommandStabilizer(
    stop_confirm_count=3,
    normal_confirm_count=8,
    forward_confirm_count=15,
    drill_confirm_count=12,
    no_hand_stop_time=1.0
)

print("开始树莓派无显示手势识别。按 Ctrl+C 退出。")
print("测试手势：张开手掌=STOP，握拳=FORWARD，食指方向=LEFT/RIGHT/GIMBAL_UP/GIMBAL_DOWN")

frame_count = 0
fps_start = time.time()

try:
    while camera.IsGrabbing():
        grab_result = camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)

        if grab_result.GrabSucceeded():
            image = converter.Convert(grab_result)
            frame = image.GetArray()

            frame = cv2.resize(frame, (640, 400))
            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            result = hands.process(rgb_frame)

            raw_command = "NO_HAND"
            hand_detected = False

            if result.multi_hand_landmarks:
                hand_detected = True
                hand_landmarks = result.multi_hand_landmarks[0]
                raw_command = classify_gesture(hand_landmarks)

            stable_command = stabilizer.update(raw_command, hand_detected)
# 每隔一段时间打印原始识别状态，方便判断是不是 Raw 在乱跳
            if frame_count % 30 == 0:
                print("Hand:", hand_detected, "| Raw:", raw_command, "| Stable:", stable_command)
            if stabilizer.should_print():
                print("Stable Command:", stable_command)
                execute_command(stable_command)

            frame_count += 1

            if time.time() - fps_start >= 5:
                fps = frame_count / (time.time() - fps_start)
                print("FPS:", round(fps, 2))
                frame_count = 0
                fps_start = time.time()

        grab_result.Release()

except KeyboardInterrupt:
    print("收到退出指令，正在关闭...")

camera.StopGrabbing()
camera.Close()
hands.close()

if ser is not None and ser.is_open:
    ser.close()
    print("串口已关闭。")

print("程序结束。")

