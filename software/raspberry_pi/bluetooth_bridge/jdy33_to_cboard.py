import serial
import time

JDY_PORT = "/dev/ttyUSB0"      # JDY-33 + CP2102
CBOARD_PORT = "/dev/ttyUSB1"   # 第二个USB-TTL接C板

JDY_BAUD = 9600
CBOARD_BAUD = 115200

SPEED = 120

cmd_map = {
    "A": f"F{SPEED}\n",  # 前进
    "E": f"B{SPEED}\n",  # 后退
    "C": f"R{SPEED}\n",  # 右转
    "G": f"L{SPEED}\n",  # 左转
    "Z": "S0\n",         # 停车
}

jdy = serial.Serial(JDY_PORT, JDY_BAUD, timeout=0.05)
cboard = serial.Serial(CBOARD_PORT, CBOARD_BAUD, timeout=0.1)

last_cmd = None
last_rx_time = time.time()

print("JDY-33 to C-board bridge started")
print("A=F120, E=B120, C=R120, G=L120, Z=S0")
print("Ctrl+C to exit")

try:
    while True:
        data = jdy.read(64)

        if data:
            text = data.decode(errors="ignore")
            print("JDY:", repr(text))

            for ch in text:
                if ch in cmd_map:
                    out = cmd_map[ch]

                    # 避免同一个命令刷太快
                    if out != last_cmd:
                        cboard.write(out.encode())
                        print("Send to C-board:", out.strip())
                        last_cmd = out

                    last_rx_time = time.time()

        # 安全保护：超过0.5秒没收到遥控数据，自动停车
        if time.time() - last_rx_time > 0.5:
            if last_cmd != "S0\n":
                cboard.write(b"S0\n")
                print("Timeout: Send S0")
                last_cmd = "S0\n"

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\nExit, send stop")
    cboard.write(b"S0\n")

finally:
    jdy.close()
    cboard.close()