import serial
import matplotlib.pyplot as plt
import math
from typing import TypedDict

class LidarData(TypedDict):
    fsa: float
    lsa: float
    cs: int
    speed: float
    timestamp: int
    confidences: list[int]
    distances: list[float]
    angles: list[float]

def calculate_lidar_data(commands: str) -> LidarData:
    data = commands.replace(" ", "")

    speed: float = int(data[2:4] + data[0:2], 16) / 100
    fsa: float = int(data[6:8] + data[4:6], 16) / 100
    lsa: float = int(data[-8:-6] + data[-10:-8], 16) / 100
    timestamp: float = int(data[-4:-2] + data[-6:-4], 16)
    cs: float = int(data[-2:], 16)

    confidences: list[int] = []
    angles: list[float] = []
    distances: list[float] = []
    angle_step: float = 0

    if lsa - fsa > 0:
        angle_step = (lsa - fsa) / 12
    else:
        angle_step = (lsa - fsa + 360) / 12
    
    for i in range(12):
        distances.append(int(data[i * 6 + 10:i * 6 + 12] + data[i * 6 + 8: i * 6 + 10], 16) / 100)
        confidences.append(int(data[i * 6 + 12:i * 6 + 14], 16))
        angles.append((angle_step * i + fsa) % 360 * math.pi / 180)
    return {"fsa": fsa, "lsa": lsa, "cs": cs, "speed": speed, "timestamp": timestamp, "confidences": confidences, "angles": angles, "distances": distances}

serial = serial.Serial(port="COM6", baudrate=230400, timeout=5.0, bytesize=8, parity='N', stopbits=1)
screen = plt.figure()
graph = screen.add_subplot(111, projection="polar")

i: int = 0
commands: str = ""
angles: list[float] = []
distances: list[float] = []
speed: float = 0
while plt.fignum_exists(screen.number):
    command_end: bool = False
    byte_end: bool = False
    if i % 40 == 39:
        plt.cla()
        graph.plot(angles, distances, ".r-")
        plt.pause(0.01)
        angles.clear()
        distances.clear()
        i = 0
    while not command_end:
        byte = serial.read()
        big_byte = int.from_bytes(byte)
        if big_byte == 0x54:
            commands += byte.hex() + " "
            byte_end = True
            continue
        elif big_byte == 0x2c and byte_end:
            commands += byte.hex()
            if len(commands[0:-5].replace(" ", "")) != 90:
                commands = ""
                command_end = True
                byte_end = False
                continue
            data = calculate_lidar_data(commands[0:-5])
            angles.extend(data["angles"])
            distances.extend(data["distances"])
            speed = data["speed"]
            commands = ""
            command_end = True
        else:
            commands += byte.hex() + " "
        byte_end = False
    i += 1
