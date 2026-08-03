import threading
import socket
import struct
import os
import queue
import time
from collections import deque
from typing import Dict, Deque, Any, Optional, Union, TypeVar, List
import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QHBoxLayout, QGroupBox, QLabel, QSizePolicy, QGridLayout, QMenuBar, QMenu, QAction, QFileDialog, QDialog, QCheckBox, QPushButton, QSlider, QSpinBox
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont
import pyqtgraph as pg
import numpy as np
import math
from ahrs.filters import Madgwick
import logging
from datetime import datetime
from OpenGL.GL import *
from OpenGL.GLU import *
from PyQt5.QtOpenGL import QGLWidget

# 添加通信协议相关常量
FRAME_HEAD1 = 0xFF
FRAME_HEAD2 = 0xFA
FRAME_TAIL1 = 0x88
FRAME_TAIL2 = 0x77

# 运动模式定义
MODE_NONE = 0x00         # 未设置步态
MODE_WORM = 0x01         # 蠕虫步态模式
MODE_SNAKE_FORWARD = 0x02 # 蛇形直线前进
MODE_SNAKE_LATERAL = 0x03 # 蛇形侧向运动
MODE_SNAKE_CIRCULAR = 0x04 # 蛇形圆周运动

# 添加全局变量来跟踪当前步态类型
current_gait_type = MODE_NONE  # 默认为未设置步态

def create_command(command_type, parameter):
    """
    创建控制命令
    :param command_type: 命令类型 (0x01: 蠕虫步态, 0x02: 蛇形直线, 0x03: 蛇形侧向, 0x04: 蛇形圆周)
    :param parameter: 参数值 (蠕虫模式: 1-8步态号; 蛇形模式: 0-90角度)
    :return: bytes 命令数据
    """
    # 对蛇形模式(0x02, 0x03, 0x04)进行角度转换
    if command_type in [0x02, 0x03, 0x04]:
        # 将0-90度角度值转换为0-255范围
        parameter = int(parameter * 255 / 90)
        # 确保转换后的值在有效范围内
        parameter = max(0, min(255, parameter))

    command = bytearray([
        FRAME_HEAD1,    # 帧头1
        FRAME_HEAD2,    # 帧头2
        command_type,   # 命令类型
        parameter,      # 参数
        FRAME_TAIL1,    # 帧尾1
        FRAME_TAIL2     # 帧尾2
    ])
    return bytes(command)

# 配置
PORTS = [12340, 12341, 12342, 12343, 12344]
BUFFER_SIZE = 65536  # 增大TCP接收缓冲区
MAX_QUEUE_SIZE = 20000  # 增大处理队列
filepath = "E:/Fudan/worms/codesnew/robot_v0/data_save_v2/imu_data01/"
FRAME_HEADER = b'\xFA\xFF'  # 帧头
FRAME_TAIL = b'\xFB\xFC'    # 帧尾
IMU_DATA_SIZE = 40          # IMU数据部分的大小为40字节
STATE_SIZE = 2
TOTAL_FRAME_SIZE = len(FRAME_HEADER) + IMU_DATA_SIZE + len(FRAME_TAIL) + STATE_SIZE
TCP_SOCKET_BUFFER = 262144  # 增大socket缓冲区

# 全局变量
data_buffers: Dict[int, bytearray] = {}  # 端口数据缓冲池
processing_queues: Dict[int, queue.Queue[bytes]] = {}  # 处理队列
last_update_time = {}
packet_counts = {port: 0 for port in PORTS}  # 每个端口的数据包计数
imu_update_frequency = {}
lock = threading.Lock()
stop_event = threading.Event()

# 添加绘图相关的全局变量
PLOT_WINDOW = 500  # 显示最近500个数据点

# 定义类型变量
T = TypeVar('T')
QueueType = queue.Queue[bytes]
DequeType = deque[Union[float, int]]

# 修改全局变量的类型注解
processing_queues: Dict[int, QueueType] = {}  # 使用类型别名
data_buffers: Dict[int, bytearray] = {}

# 修改 plot_data 的类型注解
PlotDataType = Dict[str, DequeType]
plot_data: Dict[int, PlotDataType] = {
    port: {
        'time': deque(maxlen=PLOT_WINDOW),
        'acc_x': deque(maxlen=PLOT_WINDOW),
        'acc_y': deque(maxlen=PLOT_WINDOW),
        'acc_z': deque(maxlen=PLOT_WINDOW),
        'gyro_x': deque(maxlen=PLOT_WINDOW),
        'gyro_y': deque(maxlen=PLOT_WINDOW),
        'gyro_z': deque(maxlen=PLOT_WINDOW)
    } for port in PORTS
}

# 初始化缓冲池和处理队列
for port in PORTS:
    data_buffers[port] = bytearray()
    processing_queues[port] = queue.Queue(maxsize=MAX_QUEUE_SIZE)

class LowPassFilter:
    def __init__(self, alpha=0.1):
        self.alpha = alpha
        self.last_value = None

    def filter(self, value):
        if self.last_value is None:
            self.last_value = value
            return value
        filtered = self.alpha * value + (1 - self.alpha) * self.last_value
        self.last_value = filtered
        return filtered

# 为每个端口的每个创建滤波器
filters = {port: {
    'gyro_x': LowPassFilter(),
    'gyro_y': LowPassFilter(),
    'gyro_z': LowPassFilter()
} for port in PORTS}

def process_data(data: bytes):
    """解析IMU数据"""
    orientation = struct.unpack("4f", data[:16])
    acceleration = struct.unpack("3f", data[16:28])
    gyroscope = struct.unpack("3f", data[28:])
    return orientation, acceleration, gyroscope

def save_data(imu_data, state, port):
    """保存IMU数据到文件"""
    global current_gait_type
    filename = f"{filepath}imu_data_{port}.txt"
    data_str = f"{current_gait_type} {state} "  # 添加步态类型字段
    data_str += " ".join(f"{v:.6f}" for v in imu_data["acceleration"])
    data_str += " " + " ".join(f"{v:.6f}" for v in imu_data["gyroscope"])
    data_str += " " + " ".join(f"{v:.6f}" for v in imu_data["orientation"])

    with open(filename, 'a') as f:
        f.write(data_str + "\n")

def update_frequency(port):
    """更新IMU频率"""
    global last_update_time, imu_update_frequency, packet_counts
    current_time = time.time()
    logger = port_loggers[port]

    with lock:
        if port not in last_update_time:
            last_update_time[port] = current_time
            imu_update_frequency[port] = 0
            packet_counts[port] = 1
        else:
            packet_counts[port] += 1
            elapsed_time = current_time - last_update_time[port]
            if elapsed_time >= 1.0:
                freq = round(packet_counts[port] / elapsed_time)
                imu_update_frequency[port] = freq
                logger.info(f"当前频率: {freq} Hz, 包计数: {packet_counts[port]}")
                last_update_time[port] = current_time
                packet_counts[port] = 0

def find_frame_in_buffer(data_buffer: bytearray) -> tuple[bool, int]:
    """在缓冲区中查找完整的数据帧"""
    # 查找帧头
    header_pos = -1
    for i in range(len(data_buffer) - len(FRAME_HEADER) + 1):
        if data_buffer[i:i+len(FRAME_HEADER)] == FRAME_HEADER:
            header_pos = i
            break

    if header_pos == -1:
        return False, 0

    # 检查是否有足够的数据构成完整帧
    frame_end = header_pos + TOTAL_FRAME_SIZE
    if frame_end > len(data_buffer):
        return False, 0

    # 验证帧尾
    tail_pos = frame_end - len(FRAME_TAIL)
    if data_buffer[tail_pos:frame_end] == FRAME_TAIL:
        return True, header_pos

    return False, 0

def process_buffer(port):
    """处理数据缓冲区"""
    logger = port_loggers[port]
    data_buffer = bytearray()
    last_process_time = time.time()

    while not stop_event.is_set():
        try:
            current_time = time.time()

            # 定期清理过旧的数据
            if current_time - last_process_time > 5.0:  # 5秒无数据则清理
                if len(data_buffer) > 0:
                    logger.warning(f"⚠️ 清理过期数据: {len(data_buffer)} bytes")
                    data_buffer.clear()
                last_process_time = current_time

            # 从处理队列中获取数据
            try:
                data = processing_queues[port].get_nowait()
                last_process_time = current_time
                data_buffer.extend(data)

                # 记录缓冲区大小
                logger.debug(f"📊 当前缓冲区大小: {len(data_buffer)} bytes")

            except queue.Empty:
                time.sleep(0.001)
                continue

            # 处理完整帧...

        except Exception as e:
            logger.error(f"❌ 处理数据错误: {str(e)}")
            time.sleep(0.1)

def monitor_statistics():
    """监控并显示统计信息"""
    target_freq = 100  # 目标频率
    while not stop_event.is_set():
        time.sleep(1)
        with lock:
            print("\033[2J\033[H", end="")
            print("\n" + "="*60)
            print("🔍 IMU实时数据统计 (目标: 100Hz)")
            print("="*60)
            for port in PORTS:
                if port in imu_update_frequency:
                    freq = imu_update_frequency[port]
                    percentage = (freq / target_freq) * 100

                    # 根据频率百分比选择状态
                    if percentage >= 95:
                        status = "✅"
                    elif percentage >= 80:
                        status = "⚠️"
                    else:
                        status = "❌"

                    bar = "█" * min(int(freq/10), 50)
                    print(f"端口 {port}: {freq:4d} Hz {status} [{percentage:3.0f}%] |{bar}")
                else:
                    print(f"端口 {port}: 🔄 等待数据...")
            print("="*60)
            print("📊 实时更新... (按 Ctrl+C 停止)")

# 在文件开头添加日志相关配置
import logging
from datetime import datetime

# 创建日志目录
log_path = "E:/Fudan/worms/codesnew/robot_v0/log/"
if not os.path.exists(log_path):
    os.makedirs(log_path)

# 为每个端口创建单独的日志记录器
port_loggers = {}
for port in PORTS:
    logger = logging.getLogger(f'port_{port}')
    logger.setLevel(logging.INFO)

    # 创建文件处理
    log_file = f"{log_path}port_{port}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
    file_handler = logging.FileHandler(log_file, encoding='utf-8')

    # 设置格式
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    port_loggers[port] = logger

def save_and_plot_data(imu_data, state, port, system_time, window=None):
    """保存数据并更新图表数据"""
    global current_gait_type
    logger = port_loggers[port]
    try:
        # 获取IMU数据
        acc = imu_data["acceleration"]
        gyro = imu_data["gyroscope"]

        # 更新绘图数据
        with lock:  # 添加锁保护
            if len(plot_data[port]['time']) >= PLOT_WINDOW:
                plot_data[port]['time'].popleft()
                plot_data[port]['acc_x'].popleft()
                plot_data[port]['acc_y'].popleft()
                plot_data[port]['acc_z'].popleft()
                plot_data[port]['gyro_x'].popleft()
                plot_data[port]['gyro_y'].popleft()
                plot_data[port]['gyro_z'].popleft()

            # 添加新数据
            current_time = time.time()
            plot_data[port]['time'].append(current_time)
            plot_data[port]['acc_x'].append(acc[0])
            plot_data[port]['acc_y'].append(acc[1])
            plot_data[port]['acc_z'].append(acc[2])
            plot_data[port]['gyro_x'].append(filters[port]['gyro_x'].filter(gyro[0]))
            plot_data[port]['gyro_y'].append(filters[port]['gyro_y'].filter(gyro[1]))
            plot_data[port]['gyro_z'].append(filters[port]['gyro_z'].filter(gyro[2]))

        # 保存数据到文件，添加步态类型
        filename = f"{filepath}imu_data_{port}.txt"
        data_str = f"{current_gait_type} {state} "  # 添加步态类型字段
        data_str += " ".join(f"{v:.6f}" for v in acc)
        data_str += " " + " ".join(f"{v:.6f}" for v in gyro)
        data_str += " " + " ".join(f"{v:.6f}" for v in imu_data["orientation"])
        # 显示到毫秒和微秒
        data_str += f" {system_time.strftime('%H:%M:%S.%f')}"
        data_str += "\n"

        with open(filename, 'a') as f:
            f.write(data_str)

    except Exception as e:
        logger.error(f"❌ 数据处理错误: {e}")

def handle_tcp_client(client_socket, port):
    """处理TCP客户端连接"""
    logger = port_loggers[port]
    data_buffer = bytearray()
    frame_count = 0
    last_data_time = time.time()
    last_frame_time = time.time()

    # 设置socket选项
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
    client_socket.settimeout(0.1)  # 100ms超时

    # 保存客户端socket到全局字典
    active_connections[port] = client_socket

    try:
        while not stop_event.is_set():
            try:
                data = client_socket.recv(BUFFER_SIZE)
                if not data:
                    logger.warning(f"端口 {port} 接收到空数据")
                    break

                current_time = time.time()
                data_buffer.extend(data)

                # 处理完整帧
                while len(data_buffer) >= TOTAL_FRAME_SIZE:
                    found, frame_start = find_frame_in_buffer(data_buffer)
                    if not found:
                        next_header = data_buffer.find(FRAME_HEADER[0])
                        if next_header == -1:
                            data_buffer = data_buffer[-TOTAL_FRAME_SIZE:]
                        else:
                            data_buffer = data_buffer[next_header:]
                        break

                    frame = data_buffer[frame_start:frame_start + TOTAL_FRAME_SIZE]
                    data_buffer = data_buffer[frame_start + TOTAL_FRAME_SIZE:]

                    try:
                        imu_data = {
                            "orientation": struct.unpack("4f", frame[2:18]),
                            "acceleration": struct.unpack("3f", frame[18:30]),
                            "gyroscope": struct.unpack("3f", frame[30:42]),
                        }
                        state = struct.unpack("BB", frame[42:44])
                        # 获取当前系统时间
                        system_time = datetime.now()

                        frame_count += 1
                        last_frame_time = current_time

                        # 每秒更新一次频率
                        if current_time - last_data_time >= 1.0:
                            freq = frame_count / (current_time - last_data_time)
                            with lock:
                                imu_update_frequency[port] = int(freq)
                            frame_count = 0
                            last_data_time = current_time
                            logger.debug(f"IMU更新频率: {int(freq)} Hz")

                        # 保存和更新数据
                        save_and_plot_data(imu_data, state, port, system_time)

                    except Exception as e:
                        logger.error(f"解析帧错误: {e}")
                        continue

                # 检查数据接收是否超时
                if current_time - last_frame_time > 1.0:  # 1秒没有收到新帧
                    logger.warning("数据接收超时，重置连接")
                    break

            except socket.timeout:
                continue
            except socket.error as e:
                logger.error(f"Socket错误: {e}")
                break

    except Exception as e:
        logger.error(f"处理客户端错误: {e}")
    finally:
        with lock:
            if port in imu_update_frequency:
                del imu_update_frequency[port]
        if port in active_connections:
            del active_connections[port]
        client_socket.close()

# 添加全局变量来存储活动连接
active_connections = {}

def send_control_command(self, command_type, parameter):
    """发送控制命令到所有体节"""
    global current_gait_type  # 添加全局变量声明
    try:
        print("\n===== 命令发送调试信息 =====")
        print(f"命令类型: 0x{command_type:02X}")
        print(f"参数值: {parameter}")

        # 更新当前步态类型
        current_gait_type = command_type
        print(f"当前步态类型已更新为: 0x{current_gait_type:02X}")

        # 检查所有体节端口的连接状态
        command_ports = [12340, 12341, 12342, 12343, 12344]  # 所有体节的端口
        connected_ports = []

        for port in command_ports:
            if port in active_connections:
                connected_ports.append(port)
            else:
                print(f"警告: 端口 {port} 没有可用的TCP连接")

        if not connected_ports:
            self.statusBar().showMessage("错误: 没有可用的TCP连接", 2000)
            return

        print(f"TCP连接状态: {len(connected_ports)} 个端口已连接")
        cmd = create_command(command_type, parameter)
        print(f"发送的命令内容: {[hex(x) for x in cmd]}")

        # 向所有已连接的端口发送命令
        success_count = 0
        for port in connected_ports:
            try:
                if send_command(active_connections[port], cmd):
                    success_count += 1
                    print(f"端口 {port} 命令发送成功")
                else:
                    print(f"端口 {port} 命令发送失败")
            except Exception as e:
                print(f"端口 {port} 发送错误: {str(e)}")

        mode_str = {
            MODE_NONE: "未设置步态",
            MODE_WORM: "蠕虫步态",
            MODE_SNAKE_FORWARD: "蛇形直线",
            MODE_SNAKE_LATERAL: "蛇形侧向",
            MODE_SNAKE_CIRCULAR: "蛇形圆周"
        }.get(command_type, "未知")

        # 显示发送结果
        if success_count > 0:
            self.statusBar().showMessage(
                f"命令发送成功: 类型={mode_str}, 参数={parameter} (成功: {success_count}/{len(connected_ports)}端口)",
                2000
            )
        else:
            self.statusBar().showMessage("所有端口发送失败", 2000)

    except Exception as e:
        print(f"发送错误: {str(e)}")
        self.statusBar().showMessage(f"发送错误: {str(e)}", 2000)

def start_tcp_server(port):
    """启动TCP服务器"""
    try:
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, TCP_SOCKET_BUFFER)

        hostname = socket.gethostname()
        host_ip = socket.gethostbyname(hostname)
        print(f"正在绑定 {host_ip}:{port}")

        try:
            server_socket.bind((host_ip, port))
            server_socket.listen(1)
            print(f"TCP服务器正在监听端口 {port}")

            while not stop_event.is_set():
                try:
                    client_socket, addr = server_socket.accept()
                    print(f"新的连接从 {addr} 到端口 {port}")

                    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, TCP_SOCKET_BUFFER * 2)

                    client_thread = threading.Thread(
                        target=handle_tcp_client,
                        args=(client_socket, port)
                    )
                    client_thread.daemon = True
                    client_thread.start()

                except Exception as e:
                    print(f"接受连接出错 (端口 {port}): {e}")
                    if stop_event.is_set():
                        break
                    time.sleep(1)

        except socket.error as e:
            print(f"端口 {port} 绑定失败: {e}")

    except Exception as e:
        print(f"服务器错误 (端口 {port}): {e}")
    finally:
        server_socket.close()

def start_tcp_servers():
    """启动所有TCP服务器和绘图"""
    server_threads = []

    # 启动统计监控线程
    stats_thread = threading.Thread(target=monitor_statistics)
    stats_thread.daemon = True
    stats_thread.start()

    # 启动TCP服务器线程
    for port in PORTS:
        thread = threading.Thread(target=start_tcp_server, args=(port,))
        thread.daemon = True
        server_threads.append(thread)
        thread.start()

    return server_threads

class IMUAttitudeWidget(QGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.quaternions = {port: [1.0, 0.0, 0.0, 0.0] for port in PORTS}
        self.colors = {
            12340: (1.0, 0.0, 0.0),  # 红色
            12341: (0.0, 1.0, 0.0),  # 绿色
            12342: (0.0, 0.0, 1.0),  # 蓝色
            12343: (1.0, 1.0, 0.0),  # 黄色
            12344: (1.0, 0.0, 1.0)   # 紫色
        }
        self.show_ports = {port: True for port in PORTS}

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, [0, 0, 1, 0])

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w/h, 0.1, 100.0)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0, 0, -5)

        for port, quat in self.quaternions.items():
            if not self.show_ports[port]:
                continue

            glPushMatrix()
            # 应用四元数旋转
            self.apply_quaternion(quat)
            # 设置IMU颜色
            glColor3f(*self.colors[port])
            # 绘制IMU模型
            self.draw_imu()
            glPopMatrix()

    def draw_imu(self):
        """绘制IMU的3D模型"""
        glBegin(GL_QUADS)
        # 前面
        glVertex3f(-0.5, -0.2, 0.1)
        glVertex3f(0.5, -0.2, 0.1)
        glVertex3f(0.5, 0.2, 0.1)
        glVertex3f(-0.5, 0.2, 0.1)
        # 后面
        glVertex3f(-0.5, -0.2, -0.1)
        glVertex3f(-0.5, 0.2, -0.1)
        glVertex3f(0.5, 0.2, -0.1)
        glVertex3f(0.5, -0.2, -0.1)
        glEnd()

        # 绘制坐标轴
        glBegin(GL_LINES)
        # X轴 (红色)
        glColor3f(1, 0, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(1, 0, 0)
        # Y轴 (绿色)
        glColor3f(0, 1, 0)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1, 0)
        # Z轴 (蓝色)
        glColor3f(0, 0, 1)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1)
        glEnd()

    def apply_quaternion(self, q):
        """应用四元数旋转"""
        w, x, y, z = q
        matrix = [
            [1-2*y*y-2*z*z, 2*x*y-2*w*z, 2*x*z+2*w*y, 0],
            [2*x*y+2*w*z, 1-2*x*x-2*z*z, 2*y*z-2*w*x, 0],
            [2*x*z-2*w*y, 2*y*z+2*w*x, 1-2*x*x-2*y*y, 0],
            [0, 0, 0, 1]
        ]
        glMultMatrixf(matrix)

    def update_attitude(self, port, quaternion):
        """更新指定端口的姿态"""
        self.quaternions[port] = quaternion
        self.updateGL()

class ModernMenuBar(QMenuBar):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setStyleSheet("""
            QMenuBar {
                background-color: #2d2d2d;
                color: white;
                border-bottom: 2px solid #3498db;
                padding: 5px;
            }
            QMenuBar::item {
                padding: 8px 12px;
                margin: 2px;
                border-radius: 4px;
            }
            QMenuBar::item:selected {
                background-color: #3498db;
            }
            QMenu {
                background-color: #2d2d2d;
                border: 1px solid #3498db;
                border-radius: 4px;
                padding: 5px;
            }
            QMenu::item {
                padding: 8px 25px;
                border-radius: 4px;
                color: white;
            }
            QMenu::item:selected {
                background-color: #3498db;
            }
        """)

class ModernControlPanel(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("IMU控制面板")
        self.setStyleSheet("""
            QDialog {
                background-color: #1e1e1e;
                border-radius: 10px;
            }
            QGroupBox {
                background-color: #2d2d2d;
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 12px;
                padding: 10px;
                color: white;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 8px 15px;
                border-radius: 4px;
                font-weight: bold;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QLabel {
                color: white;
                font-size: 12px;
            }
            QCheckBox {
                color: white;
                padding: 5px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
            }
        """)

        layout = QVBoxLayout(self)

        # 创建IMU控制卡片
        self.create_imu_control_card(layout)
        # 创建显示设置卡片
        self.create_display_settings_card(layout)
        # 创建数据记录卡片
        self.create_data_recording_card(layout)

        self.setLayout(layout)

    def create_imu_control_card(self, parent_layout):
        group = QGroupBox("IMU控制")
        layout = QGridLayout()

        for i, port in enumerate(PORTS):
            card = QWidget()
            card_layout = QHBoxLayout()

            # IMU状态指器
            status = QLabel("●")
            status.setStyleSheet("color: #2ecc71; font-size: 20px;")

            # IMU信息
            info = QLabel(f"IMU {port}")
            info.setStyleSheet("font-size: 14px;")

            # 控制按钮
            control_btn = QPushButton("配置")
            control_btn.setFixedWidth(60)

            card_layout.addWidget(status)
            card_layout.addWidget(info)
            card_layout.addWidget(control_btn)
            card.setLayout(card_layout)

            layout.addWidget(card, i // 2, i % 2)

        group.setLayout(layout)
        parent_layout.addWidget(group)

    def create_display_settings_card(self, parent_layout):
        group = QGroupBox("显示设置")
        layout = QVBoxLayout()

        # 显示选项
        options = [
            ("显示加速度", True),
            ("显示角速度", True),
            ("显示姿态", True),
            ("显示3D视图", False)
        ]

        for text, checked in options:
            checkbox = QCheckBox(text)
            checkbox.setChecked(checked)
            layout.addWidget(checkbox)

        group.setLayout(layout)
        parent_layout.addWidget(group)

    def create_data_recording_card(self, parent_layout):
        group = QGroupBox("数据记录")
        layout = QVBoxLayout()

        # 路径显示
        path_layout = QHBoxLayout()
        path_label = QLabel("保存路径:")
        path_value = QLabel(filepath)
        path_value.setStyleSheet("color: #3498db;")
        path_btn = QPushButton("选择")
        path_btn.setFixedWidth(60)

        path_layout.addWidget(path_label)
        path_layout.addWidget(path_value)
        path_layout.addWidget(path_btn)

        # 记录控制
        control_layout = QHBoxLayout()
        record_btn = QPushButton("开始记录")
        record_btn.setStyleSheet("background-color: #27ae60;")
        stop_btn = QPushButton("停止")
        stop_btn.setStyleSheet("background-color: #c0392b;")

        control_layout.addWidget(record_btn)
        control_layout.addWidget(stop_btn)

        layout.addLayout(path_layout)
        layout.addLayout(control_layout)

        group.setLayout(layout)
        parent_layout.addWidget(group)

class IMUCard(QGroupBox):
    def __init__(self, port, parent=None):
        super().__init__(parent)
        self.port = port
        self.parent_window = parent
        self.is_expanded = False
        # 修改标题格式：从IMU 1234x改为体节x
        segment_num = str(port)[-1]  # 获取端口号的最后一位数字
        self.setTitle(f"体节{segment_num}")
        self.setStyleSheet("""
            QGroupBox {
                color: white;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 6px;
                margin-top: 12px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        self.curves = {}

        # 创建主布局
        self.main_layout = QVBoxLayout(self)
        self.setup_ui()

        # 添加鼠标点击事件
        self.mousePressEvent = self.on_card_click

    def setup_ui(self):
        """初始化UI组件"""
        # 创建状态指示器
        status_layout = QHBoxLayout()
        self.status_indicator = QLabel("●")
        self.status_indicator.setStyleSheet("color: #e74c3c; font-size: 20px;")
        self.status_text = QLabel("离线")
        status_layout.addWidget(self.status_indicator)
        status_layout.addWidget(self.status_text)
        status_layout.addStretch()

        # 创建图表
        plots_layout = QHBoxLayout()
        self.acc_plot = self.create_plot("加速度 (m/s²)")
        self.gyro_plot = self.create_plot("角速度 (rad/s)")
        plots_layout.addWidget(self.acc_plot)
        plots_layout.addWidget(self.gyro_plot)

        # 创建数值显示区域
        values_layout = QGridLayout()
        self.acc_values = {
            'x': QLabel("X: 0.000"),
            'y': QLabel("Y: 0.000"),
            'z': QLabel("Z: 0.000")
        }
        self.gyro_values = {
            'x': QLabel("X: 0.000"),
            'y': QLabel("Y: 0.000"),
            'z': QLabel("Z: 0.000")
        }

        # 设置数值标签样式
        for values in [self.acc_values, self.gyro_values]:
            for label in values.values():
                label.setStyleSheet("""
                    color: white;
                    background-color: #34495e;
                    padding: 5px;
                    border-radius: 5px;
                    margin: 2px;
                """)

        # 组装布局
        self.main_layout.addLayout(status_layout)
        self.main_layout.addLayout(plots_layout)
        self.main_layout.addLayout(values_layout)

    def on_card_click(self, event):
        """处卡片点击事件"""
        if not self.is_expanded:
            self.expand_card()

    def expand_card(self):
        """放大显示当前卡片"""
        self.expanded_window = QDialog(self.parent_window)
        # 同样修改扩展窗口的标题
        segment_num = str(self.port)[-1]
        self.expanded_window.setWindowTitle(f"体节{segment_num} 详细数据")
        self.expanded_window.setStyleSheet("""
            QDialog {
                background-color: #1a1a1a;
            }
        """)

        # 创建布局
        layout = QVBoxLayout()

        # 添加返按钮
        back_btn = QPushButton("返回总览")
        back_btn.setStyleSheet("""
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 10px;
                border-radius: 5px;
                font-size: 14px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
        """)
        back_btn.clicked.connect(self.expanded_window.close)

        # 创建放大版的图表
        expanded_plots = QWidget()
        plots_layout = QVBoxLayout()

        # 创建更大的图表
        acc_plot = self.create_plot("加速度 (m/s²)", expanded=True)
        gyro_plot = self.create_plot("角速度 (rad/s)", expanded=True)

        plots_layout.addWidget(acc_plot)
        plots_layout.addWidget(gyro_plot)
        expanded_plots.setLayout(plots_layout)

        # 添加到主布局
        layout.addWidget(back_btn)
        layout.addWidget(expanded_plots)

        # 添加例
        legend = self.create_expanded_legend()
        layout.addWidget(legend)

        self.expanded_window.setLayout(layout)
        self.expanded_window.resize(1200, 800)
        self.expanded_window.show()

    def create_expanded_legend(self):
        """创建放大视图的图例"""
        legend = QWidget()
        layout = QHBoxLayout()

        colors = [
            ("X轴", "#ff4b4b"),
            ("Y轴", "#4bff4b"),
            ("Z轴", "#4b4bff")
        ]

        for axis, color in colors:
            item = QWidget()
            item_layout = QHBoxLayout()

            color_box = QLabel()
            color_box.setFixedSize(30, 30)
            color_box.setStyleSheet(f"""
                background-color: {color};
                border-radius: 5px;
                margin: 3px;
            """)

            label = QLabel(axis)
            label.setStyleSheet("""
                color: white;
                font-size: 14px;
                font-weight: bold;
            """)

            item_layout.addWidget(color_box)
            item_layout.addWidget(label)

            item.setLayout(item_layout)
            layout.addWidget(item)

        legend.setLayout(layout)
        legend.setStyleSheet("""
            background-color: #2d2d2d;
            border-radius: 10px;
            padding: 10px;
            margin: 10px;
        """)

        return legend

    def create_plot(self, title, expanded=False):
        """创建图表，支持普通和放大两种模式"""
        plot = pg.PlotWidget()
        plot.setBackground('#1e1e1e')
        plot.showGrid(x=True, y=True, alpha=0.2)

        # 根据模式设置不同的字体大小
        font_size = '14pt' if expanded else '10pt'
        plot.setTitle(title, color='#ffffff', size=font_size)
        plot.setLabel('left', 'Value', color='#ffffff', size=font_size)
        plot.setLabel('bottom', 'Samples', color='#ffffff', size=font_size)

        # 创建曲线
        curves = {
            'x': plot.plot(pen=pg.mkPen('#ff4b4b', width=3 if expanded else 2), name='X轴'),
            'y': plot.plot(pen=pg.mkPen('#4bff4b', width=3 if expanded else 2), name='Y轴'),
            'z': plot.plot(pen=pg.mkPen('#4b4bff', width=3 if expanded else 2), name='Z轴')
        }

        if title not in self.curves:
            self.curves[title] = curves

        return plot

    def create_legend(self):
        """创建图例"""
        legend = QWidget()
        layout = QVBoxLayout()

        # 创建图例标题
        title = QLabel("轴线说明")
        title.setStyleSheet("""
            color: white;
            font-size: 14px;
            font-weight: bold;
            padding: 5px;
        """)
        layout.addWidget(title)

        # 创建颜色说明
        colors = [
            ("X轴", "#ff4b4b"),
            ("Y轴", "#4bff4b"),
            ("Z轴", "#4b4bff")
        ]

        for axis, color in colors:
            item = QWidget()
            item_layout = QHBoxLayout()

            color_box = QLabel()
            color_box.setFixedSize(20, 20)
            color_box.setStyleSheet(f"""
                background-color: {color};
                border-radius: 3px;
                margin: 2px;
            """)

            label = QLabel(axis)
            label.setStyleSheet("color: white; font-size: 12px;")

            item_layout.addWidget(color_box)
            item_layout.addWidget(label)
            item_layout.addStretch()

            item.setLayout(item_layout)
            layout.addWidget(item)

        layout.addStretch()
        legend.setLayout(layout)
        legend.setStyleSheet("""
            background-color: #2d2d2d;
            border-radius: 5px;
            padding: 5px;
            margin: 5px;
            max-width: 150px;
        """)

        return legend

    def update_status(self, is_connected, frequency=0):
        """更新状态示"""
        if is_connected:
            self.status_indicator.setStyleSheet("color: #2ecc71; font-size: 20px;")
            self.status_text.setText(f"在线 ({frequency} Hz)")
        else:
            self.status_indicator.setStyleSheet("color: #e74c3c; font-size: 20px;")
            self.status_text.setText("离线")

    def update_values(self, acc_data, gyro_data):
        """更新数值显示"""
        for i, axis in enumerate(['x', 'y', 'z']):
            self.acc_values[axis].setText(f"{axis.upper()}: {acc_data[i]:.3f}")
            self.gyro_values[axis].setText(f"{axis.upper()}: {gyro_data[i]:.3f}")

    def update_plots(self, port_data):
        """更新图表"""
        if len(port_data['time']) > 0:
            t = list(range(len(port_data['time'])))

            # 新加速度图
            self.curves['加速度 (m/s²)']['x'].setData(t, list(port_data['acc_x']))
            self.curves['加速度 (m/s²)']['y'].setData(t, list(port_data['acc_y']))
            self.curves['加速度 (m/s²)']['z'].setData(t, list(port_data['acc_z']))

            # 更新角速度图
            self.curves['角速度 (rad/s)']['x'].setData(t, list(port_data['gyro_x']))
            self.curves['角速度 (rad/s)']['y'].setData(t, list(port_data['gyro_y']))
            self.curves['角速度 (rad/s)']['z'].setData(t, list(port_data['gyro_z']))

class ModernIMUWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('BSRL-SerpentiFlex-Dual-Mode-Robot')
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
            }
        """)

        # 创建中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # 创建主布局
        main_layout = QHBoxLayout(central_widget)

        # 创建卡片网格布局
        cards_widget = QWidget()
        cards_layout = QGridLayout(cards_widget)
        cards_layout.setSpacing(10)

        # 创建IMU卡片
        self.imu_cards = {}
        for i, port in enumerate(PORTS):
            card = IMUCard(port)
            self.imu_cards[port] = card
            cards_layout.addWidget(card, i // 2, i % 2)

        # 添加卡片区域和控制面板
        main_layout.addWidget(cards_widget, stretch=4)
        main_layout.addWidget(self.create_control_panel(), stretch=1)

        # 设置定时器更新数据
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_all)
        self.update_timer.start(50)  # 20Hz新

        self.attitude_window = None
        self.attitude_widget = None

    def create_control_panel(self):
        """创建控制面板"""
        control_panel = QWidget()
        control_panel.setMaximumWidth(350)  # 增加宽度
        control_panel.setStyleSheet("""
            QWidget {
                background-color: #2d2d2d;
                border-radius: 10px;
            }
            QLabel {
                color: white;  /* 设置所有标签文字为白色 */
            }
        """)

        layout = QVBoxLayout(control_panel)
        layout.setSpacing(10)  # 增加垂直间距

        # 数据说明区域
        legend_group = QGroupBox("数据说明")
        legend_group.setMaximumHeight(300)  # 增加高度
        legend_group.setStyleSheet("""
            QGroupBox {
                color: white;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 6px;
                margin-top: 12px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QLabel {
                color: white;
                font-size: 13px;
            }
        """)

        legend_layout = QVBoxLayout()
        legend_layout.setSpacing(6)  # 增加内部间距

        # 添加轴线说明
        axes_widget = QWidget()
        axes_layout = QVBoxLayout()
        axes_layout.setSpacing(6)  # 增加轴线说明的间距

        colors = [
            ("X轴", "#ff4b4b", "Roll"),
            ("Y轴", "#4bff4b", "Pitch"),
            ("Z轴", "#4b4bff", "Yaw")
        ]

        for axis, color, rotation in colors:
            item = QWidget()
            item_layout = QHBoxLayout()
            item_layout.setContentsMargins(3, 3, 3, 3)  # 增加边距

            color_box = QLabel()
            color_box.setFixedSize(20, 20)  # 增加颜色框大小
            color_box.setStyleSheet(f"""
                background-color: {color};
                border-radius: 3px;
                margin: 2px;
            """)

            label = QLabel(f"{axis} ({rotation})")
            label.setStyleSheet("color: white; font-size: 13px;")

            item_layout.addWidget(color_box)
            item_layout.addWidget(label)
            item_layout.addStretch()

            item.setLayout(item_layout)
            axes_layout.addWidget(item)

        axes_widget.setLayout(axes_layout)
        legend_layout.addWidget(axes_widget)

        # 添加状态指示说明
        status_widget = QWidget()
        status_layout = QVBoxLayout()
        status_layout.setSpacing(6)  # 增加状态说明的间距

        status_info = [
            ("✅", "在线 (>95%)", "#2ecc71"),
            ("⚠️", "异常 (80-95%)", "#f1c40f"),
            ("❌", "离线 (<80%)", "#e74c3c")
        ]

        for icon, text, color in status_info:
            item = QWidget()
            item_layout = QHBoxLayout()
            item_layout.setContentsMargins(3, 3, 3, 3)  # 增加边距

            icon_label = QLabel(icon)
            icon_label.setStyleSheet("font-size: 14px;")
            text_label = QLabel(text)
            text_label.setStyleSheet("color: white; font-size: 13px;")

            item_layout.addWidget(icon_label)
            item_layout.addWidget(text_label)
            item_layout.addStretch()

            item.setLayout(item_layout)
            status_layout.addWidget(item)

        status_widget.setLayout(status_layout)
        legend_layout.addWidget(status_widget)

        legend_group.setLayout(legend_layout)
        layout.addWidget(legend_group)

        # 4. 系统状态区
        status_group = QGroupBox("系统状态")
        status_group.setStyleSheet("""
            QGroupBox {
                color: white;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 6px;
                margin-top: 12px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        status_layout = QVBoxLayout()

        # 添加端口状态
        self.status_labels = {}
        for port in PORTS:
            port_widget = QWidget()
            port_layout = QHBoxLayout()

            status_dot = QLabel("●")
            status_dot.setStyleSheet("color: #e74c3c;")  # 默认红色（离线）

            status_label = QLabel(f"端口 {port}")
            status_label.setStyleSheet("color: white;")
            freq_label = QLabel("0 Hz")
            freq_label.setStyleSheet("color: white;")

            port_layout.addWidget(status_dot)
            port_layout.addWidget(status_label)
            port_layout.addWidget(freq_label)

            port_widget.setLayout(port_layout)
            status_layout.addWidget(port_widget)

            self.status_labels[port] = {
                'dot': status_dot,
                'label': status_label,
                'freq': freq_label
            }

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # 添加命令控制区
        command_group = QGroupBox("命令控制")
        command_layout = QVBoxLayout()
        command_group.setStyleSheet("""
            QGroupBox {
                color: white;
                font-size: 14px;
                font-weight: bold;
                border: 2px solid #3498db;
                border-radius: 6px;
                margin-top: 12px;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                padding: 8px;
                border-radius: 4px;
                font-size: 12px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QSpinBox {
                background-color: #2c3e50;
                color: white;
                border: 1px solid #3498db;
                border-radius: 4px;
                padding: 5px;
                font-size: 12px;
            }
            QLabel {
                color: white;
                font-size: 12px;
            }
        """)

        # 步态模式控制
        gait_widget = QWidget()
        gait_layout = QHBoxLayout()
        gait_btn = QPushButton("发送蠕虫步态")
        gait_param = QSpinBox()
        gait_param.setRange(1, 8)
        gait_param.setValue(1)
        gait_layout.addWidget(gait_btn)
        gait_layout.addWidget(gait_param)
        gait_widget.setLayout(gait_layout)
        gait_btn.clicked.connect(lambda: self.send_control_command(MODE_WORM, gait_param.value()))
        command_layout.addWidget(gait_widget)

        # 蛇形模式控制
        snake_widget = QWidget()
        snake_layout = QHBoxLayout()
        snake_btn = QPushButton("发送蛇形直线命令")
        snake_angle = QSpinBox()
        snake_angle.setRange(0, 90)
        snake_angle.setValue(30)
        snake_layout.addWidget(snake_btn)
        angle_label = QLabel("角度:")
        snake_layout.addWidget(angle_label)
        snake_layout.addWidget(snake_angle)
        snake_widget.setLayout(snake_layout)
        snake_btn.clicked.connect(lambda: self.send_control_command(MODE_SNAKE_FORWARD,
                                snake_angle.value()))  # 直接传入角度值
        command_layout.addWidget(snake_widget)

        # 侧向运动控制
        lateral_widget = QWidget()
        lateral_layout = QHBoxLayout()
        lateral_btn = QPushButton("发送蛇形侧向命令")
        lateral_angle = QSpinBox()
        lateral_angle.setRange(0, 90)
        lateral_angle.setValue(30)
        lateral_layout.addWidget(lateral_btn)
        angle_label = QLabel("角度:")
        lateral_layout.addWidget(angle_label)
        lateral_layout.addWidget(lateral_angle)
        lateral_widget.setLayout(lateral_layout)
        lateral_btn.clicked.connect(lambda: self.send_control_command(MODE_SNAKE_LATERAL,
                                  lateral_angle.value()))  # 直接传入角度值
        command_layout.addWidget(lateral_widget)

        # 圆周运动控制
        circular_widget = QWidget()
        circular_layout = QHBoxLayout()
        circular_btn = QPushButton("发送蛇形圆周命令")
        circular_angle = QSpinBox()
        circular_angle.setRange(0, 90)
        circular_angle.setValue(30)
        circular_layout.addWidget(circular_btn)
        angle_label = QLabel("角度:")
        circular_layout.addWidget(angle_label)
        circular_layout.addWidget(circular_angle)
        circular_widget.setLayout(circular_layout)
        circular_btn.clicked.connect(lambda: self.send_control_command(MODE_SNAKE_CIRCULAR,
                                   circular_angle.value()))  # 直接传入角度值
        command_layout.addWidget(circular_widget)

        command_group.setLayout(command_layout)
        layout.addWidget(command_group)

        return control_panel

    def update_all(self):
        """更新所有显示内容"""
        for port in PORTS:
            if port in imu_update_frequency:
                freq = imu_update_frequency[port]
                percentage = (freq / 100) * 100  # 目标频率100Hz

                # 根据频率百分比选择状态图标
                if percentage >= 95:
                    status_icon = "✅"
                elif percentage >= 80:
                    status_icon = "⚠️"
                else:
                    status_icon = "❌"

                self.status_labels[port]['dot'].setText(status_icon)
                self.status_labels[port]['label'].setText(f"端口 {port}")
                self.status_labels[port]['freq'].setText(f"{freq} Hz")

                # 更新IMU卡片状态
                self.imu_cards[port].update_status(True, freq)

                try:
                    # 更新数据显示
                    with lock:
                        if len(plot_data[port]['time']) > 0:
                            acc_data = [
                                plot_data[port]['acc_x'][-1],
                                plot_data[port]['acc_y'][-1],
                                plot_data[port]['acc_z'][-1]
                            ]  # 添加缺失的右括号
                            gyro_data = [
                                plot_data[port]['gyro_x'][-1],
                                plot_data[port]['gyro_y'][-1],
                                plot_data[port]['gyro_z'][-1]
                            ]  # 添加缺失的右括号
                            self.imu_cards[port].update_values(acc_data, gyro_data)
                            self.imu_cards[port].update_plots(plot_data[port])
                except Exception as e:
                    print(f"更新显示错误 (端口 {port}): {e}")
            else:
                # 如果端口不在频率字典中，显示为离线状态
                self.status_labels[port]['dot'].setText("❌")
                self.status_labels[port]['label'].setText(f"端口 {port} (离线)")
                self.status_labels[port]['freq'].setText("0 Hz")
                self.imu_cards[port].update_status(False)

    def save_and_plot_data(self, imu_data, state, port, window=None):
        """保存数据并更新图表数据"""
        logger = port_loggers[port]
        try:
            # 获取IMU数据
            acc = imu_data["acceleration"]
            gyro = imu_data["gyroscope"]

            # 更新绘图数据
            with lock:  # 添加锁保护
                if len(plot_data[port]['time']) >= PLOT_WINDOW:
                    plot_data[port]['time'].popleft()
                    plot_data[port]['acc_x'].popleft()
                    plot_data[port]['acc_y'].popleft()
                    plot_data[port]['acc_z'].popleft()
                    plot_data[port]['gyro_x'].popleft()
                    plot_data[port]['gyro_y'].popleft()
                    plot_data[port]['gyro_z'].popleft()

                # 添加新数据
                current_time = time.time()
                plot_data[port]['time'].append(current_time)
                plot_data[port]['acc_x'].append(acc[0])
                plot_data[port]['acc_y'].append(acc[1])
                plot_data[port]['acc_z'].append(acc[2])
                plot_data[port]['gyro_x'].append(filters[port]['gyro_x'].filter(gyro[0]))
                plot_data[port]['gyro_y'].append(filters[port]['gyro_y'].filter(gyro[1]))
                plot_data[port]['gyro_z'].append(filters[port]['gyro_z'].filter(gyro[2]))

            # 保存数据到文件，添加步态类型
            filename = f"{filepath}imu_data_{port}.txt"
            data_str = f"{current_gait_type} {state} "  # 添加步态类型字段
            data_str += " ".join(f"{v:.6f}" for v in acc)
            data_str += " " + " ".join(f"{v:.6f}" for v in gyro)
            data_str += " " + " ".join(f"{v:.6f}" for v in imu_data["orientation"])
            data_str += "\n"

            with open(filename, 'a') as f:
                f.write(data_str)

        except Exception as e:
            logger.error(f"❌ 数据处理错误: {e}")

    def send_control_command(self, command_type, parameter):
        """发送控制命令到所有体节"""
        global current_gait_type  # 添加全局变量声明
        try:
            print("\n===== 命令发送调试信息 =====")
            print(f"命令类型: 0x{command_type:02X}")
            print(f"参数值: {parameter}")

            # 更新当前步态类型
            current_gait_type = command_type
            print(f"当前步态类型已更新为: 0x{current_gait_type:02X}")

            # 检查所有体节端口的连接状态
            command_ports = [12340, 12341, 12342, 12343, 12344]  # 所有体节的端口
            connected_ports = []

            for port in command_ports:
                if port in active_connections:
                    connected_ports.append(port)
                else:
                    print(f"警告: 端口 {port} 没有可用的TCP连接")

            if not connected_ports:
                self.statusBar().showMessage("错误: 没有可用的TCP连接", 2000)
                return

            print(f"TCP连接状态: {len(connected_ports)} 个端口已连接")
            cmd = create_command(command_type, parameter)
            print(f"发送的命令内容: {[hex(x) for x in cmd]}")

            # 向所有已连接的端口发送命令
            success_count = 0
            for port in connected_ports:
                try:
                    if send_command(active_connections[port], cmd):
                        success_count += 1
                        print(f"端口 {port} 命令发送成功")
                    else:
                        print(f"端口 {port} 命令发送失败")
                except Exception as e:
                    print(f"端口 {port} 发送错误: {str(e)}")

            mode_str = {
                MODE_NONE: "未设置步态",
                MODE_WORM: "蠕虫步态",
                MODE_SNAKE_FORWARD: "蛇形直线",
                MODE_SNAKE_LATERAL: "蛇形侧向",
                MODE_SNAKE_CIRCULAR: "蛇形圆周"
            }.get(command_type, "未知")

            # 显示发送结果
            if success_count > 0:
                self.statusBar().showMessage(
                    f"命令发送成功: 类型={mode_str}, 参数={parameter} (成功: {success_count}/{len(connected_ports)}端口)",
                    2000
                )
            else:
                self.statusBar().showMessage("所有端口发送失败", 2000)

        except Exception as e:
            print(f"发送错误: {str(e)}")
            self.statusBar().showMessage(f"发送错误: {str(e)}", 2000)

# 添加IMU姿态解算相关的函数
class IMUProcessor:
    def __init__(self, sample_freq=100.0):
        self.sample_freq = sample_freq
        self.madgwick = Madgwick(frequency=sample_freq)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # 初始四元数
        self.euler_angles = np.zeros(3)  # [roll, pitch, yaw]

    def update(self, gyro, acc):
        """
        更新IMU姿态
        :param gyro: 角速度 [wx, wy, wz] (rad/s)
        :param acc: 加速度 [ax, ay, az] (m/s^2)
        """
        # 使用Madgwick滤波器更新姿
        self.quaternion = self.madgwick.updateIMU(
            self.quaternion,
            np.array(gyro),
            np.array(acc)
        )

        # 计算欧拉角 (rad)
        self.euler_angles = self.quaternion_to_euler(self.quaternion)

        return self.quaternion, self.euler_angles

    @staticmethod
    def quaternion_to_euler(q):
        """
        四元数转欧拉角
        :return: [roll, pitch, yaw] in radians
        """
        # 提取四元数分量
        w, x, y, z = q

        # 计算欧拉角
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

        return np.array([roll, pitch, yaw])

# 为每个端口创建IMU处理器
imu_processors = {port: IMUProcessor() for port in PORTS}

class ZeroVelocityDetector:
    def __init__(self, threshold=0.01, window_size=10):
        self.acc_threshold = threshold  # 加速度阈值
        self.gyro_threshold = threshold  # 角速度阈值
        self.window_size = window_size
        self.acc_window = []
        self.gyro_window = []

    def is_zero_velocity(self, acc, gyro):
        """
        检测是否处于零速度状态
        :param acc: [ax, ay, az] 加速度数据
        :param gyro: [wx, wy, wz] 角速度数据
        :return: bool 是否处于零速度状态
        """
        # 计算加速度和角速度的模
        acc_magnitude = math.sqrt(sum(x*x for x in acc))
        gyro_magnitude = math.sqrt(sum(x*x for x in gyro))

        # 更新窗口数据
        self.acc_window.append(acc_magnitude)
        self.gyro_window.append(gyro_magnitude)
        if len(self.acc_window) > self.window_size:
            self.acc_window.pop(0)
            self.gyro_window.pop(0)

        # 计算窗口内的平均值
        if len(self.acc_window) == self.window_size:
            acc_mean = sum(self.acc_window) / self.window_size
            gyro_mean = sum(self.gyro_window) / self.window_size

            # 判断是否为零速度状态
            is_zero = (abs(acc_mean - 9.81) < self.acc_threshold and  # 重力加速度约为9.81
                      gyro_mean < self.gyro_threshold)
            return is_zero

        return False

# 每个端口创建零速度检测器
zero_velocity_detectors = {port: ZeroVelocityDetector() for port in PORTS}

def clear_plot_data(port):
    """清理指定端口的绘图数据"""
    plot_data[port] = {
        'time': deque(maxlen=PLOT_WINDOW),
        'acc_x': deque(maxlen=PLOT_WINDOW),
        'acc_y': deque(maxlen=PLOT_WINDOW),
        'acc_z': deque(maxlen=PLOT_WINDOW),
        'gyro_x': deque(maxlen=PLOT_WINDOW),
        'gyro_y': deque(maxlen=PLOT_WINDOW),
        'gyro_z': deque(maxlen=PLOT_WINDOW),
        'euler_x': deque(maxlen=PLOT_WINDOW),
        'euler_y': deque(maxlen=PLOT_WINDOW),
        'euler_z': deque(maxlen=PLOT_WINDOW)
    }

def send_heartbeat(client_socket, port):
    """发送心跳包"""
    logger = port_loggers[port]
    while not stop_event.is_set():
        try:
            client_socket.send(b'\x00\x00')  # 发送心跳包
            time.sleep(1)  # 每秒发送一次
        except Exception as e:
            logger.error(f"❌ 心跳包发送失败: {str(e)}")
            break

def send_command(socket, command):
    """
    发送命令到ESP32
    :param socket: TCP socket连接
    :param command: 命令字节数据
    :return: bool 是否发送成功
    """
    try:
        socket.send(command)
        return True
    except Exception as e:
        print(f"发送命令失败: {e}")
        return False

def main():
    """主函数"""
    if not os.path.exists(filepath):
        os.makedirs(filepath)

    # 创建 QApplication 实例
    app = QApplication(sys.argv)

    # 创建并显示绘图窗口
    window = ModernIMUWindow()
    window.show()

    # 启动服务器线程
    server_threads = start_tcp_servers()

    # 运行应用程序
    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        print("\n正在关闭服务器...")
        stop_event.set()
        for thread in server_threads:
            thread.join()
        print("服务器已关闭")

if __name__ == '__main__':
    main()
