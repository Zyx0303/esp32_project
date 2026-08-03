import socket
import time
import logging
import threading
from datetime import datetime
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                            QHBoxLayout, QLabel, QPushButton, QTextEdit, QGridLayout, QGroupBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import sys

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# 服务器配置
SERVER_IP = "192.168.3.246"
SERVER_PORTS = [12340, 12341, 12342, 12343, 12344]

# 在类定义前添加日志配置
logger = logging.getLogger('TCPConnection')
logger.setLevel(logging.INFO)

class TCPConnection:
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket = None
        self.connected = False

    def connect(self):
        if not self.connected:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(1.0)
                self.socket.connect((SERVER_IP, self.port))
                self.connected = True
                logger.info(f"✨ 已连接到端口 {self.port}")
                return True
            except Exception as e:
                logger.error(f"❌ 连接失败: {str(e)}")
                if self.socket:
                    self.socket.close()
                    self.socket = None
                return False
        return True

    def send(self, data):
        try:
            if not self.connected:
                if not self.connect():
                    return False

            # 发送数据
            self.socket.send(data)
            logger.info(f"📤 发送数据到端口 {self.port}")
            return True
        except Exception as e:
            logger.error(f"❌ 发送失败: {str(e)}")
            self.connected = False
            if self.socket:
                self.socket.close()
                self.socket = None
            return False

    def close(self):
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False

class CommandWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 控制面板")
        self.setGeometry(100, 100, 1200, 800)

        # 初始化连接
        self.connections = {
            port: TCPConnection(SERVER_IP, port)
            for port in SERVER_PORTS
        }
        logger.info("✅ TCP连接管理器初始化完成")

        # 创建主窗口部件
        main_widget = QWidget()
        self.setCentralWidget(main_widget)

        # 创建主布局
        layout = QHBoxLayout(main_widget)

        # 左侧状态面板
        status_panel = self.create_status_panel()

        # 右侧控制面板
        control_panel = self.create_control_panel()

        # 添加面板到主布局
        layout.addWidget(status_panel, 1)
        layout.addWidget(control_panel, 2)

        # 设置定时器更新状态
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.check_ports)
        self.update_timer.start(500)

    def create_status_panel(self):
        status_panel = QWidget()
        status_layout = QVBoxLayout(status_panel)
        status_layout.setSpacing(20)

        # 状态标题
        status_title = QLabel("ESP32 连接状态")
        status_title.setFont(QFont('Arial', 14, QFont.Bold))
        status_layout.addWidget(status_title)

        # 状态指示器网格
        self.status_indicators = {}
        status_grid = QGridLayout()
        for i, port in enumerate(SERVER_PORTS):
            port_label = QLabel(f"端口 {port}:")
            status_indicator = QLabel("🔴 未连接")
            status_indicator.setFont(QFont('Arial', 12))
            self.status_indicators[port] = status_indicator

            status_grid.addWidget(port_label, i, 0)
            status_grid.addWidget(status_indicator, i, 1)

        status_layout.addLayout(status_grid)
        status_layout.addStretch()
        return status_panel

    def create_control_panel(self):
        control_panel = QWidget()
        control_layout = QVBoxLayout(control_panel)

        # 控制标题
        control_title = QLabel("步态控制")
        control_title.setFont(QFont('Arial', 14, QFont.Bold))
        control_layout.addWidget(control_title)

        # 蠕虫步态按钮组
        worm_group = QGroupBox("蠕虫步态")
        worm_layout = QVBoxLayout()
        for i in range(7):  # 0-6号步态
            btn = QPushButton(f"蠕虫步态 {i}")
            btn.clicked.connect(lambda checked, x=i: self.send_worm_gait(x))
            worm_layout.addWidget(btn)
        worm_group.setLayout(worm_layout)

        # 蛇形步态按钮组
        snake_group = QGroupBox("蛇形步态")
        snake_layout = QVBoxLayout()
        snake_angles = [30, 20]  # 可用的最大角度
        for angle in snake_angles:
            btn = QPushButton(f"蛇形步态（最大角{angle}°）")
            btn.clicked.connect(lambda checked, a=angle: self.send_snake_gait(a))
            snake_layout.addWidget(btn)
        snake_group.setLayout(snake_layout)

        # 目标端口选择
        port_group = QGroupBox("目标端口")
        port_layout = QHBoxLayout()
        self.send_all_btn = QPushButton("发送到所有端口")
        self.send_all_btn.clicked.connect(self.send_to_all)
        port_layout.addWidget(self.send_all_btn)

        self.target_port = None  # 当前选中的端口
        for port in SERVER_PORTS:
            btn = QPushButton(f"端口 {port}")
            btn.clicked.connect(lambda checked, p=port: self.set_target_port(p))
            port_layout.addWidget(btn)
        port_group.setLayout(port_layout)

        # 日志显示区域
        self.log_display = QTextEdit()
        self.log_display.setReadOnly(True)

        # 添加所有组件到控制面板
        control_layout.addWidget(worm_group)
        control_layout.addWidget(snake_group)
        control_layout.addWidget(port_group)
        control_layout.addWidget(self.log_display)

        return control_panel

    def set_target_port(self, port):
        """设置目标端口"""
        self.target_port = port
        self.log_message(f"已选择端口: {port}")

    def send_worm_gait(self, gait_no):
        """发送蠕虫步态命令"""
        command = bytes([
            0xFF, 0xFA,      # 帧头
            0x01, gait_no+1, # 模式1(蠕虫步态)，参数为步态号+1
            0x88, 0x77       # 帧尾
        ])
        self.send_command(command, f"蠕虫步态 {gait_no}")

    def send_snake_gait(self, angle):
        """发送蛇形步态命令"""
        command = bytes([
            0xFF, 0xFA,    # 帧头
            0x02, angle,   # 模式2(蛇形步态)，参数为最大角度
            0x88, 0x77     # 帧尾
        ])
        self.send_command(command, f"蛇形步态（最大角{angle}°）")

    def send_command(self, command, description):
        """发送命令的通用方法"""
        if self.target_port is None and not self.send_to_all:
            self.log_message("❌ 请先选择目标端口")
            return

        # 打印完整的命令内容
        self.log_message(f"准备发送命令:")
        hex_str = ' '.join([f'0x{b:02X}' for b in command])
        self.log_message(f"命令内容: {hex_str}")

        if self.target_port:
            # 发送到特定端口
            connection = self.connections[self.target_port]
            if connection.send(command):
                self.log_message(f"📤 发送{description}到端口 {self.target_port}")
                self.log_message(f"发送成功，字节数: {len(command)}")
            else:
                self.log_message(f"❌ 发送失败 (端口 {self.target_port})")
        else:
            # 发送到所有端口
            for port, connection in self.connections.items():
                if connection.connected:
                    if connection.send(command):
                        self.log_message(f"📤 发送{description}到端口 {port}")
                    else:
                        self.log_message(f"❌ 发送失败 (端口 {port})")

    def check_ports(self):
        """检查每个端口是否有活跃连接"""
        for port, connection in self.connections.items():
            if not connection.connected:
                # 尝试建立连接
                if connection.connect():
                    self.status_indicators[port].setText("🟢 已连接")
                else:
                    self.status_indicators[port].setText("🔴 未连接")
            else:
                self.status_indicators[port].setText("🟢 已连接")

    def send_to_port(self, port):
        """向指定端口发送命令"""
        connection = self.connections[port]

        # 修改指令格式，用于测试0号电机转动20度
        command = bytes([
            0xFF, 0xFA,  # 帧头
            0x01, 0x02,  # 模式和参数 (与STM32代码中的模式对应)
            0x88, 0x77   # 帧尾
        ])

        if connection.send(command):
            self.log_message(f"📤 发送舵机控制指令到端口 {port}: {command.hex()}")
            # 添加详细的调试信息
            hex_str = ' '.join([f'0x{b:02X}' for b in command])
            self.log_message(f"指令详情: {hex_str}")
        else:
            self.log_message(f"❌ 发送失败 (端口 {port})")

    def send_to_all(self):
        """向所有已连接的端口发送命令"""
        for port, connection in self.connections.items():
            if connection.connected:
                self.send_to_port(port)

    def log_message(self, message):
        """添加日志消息"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_display.append(f"[{timestamp}] {message}")

    def closeEvent(self, event):
        """窗口关闭时清理连接"""
        for connection in self.connections.values():
            connection.close()
        event.accept()

def main():
    """主函数"""
    app = QApplication(sys.argv)
    window = CommandWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
