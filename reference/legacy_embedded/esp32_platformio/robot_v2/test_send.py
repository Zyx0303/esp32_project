import socket

def create_command(command_type, parameter):
    """创建控制命令
    Args:
        command_type: 命令类型 (0x01:步态模式, 0x02:蛇形模式)
        parameter: 命令参数
            - 步态模式: 直接使用参数值
            - 蛇形模式: 角度值(0-90)会被转换为合适的字节值
    """
    if command_type == 0x02:  # 蛇形模式
        # 将角度映射到0-255范围
        # 例如：90度 -> 255, 45度 -> 128
        parameter = int((parameter / 90.0) * 255)

    return bytes([
        0xFF, 0xFA,
        command_type,
        parameter,
        0x88, 0x77
    ])

def print_command_info(command):
    """打印命令详细信息"""
    print("\n发送命令详情:")
    print(f"帧头: 0x{command[0]:02X} 0x{command[1]:02X}")
    print(f"命令: 0x{command[2]:02X} ({get_command_type(command[2])})")
    print(f"参数: 0x{command[3]:02X}")
    print(f"帧尾: 0x{command[4]:02X} 0x{command[5]:02X}")
    print(f"完整命令: {' '.join([f'0x{b:02X}' for b in command])}")

def get_command_type(mode):
    """获取命令类型描述"""
    if mode == 0x01:
        return "步态模式"
    elif mode == 0x02:
        return "蛇形模式"
    return "未知模式"

def send_command(client_socket, command):
    """通过已建立的连接发送命令
    Args:
        client_socket: 已建立的TCP连接socket
        command: 要发送的命令字节序列
    Returns:
        bool: 发送是否成功
    """
    try:
        print_command_info(command)
        client_socket.send(command)
        print(f"[发送] 命令已通过TCP连接发送")
        return True
    except Exception as e:
        print(f"[错误] 发送失败: {str(e)}")
        return False

# 示例使用方法
if __name__ == "__main__":
    # 这里仅作为示例，实际使用时应该通过robot_v2.py中的TCP连接发送
    cmd = create_command(0x01, 0x01)
    print("注意：这是示例代码，实际应通过robot_v2.py中的TCP连接发送命令")
    print("命令内容：")
    print_command_info(cmd)
