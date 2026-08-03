import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.font_manager import FontProperties

# 设置中文字体（如果需要显示中文）
try:
    plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
    plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号
except:
    print("Warning: Chinese font not found, using English instead")

# 定义 Matsuoka 神经振荡器模型
def matsuoka_oscillator(v_flexor, v_extensor, y_flexor, y_extensor, params, dt):
    """
    Matsuoka 振荡器的更新函数。

    参数:
    - v_flexor, v_extensor: 屈肌和伸肌神经元的内部状态
    - y_flexor, y_extensor: 屈肌和伸肌神经元的输出
    - params: 包含振荡器参数的字典
    - dt: 时间步长

    返回:
    - 更新后的 v_flexor, v_extensor, y_flexor, y_extensor
    """
    tau_r = params["tau_r"]  # 上升时间常数
    beta = params["beta"]    # 自抑制权重
    w_ef = params["w_ef"]    # 屈肌和伸肌之间的连接权重
    input_signal = params["input_signal"]  # 外部输入信号

    # 更新内部状态
    dv_flexor = (-v_flexor - beta * y_flexor + w_ef * y_extensor + input_signal) / tau_r
    dv_extensor = (-v_extensor - beta * y_extensor + w_ef * y_flexor + input_signal) / tau_r
    v_flexor += dv_flexor * dt
    v_extensor += dv_extensor * dt

    # 更新输出（非线性激活函数）
    y_flexor = max(0, v_flexor)
    y_extensor = max(0, v_extensor)

    return v_flexor, v_extensor, y_flexor, y_extensor

# 仿真参数
time_steps = 1000   # 时间步数
dt = 0.01           # 时间步长
time = np.arange(0, time_steps * dt, dt)

# 初始化神经振荡器状态
v_flexor, v_extensor = 0.0, 0.0
y_flexor, y_extensor = 0.0, 0.0

# 振荡器��数
params = {
    "tau_r": 0.1,       # 上升时间常数
    "beta": 2.5,        # 自抑制系数
    "w_ef": 1.0,        # 屈肌和伸肌之间的连接权重
    "input_signal": 1.0 # 外部输入信号
}

# 存储输出结果
outputs_flexor = []
outputs_extensor = []

# 仿真神经振荡器
for t in time:
    v_flexor, v_extensor, y_flexor, y_extensor = matsuoka_oscillator(
        v_flexor, v_extensor, y_flexor, y_extensor, params, dt
    )
    outputs_flexor.append(y_flexor)
    outputs_extensor.append(y_extensor)

# 绘制仿真结果
plt.figure(figsize=(12, 6))
plt.plot(time, outputs_flexor, label="Flexor Output (Flexor Output)")
plt.plot(time, outputs_extensor, label="Extensor Output (Extensor Output)", linestyle='dashed')
plt.title('Snake Robot Circular Motion')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.legend()
plt.grid(True)
plt.show()
