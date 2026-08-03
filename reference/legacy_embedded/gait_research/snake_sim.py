import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib

# 设置中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
matplotlib.rcParams['axes.unicode_minus'] = False    # 用来正常显示负号

# 添加新的参数配置类
class SnakeParams:
    def __init__(self):
        # 基本运动参数
        self.frequency = 1.4     # 运动频率 Hz
        self.amplitude = 30.0    # 运动幅度(度)
        self.phase_bias = np.pi/2  # 相位差
        self.n_segments = 5      # 节段数量
        self.segment_length = 0.2  # 节段长度

        # 高级参数
        self.coupling_weight = 1.0  # 耦合强度
        self.mu_t = 0.3    # 切向摩擦系数
        self.mu_n = 1.0    # 法向摩擦系数
        self.v_f = 0.3     # 前进速度

        # 运动模式
        self.gait_mode = 'serpentine'  # 'serpentine' or 'worm'
        self.turning_bias = 0.0  # 转向偏置

# 在 SnakeParams 类之后添加 SnakeSimulator 类
class SnakeSimulator:
    """蛇形机器人仿真器"""
    def __init__(self, params: SnakeParams):
        self.params = params
        self.time = 0
        self.x_offset = 0
        self.y_offset = 0
        self.phase = 0

        # 创建图形对象
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))

        # 存储历史数据
        self.angle_history = {i: [] for i in range(self.params.n_segments - 1)}
        self.time_history = []
        self.trajectory_x = []
        self.trajectory_y = []

    def calculate_joint_angles(self):
        """计算各关节角度"""
        angles = []
        for i in range(self.params.n_segments - 1):
            if self.params.gait_mode == 'serpentine':
                # 蛇形运动
                angle = self.params.amplitude * np.sin(
                    2 * np.pi * self.params.frequency * self.time -
                    i * self.params.phase_bias
                )
            else:
                # 蠕虫运动
                angle = self.params.amplitude * np.sin(
                    2 * np.pi * self.params.frequency * self.time
                ) * np.exp(-i / 2)  # 振幅随节段衰减
            angles.append(angle)
        return np.array(angles)

    def calculate_positions(self, angles):
        """计算各节段位置"""
        x_positions = []
        y_positions = []

        # 计算头部位置（考虑整体运动）
        heading_angle = sum(np.deg2rad(angles))  # 计算整体朝向
        self.x_offset += self.params.v_f * np.cos(heading_angle) * 0.1
        self.y_offset += self.params.v_f * np.sin(heading_angle) * 0.1

        x_current = self.x_offset
        y_current = self.y_offset
        x_positions.append(x_current)
        y_positions.append(y_current)

        # 计算各节段位置
        cumulative_angle = 0
        for i in range(self.params.n_segments):
            if i < len(angles):
                cumulative_angle += np.deg2rad(angles[i])

            x_current = x_positions[-1] + self.params.segment_length * np.cos(cumulative_angle)
            y_current = y_positions[-1] + self.params.segment_length * np.sin(cumulative_angle)

            x_positions.append(x_current)
            y_positions.append(y_current)

        return np.array(x_positions), np.array(y_positions)

    def update(self, frame):
        """更新动画帧"""
        self.time += 0.1

        # 清除图形
        self.ax1.clear()
        self.ax2.clear()

        # 计算关节角度和位置
        angles = self.calculate_joint_angles()
        x, y = self.calculate_positions(angles)

        # 更新轨迹
        self.trajectory_x.append(x[0])
        self.trajectory_y.append(y[0])
        if len(self.trajectory_x) > 100:  # 保持轨迹长度
            self.trajectory_x.pop(0)
            self.trajectory_y.pop(0)

        # 绘制机器人形态
        self.ax1.plot(x, y, 'bo-', linewidth=2, markersize=8)
        self.ax1.plot(self.trajectory_x, self.trajectory_y, 'r--', alpha=0.5)

        # 动态调整显示范围
        x_center = np.mean(x)
        y_center = np.mean(y)
        plot_range = self.params.n_segments * self.params.segment_length * 2
        self.ax1.set_xlim(x_center - plot_range, x_center + plot_range)
        self.ax1.set_ylim(y_center - plot_range, y_center + plot_range)

        # 设置图形属性
        self.ax1.grid(True)
        self.ax1.set_aspect('equal')
        self.ax1.set_title('蛇形机器人运动仿真')

        # 更新角度历史
        self.time_history.append(self.time)
        for i, angle in enumerate(angles):
            self.angle_history[i].append(angle)

        # 保持历史数据长度
        if len(self.time_history) > 100:
            self.time_history.pop(0)
            for i in self.angle_history:
                self.angle_history[i].pop(0)

        # 绘制角度变化
        for i in range(len(angles)):
            self.ax2.plot(self.time_history, self.angle_history[i],
                         label=f'关节 {i+1}')

        self.ax2.grid(True)
        self.ax2.legend()
        self.ax2.set_title('关节角度变化')
        self.ax2.set_xlabel('时间 (s)')
        self.ax2.set_ylabel('角度 (°)')

        self.fig.tight_layout()
        return self.ax1, self.ax2

    def animate(self):
        """创建动画"""
        anim = FuncAnimation(
            self.fig,
            self.update,
            frames=None,
            interval=50,
            blit=False,  # 改为False以确保正确更新
            cache_frame_data=False  # 禁用帧缓存
        )
        return anim

class SnakeSimGUI:
    def __init__(self):
        self.params = SnakeParams()
        self.setup_gui()

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("蛇形机器人参数仿真器")

        # 创建主框架
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 创建参数调节区域
        param_frame = ttk.LabelFrame(main_frame, text="参数控制")
        param_frame.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.Y)

        # 添加滑动条
        self.sliders = {}
        self.add_slider(param_frame, "频率 (Hz)", "frequency", 0.1, 3.0, self.params.frequency)
        self.add_slider(param_frame, "幅度 (度)", "amplitude", 0, 45.0, self.params.amplitude)
        self.add_slider(param_frame, "相位差 (π)", "phase_bias", 0, 2.0, self.params.phase_bias/np.pi)
        self.add_slider(param_frame, "耦合强度", "coupling_weight", 0.1, 2.0, self.params.coupling_weight)

        # 添加运动模式选择
        mode_frame = ttk.LabelFrame(param_frame, text="运动模式")
        mode_frame.pack(fill=tk.X, padx=5, pady=5)
        self.mode_var = tk.StringVar(value=self.params.gait_mode)
        ttk.Radiobutton(mode_frame, text="蛇形模式", variable=self.mode_var,
                       value="serpentine").pack()
        ttk.Radiobutton(mode_frame, text="蠕虫模式", variable=self.mode_var,
                       value="worm").pack()

        # 添加控制按钮
        control_frame = ttk.Frame(param_frame)
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        ttk.Button(control_frame, text="开始仿真", command=self.start_simulation).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame, text="停止", command=self.stop_simulation).pack(side=tk.LEFT, padx=2)
        ttk.Button(control_frame, text="重置", command=self.reset_simulation).pack(side=tk.LEFT, padx=2)

        # 创建画布框架
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # 创建matplotlib图形
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def add_slider(self, parent, label, param_name, min_val, max_val, default_val):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=2)
        ttk.Label(frame, text=label).pack(side=tk.LEFT)
        slider = ttk.Scale(frame, from_=min_val, to=max_val, orient=tk.HORIZONTAL)
        slider.set(default_val)
        slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)
        self.sliders[param_name] = slider

    def update_params(self):
        """从滑动条更新参数"""
        self.params.frequency = self.sliders["frequency"].get()
        self.params.amplitude = self.sliders["amplitude"].get()
        self.params.phase_bias = self.sliders["phase_bias"].get() * np.pi
        self.params.coupling_weight = self.sliders["coupling_weight"].get()
        self.params.gait_mode = self.mode_var.get()

    def start_simulation(self):
        """开始仿真"""
        self.update_params()
        if hasattr(self, 'anim'):
            self.anim.event_source.stop()

        # 清除现有图形
        self.ax1.clear()
        self.ax2.clear()

        # 创建仿真对象并开始动画
        self.simulator = SnakeSimulator(self.params)
        self.simulator.fig = self.fig
        self.simulator.ax1 = self.ax1
        self.simulator.ax2 = self.ax2

        self.anim = self.simulator.animate()
        self.canvas.draw()

    def stop_simulation(self):
        """停止仿真"""
        if hasattr(self, 'anim'):
            self.anim.event_source.stop()

    def reset_simulation(self):
        """重置仿真"""
        self.stop_simulation()
        # 重置参数到默认值
        for name, slider in self.sliders.items():
            slider.set(getattr(SnakeParams(), name))
        self.mode_var.set('serpentine')

    def run(self):
        """运行GUI"""
        self.root.mainloop()

def main():
    app = SnakeSimGUI()
    app.run()

if __name__ == "__main__":
    main()
