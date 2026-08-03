import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass
from typing import Tuple
import matplotlib.patches as patches
from deap import base, creator, tools, algorithms
import random
import matplotlib.gridspec as gridspec
import dataclasses

@dataclass
class CPGParams:
    """CPG和物理参数类"""
    # 基本结构参数
    n_segments: int = 8          # 节段数量
    segment_length: float = 0.2  # 节段长度
    mass: float = 1.0           # 质量

    # CPG核心参数
    frequency: float = 1.0       # 频率(Hz)
    amplitude: float = 30.0      # 运动幅度(度)
    phase_bias: float = np.pi/2  # 相邻振荡器的相位差(弧度)
    coupling_weight: float = 1.0 # 耦合强度
    convergence_rate: float = 1.0# 收敛率σ
    phase_reset_threshold: float = 0.1  # 相位重置阈值

    # 运动参数
    v_f: float = 0.3            # 前进速度

    # 物理参数
    mu_t: float = 0.3           # 切向摩擦系数
    mu_n: float = 1.0           # 法向摩擦系数
    control_gain: float = 15.0   # 控制增益

    # 运动限制
    max_angle: float = 35.0      # 最大关节角度(度)
    min_angle: float = -35.0     # 最小关节角度(度)

    # 仿真参数
    dt: float = 0.001            # 时间步长

    # 转向控制参数
    turning_bias: float = 0.0    # 转向偏置角度
    turning_offset: float = 0.0  # 转向补偿

    @property
    def omega(self) -> float:
        """角频率(rad/s)"""
        return 2 * np.pi * self.frequency

    def set_turning(self, direction: str, intensity: float = 1.0):
        """设置转向参数
        Args:
            direction: 'left' or 'right'
            intensity: 转向强度 (0-1)
        """
        if direction == 'left':
            self.turning_bias = 15.0 * intensity
            self.turning_offset = -10.0 * intensity
        elif direction == 'right':
            self.turning_bias = -15.0 * intensity
            self.turning_offset = 10.0 * intensity
        else:
            self.turning_bias = 0.0
            self.turning_offset = 0.0

def create_gait_params():
    """创建不同步态的参数"""
    gaits = {
        "Normal": CPGParams(
            frequency=1.0,
            amplitude=30.0,
            phase_bias=np.pi/2,
            coupling_weight=1.0,
            convergence_rate=1.0,
            mu_t=0.3,
            mu_n=1.0,
            control_gain=15.0
        ),
        "Fast": CPGParams(
            frequency=2.0,
            amplitude=30.0,
            phase_bias=np.pi/2,
            coupling_weight=1.2,
            convergence_rate=1.5,
            mu_t=0.3,
            mu_n=1.0,
            control_gain=15.0
        ),
        "Large": CPGParams(
            frequency=1.0,
            amplitude=40.0,
            phase_bias=2*np.pi/3,
            coupling_weight=1.0,
            convergence_rate=1.0,
            mu_t=0.3,
            mu_n=1.0,
            control_gain=15.0
        ),
        "Smooth": CPGParams(
            frequency=1.0,
            amplitude=25.0,
            phase_bias=np.pi/3,
            coupling_weight=0.8,
            convergence_rate=0.8,
            mu_t=0.2,  # 降低摩擦系数使运动更平滑
            mu_n=0.8,
            control_gain=10.0
        ),
        "Aggressive": CPGParams(
            frequency=2.5,
            amplitude=35.0,
            phase_bias=2*np.pi/3,
            coupling_weight=1.5,
            convergence_rate=2.0,
            mu_t=0.4,  # 增加摩擦系数提供更强的推进力
            mu_n=1.2,
            control_gain=20.0
        )
    }
    return gaits

def simulate_gait(params: CPGParams, simulation_time: float = 30.0):
    """模拟特定参数下的CPG"""
    cpg = CPGNetwork(params)
    times = np.arange(0, simulation_time, params.dt)
    angles_history = []
    positions_history = []

    for t in times:
        angles = cpg.update(params.dt)
        angles_history.append(angles)

        # 如果需要，也可以记录位置信息
        if hasattr(cpg, 'get_positions'):
            positions = cpg.get_positions()
            positions_history.append(positions)

    return {
        'times': times,
        'angles': np.array(angles_history),
        'positions': np.array(positions_history) if positions_history else None
    }

class HopfOscillator:
    """基于理论公式的Hopf振荡器实现"""
    def __init__(self, init_phase: float = 0.0):
        self.u = np.cos(init_phase)  # 初始状态 u
        self.v = np.sin(init_phase)  # 初始状态 v
        self.phase = init_phase      # 初始相位
        self.r = np.sqrt(self.u**2 + self.v**2)  # 添加振幅追踪

    def update(self, params: CPGParams, dt: float, coupling_sum: float = 0) -> float:
        """更新振荡器状态"""
        f = params.omega / (2 * np.pi)  # 频率
        mu = params.amplitude  # 修改这里
        sigma = getattr(params, 'convergence_rate', 1.0)  # 收敛速率 (默认值 1.0)

        # 计算状态导数
        r_squared = self.u**2 + self.v**2
        self.r = np.sqrt(r_squared)  # 追踪振幅
        du = -2 * np.pi * f * self.v + sigma * (mu - r_squared) * self.u + coupling_sum
        dv = 2 * np.pi * f * self.u + sigma * (mu - r_squared) * self.v

        # 使用 RK4 积分
        k1u, k1v = du, dv
        u_temp = self.u + dt * k1u / 2
        v_temp = self.v + dt * k1v / 2
        r_squared = u_temp**2 + v_temp**2
        k2u = -2 * np.pi * f * v_temp + sigma * (mu - r_squared) * u_temp + coupling_sum
        k2v = 2 * np.pi * f * u_temp + sigma * (mu - r_squared) * v_temp

        u_temp = self.u + dt * k2u / 2
        v_temp = self.v + dt * k2v / 2
        r_squared = u_temp**2 + v_temp**2
        k3u = -2 * np.pi * f * v_temp + sigma * (mu - r_squared) * u_temp + coupling_sum
        k3v = 2 * np.pi * f * u_temp + sigma * (mu - r_squared) * v_temp

        u_temp = self.u + dt * k3u
        v_temp = self.v + dt * k3v
        r_squared = u_temp**2 + v_temp**2
        k4u = -2 * np.pi * f * v_temp + sigma * (mu - r_squared) * u_temp + coupling_sum
        k4v = 2 * np.pi * f * u_temp + sigma * (mu - r_squared) * v_temp

        # 更新状态
        self.u += dt * (k1u + 2*k2u + 2*k3u + k4u) / 6
        self.v += dt * (k1v + 2*k2v + 2*k3v + k4v) / 6

        # 更新相位
        self.phase = np.arctan2(self.v, self.u)
        return self.phase


class CPGNetwork:
    """基于理论公式的CPG网络"""
    def __init__(self, params: CPGParams, use_rotation_matrix: bool = True):
        self.params = params
        self.n_oscillators = params.n_segments - 1
        self.use_rotation_matrix = use_rotation_matrix  # 决定使用哪种方法
        self.t = 0.0

        # 初始化振荡器
        self.oscillators = [HopfOscillator(-i * params.phase_bias) for i in range(self.n_oscillators)]

        # 初始化耦合矩阵
        lambda_ = params.coupling_weight  # 耦合强度
        self.coupling_matrix = np.zeros((self.n_oscillators, self.n_oscillators))
        for i in range(self.n_oscillators):
            if i > 0:
                self.coupling_matrix[i][i-1] = lambda_
            if i < self.n_oscillators - 1:
                self.coupling_matrix[i][i+1] = lambda_

    def update(self, dt: float) -> np.ndarray:
        """更新网络状态"""
        joint_angles = np.zeros(self.n_oscillators)
        phases = [osc.phase for osc in self.oscillators]

        # 更新振荡器
        for i, osc in enumerate(self.oscillators):
            coupling_sum = 0
            for j in range(self.n_oscillators):
                if self.coupling_matrix[i][j] != 0:
                    phase_diff = phases[j] - phases[i]
                    if i < j:
                        phase_diff += self.params.phase_bias
                    else:
                        phase_diff -= self.params.phase_bias
                    coupling_sum += self.coupling_matrix[i][j] * np.sin(phase_diff)

            # 更新状态并添加转向偏置
            base_angle = self.params.amplitude * np.sin(
                osc.update(params=self.params, dt=dt, coupling_sum=coupling_sum)
            )

            # 添加转向效果
            turning_effect = self.params.turning_bias * \
                           np.exp(-i / (self.n_oscillators/2))  # 转向效果随节段衰减
            offset = self.params.turning_offset * \
                    (1 - i/(self.n_oscillators-1))  # 线性衰减的偏移

            joint_angles[i] = base_angle + turning_effect + offset

        return np.clip(joint_angles, self.params.min_angle, self.params.max_angle)

class SnakeRobot:
    """蛇形机器人"""
    def __init__(self, params: CPGParams):
        self.params = params
        self.cpg = CPGNetwork(params)
        self.reset_state()
        # 添加摩擦系数
        self.mu_t = 0.3  # 切向摩擦系数
        self.mu_n = 1.0  # 法向摩擦系数（大于切向摩擦系数）

    def reset_state(self):
        """初始化状态"""
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.t = 0.0
        # 添加速度状态
        self.vx = 0.0
        self.vy = 0.0
        self.prev_positions = None

    def calculate_positions(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """基于Serpenoid曲线的运动学计算"""
        x_pos = np.zeros(self.params.n_segments)
        y_pos = np.zeros(self.params.n_segments)

        # 计算前进方向的单位向量
        direction = np.array([np.cos(self.theta), np.sin(self.theta)])

        # 计算各节段位置
        for i in range(self.params.n_segments):
            if i == 0:
                # 头部位置更新（考虑前进速度）
                self.x += self.params.v_f * np.cos(self.theta) * self.params.dt
                self.y += self.params.v_f * np.sin(self.theta) * self.params.dt
                x_pos[i] = self.x
                y_pos[i] = self.y
            else:
                # 计算相对角度
                relative_angle = joint_angles[i-1] if i-1 < len(joint_angles) else 0
                # 累积角度
                total_angle = self.theta + relative_angle
                # 计算节段位置
                x_pos[i] = x_pos[i-1] - self.params.segment_length * np.cos(np.deg2rad(total_angle))
                y_pos[i] = y_pos[i-1] - self.params.segment_length * np.sin(np.deg2rad(total_angle))

        return x_pos, y_pos

    def calculate_friction_forces(self, joint_angles: np.ndarray, x_pos: np.ndarray, y_pos: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """计算各向异性摩擦力"""
        fx = np.zeros(self.params.n_segments)
        fy = np.zeros(self.params.n_segments)

        for i in range(self.params.n_segments):
            # 计算节段的方向角
            if i < self.params.n_segments - 1:
                dx = x_pos[i+1] - x_pos[i]
                dy = y_pos[i+1] - y_pos[i]
                segment_angle = np.arctan2(dy, dx)
            else:
                segment_angle = segment_angle  # 最后一个节段使用前一个的角度

            # 计算切向和法向单位向量
            tangent = np.array([np.cos(segment_angle), np.sin(segment_angle)])
            normal = np.array([-np.sin(segment_angle), np.cos(segment_angle)])

            # 计算节段速度
            if self.prev_positions is not None:
                vx = (x_pos[i] - self.prev_positions[0][i]) / self.params.dt
                vy = (y_pos[i] - self.prev_positions[1][i]) / self.params.dt
                velocity = np.array([vx, vy])

                # 分解速度到切向和法向
                v_t = np.dot(velocity, tangent)
                v_n = np.dot(velocity, normal)

                # 计算摩擦力
                f_t = -self.mu_t * np.sign(v_t) * abs(v_t) * tangent
                f_n = -self.mu_n * np.sign(v_n) * abs(v_n) * normal

                # 合成摩擦力
                fx[i] = f_t[0] + f_n[0]
                fy[i] = f_t[1] + f_n[1]

        return fx, fy

    def update_state(self, dt: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """更新机器人状态，包含动力学计算"""
        self.t += dt

        # 获取CPG输出的关节角度
        joint_angles = self.cpg.update(dt)

        # 计算节段位置
        x_pos, y_pos = self.calculate_positions(joint_angles)

        # 计算摩擦力
        fx, fy = self.calculate_friction_forces(joint_angles, x_pos, y_pos)

        # 更新速度（简化的动力学模型）
        mass = 1.0  # 假设单位质量
        self.vx += np.sum(fx) / mass * dt
        self.vy += np.sum(fy) / mass * dt

        # 更新位置
        self.x += self.vx * dt
        self.y += self.vy * dt

        # 更新头部方向
        if np.abs(self.vx) > 1e-6 or np.abs(self.vy) > 1e-6:
            self.theta = np.arctan2(self.vy, self.vx)

        # 保存当前位置用于下次计算
        self.prev_positions = (x_pos.copy(), y_pos.copy())

        return x_pos, y_pos, joint_angles



class SnakeVisualizer:
    """可视化器"""
    def __init__(self, params: CPGParams):
        self.params = params
        self.robot = SnakeRobot(params)

    def animate(self, t_span: np.ndarray, interval=1):
        """动画显示"""
        # 创建主图和网格
        fig = plt.figure(figsize=(20, 12))

        # 创建三列布局：左侧CPG信号，中间运动显示，右侧关节角度
        gs = fig.add_gridspec(1, 3, width_ratios=[1, 2, 1])

        # 左侧CPG信号显示
        gs_left = gs[0].subgridspec(self.params.n_segments-1, 1)
        # 中间运动显示
        gs_middle = gs[1].subgridspec(4, 1)
        # 右侧关节角度显示
        gs_right = gs[2].subgridspec(self.params.n_segments-1, 1)

        # 创建CPG信号子图（左侧）
        ax_cpgs = []
        cpg_lines = []
        cpg_data = [[] for _ in range(self.params.n_segments - 1)]
        time_data = []

        # 创建关节角度子图（右侧）
        ax_angles = []
        angle_lines = []
        angle_data = [[] for _ in range(self.params.n_segments - 1)]

        # 初始化左侧CPG信号显示
        for i in range(self.params.n_segments - 1):
            # CPG信号子图
            ax = fig.add_subplot(gs_left[i, 0])
            ax.set_title(f'CPG {i+1}')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('CPG (deg)')
            ax.grid(True)
            ax.set_ylim(-45, 45)
            ax.set_xlim(0, 5.0)  # 设置固定的时间范围

            line, = ax.plot([], [], 'b-', label=f'CPG {i+1}')
            cpg_lines.append(line)
            ax_cpgs.append(ax)

            # 关节角度图
            ax_angle = fig.add_subplot(gs_right[i, 0])
            ax_angle.set_title(f'Joint {i+1} Angle')
            ax_angle.set_xlabel('Time (s)')
            ax_angle.set_ylabel('Angle (deg)')
            ax_angle.grid(True)
            ax_angle.set_ylim(-45, 45)
            ax_angle.set_xlim(0, 5.0)

            angle_line, = ax_angle.plot([], [], 'r-', label=f'Joint {i+1}')
            angle_lines.append(angle_line)
            ax_angles.append(ax_angle)

        # 主运动显示区域
        ax_main = fig.add_subplot(gs_middle[:-1, 0])

        # 信息显示区域
        ax_info = fig.add_subplot(gs_middle[-1, 0])

        # 设置主显示范围，确保蛇在中
        env_size = 15.0
        ax_main.set_xlim(-env_size/2, env_size/2)  # 修改显示范围
        ax_main.set_ylim(-env_size/2, env_size/2)

        # 初始化信息显示
        info_text = ax_info.text(0.05, 0.95, '',
                               transform=ax_info.transAxes,
                               verticalalignment='top',
                               fontsize=10)
        ax_info.set_title('Motion Info')
        ax_info.axis('off')

        # 初始化蛇的身体
        segments = []
        for i in range(self.params.n_segments):
            segment = patches.Rectangle(
                (0, 0),
                self.params.segment_length,
                self.params.segment_length*0.3,  # 增加宽度
                color='blue',
                alpha=0.8,
                ec='black'  # 添加框
            )
            segments.append(segment)
            ax_main.add_patch(segment)

        # 更明显的头部标记
        head = ax_main.add_patch(
            patches.Circle((0, 0),
                         self.params.segment_length*0.2,
                         color='red',
                         ec='black'
            )
        )

        # 使用尾部迹
        trail, = ax_main.plot([], [], 'g--', alpha=0.5)
        trail_x, trail_y = [], []

        def init():
            self.robot.reset_state()
            for segment in segments:
                segment.set_xy((0, 0))
            for line in cpg_lines:
                line.set_data([], [])
            for line in angle_lines:
                line.set_data([], [])
            trail.set_data([], [])
            info_text.set_text('')
            return segments + [head, trail] + cpg_lines + angle_lines

        dt = t_span[1] - t_span[0]
        start_recording = False  # 记是否开始记录轨迹

        def update(frame):
            nonlocal start_recording
            t = frame * dt

            # 更新机器人状态
            x_pos, y_pos, joint_angles = self.robot.update_state(dt)

            # 更新时间数据
            time_data.append(t)

            # 更新CPG和角度数据
            for i, angle in enumerate(joint_angles):
                cpg_data[i].append(angle)
                angle_data[i].append(angle)

            # 持定时间窗口
            window_size = int(2.0 / dt)  # 减小显示窗口到2秒
            if len(time_data) > window_size:
                time_data.pop(0)
                for data in cpg_data:
                    data.pop(0)
                for data in angle_data:
                    data.pop(0)

            # 更新CPG信号显示
            for i, line in enumerate(cpg_lines):
                line.set_data(time_data, cpg_data[i])
                if len(time_data) > 1:  # 防止时间数据为空
                    ax_cpgs[i].set_xlim(time_data[0], time_data[-1])

            # 更新关节角度显示
            for i, line in enumerate(angle_lines):
                line.set_data(time_data, angle_data[i])
                if len(time_data) > 1:
                    ax_angles[i].set_xlim(time_data[0], time_data[-1])

            # 更新信息显示
            info_text.set_text(
                f'Time: {t:.1f} s\n'
                f'Forward Speed: {self.params.v_f:.2f} m/s\n'  # 使用参数中的速度
                f'Mean Angle: {np.mean(joint_angles):.1f}°\n'
                f'Max Angle: {np.max(np.abs(joint_angles)):.1f}°\n'
                f'Wave Length: {2*np.pi/self.params.phase_bias:.1f} segments\n'
                f'Frequency: {self.params.omega/(2*np.pi):.1f} Hz'
            )

            # 更新蛇的身体段
            for i, segment in enumerate(segments):
                if i < len(x_pos)-1:
                    dx = x_pos[i+1] - x_pos[i]
                    dy = y_pos[i+1] - y_pos[i]
                    angle = np.arctan2(dy, dx)

                    segment.set_xy((x_pos[i], y_pos[i] - self.params.segment_length*0.1))
                    segment.set_angle(np.rad2deg(angle))

            # 更新头部位置
            head.center = (x_pos[0], y_pos[0])

            # 延迟开始记录轨迹，等待运动稳定
            if t > 1.0:  # 1秒后开始记
                start_recording = True

            # 更新尾部轨迹
            if start_recording:
                trail_x.append(x_pos[-1])
                trail_y.append(y_pos[-1])
                if len(trail_x) > 500:
                    trail_x.pop(0)
                    trail_y.pop(0)
            trail.set_data(trail_x, trail_y)

            return segments + [head, trail] + cpg_lines + angle_lines

        ax_main.set_aspect('equal')
        ax_main.grid(True)
        ax_main.set_title('Snake Robot Motion')

        anim = FuncAnimation(
            fig, update, init_func=init,
            frames=len(t_span), interval=interval,
            blit=True, repeat=True
        )

        plt.tight_layout()
        plt.show()
        return anim

class GAOptimizer:
    """遗法优化器"""
    def __init__(self, params: CPGParams):
        self.base_params = params
        self.setup_ga()
        self.best_fitness_history = []
        self.convergence_window = 10     # 降低到10代
        self.convergence_threshold = 1e-3  # 提高到1e-3
        self.max_no_improve = 15         # 15代没改进就停

    def setup_ga(self):
        """设置遗传算法"""
        # 适应度类和个
        creator.create("FitnessMax", base.Fitness, weights=(1.0,))
        creator.create("Individual", list, fitness=creator.FitnessMax)

        self.toolbox = base.Toolbox()

        # 定义基因
        self.toolbox.register("mu", random.uniform, 0.5, 2.0)
        self.toolbox.register("omega", random.uniform, 0.5, 3.0)
        self.toolbox.register("phase_bias", random.uniform, np.pi/6, np.pi/2)
        self.toolbox.register("coupling_weight", random.uniform, 0.5, 3.0)

        # 创建个体和种群
        self.toolbox.register("individual", tools.initCycle, creator.Individual,
                            (self.toolbox.mu, self.toolbox.omega,
                             self.toolbox.phase_bias, self.toolbox.coupling_weight), n=1)
        self.toolbox.register("population", tools.initRepeat, list, self.toolbox.individual)

        # 注册遗传操作
        self.toolbox.register("evaluate", self.evaluate_individual)
        self.toolbox.register("mate", tools.cxTwoPoint)
        self.toolbox.register("mutate", self.mutate_individual, indpb=0.2)
        self.toolbox.register("select", tools.selTournament, tournsize=3)

    def mutate_individual(self, individual, indpb):
        """变异操作"""
        for i in range(len(individual)):
            if random.random() < indpb:
                if i == 0:  # mu
                    individual[i] = random.uniform(0.5, 2.0)
                elif i == 1:  # omega
                    individual[i] = random.uniform(0.5, 3.0)
                elif i == 2:  # phase_bias
                    individual[i] = random.uniform(np.pi/6, np.pi/2)
                else:  # coupling_weight
                    individual[i] = random.uniform(0.5, 3.0)
        return individual,

    def evaluate_individual(self, individual):
        """改进的适应度评估"""
        test_params = CPGParams(
            n_segments=self.base_params.n_segments,
            segment_length=self.base_params.segment_length,
            mu=individual[0],
            omega=individual[1],
            phase_bias=individual[2],
            coupling_weight=individual[3],
            v_f=self.base_params.v_f
        )

        robot = SnakeRobot(test_params)
        dt = 0.01
        steps = 300

        # 记录数据
        x_positions = []
        y_positions = []
        joint_angles_history = []
        time_points = []  # 添加时间记录
        current_time = 0.0  # 添加时间追踪

        # 运行仿真
        for step in range(steps):
            current_time = step * dt  # 更新当前时间
            time_points.append(current_time)
            x_pos, y_pos, angles = robot.update_state(dt)
            x_positions.append(x_pos[0])
            y_positions.append(y_pos[0])
            joint_angles_history.append(angles)

        # 1. 评估前进距离（正向奖励）
        forward_distance = x_positions[-1] - x_positions[0]
        distance_reward = max(0, forward_distance * 10)  # 放大前进距离的奖励

        # 2. 评估波形质量
        wave_quality = 0
        for angles in joint_angles_history:
            # 检查波形振幅
            amplitude = np.max(np.abs(angles))
            if 25 <= amplitude <= 35:  # 理想振幅范围
                wave_quality += 1

            # 检查相位差
            phase_diffs = np.diff(angles)
            if np.all(np.abs(phase_diffs) <= test_params.phase_bias * 1.2):
                wave_quality += 1

        wave_quality = wave_quality / (2 * len(joint_angles_history))  # 归一化

        # 3. 评估路径直线性（负向惩罚）
        path_deviation = np.std(y_positions)
        straightness_penalty = max(0, path_deviation * 2)

        # 4. 评估运动平滑
        smoothness = np.mean([
            np.mean(np.abs(np.diff(angles)))
            for angles in joint_angles_history
        ])
        smoothness_reward = 1 / (1 + smoothness)  # 平滑度奖励

        # 评估跟踪效果
        tracking_error = 0
        for t_idx, angles in enumerate(joint_angles_history):
            # 使用正确的时间点计算理想角度
            t = time_points[t_idx]
            ideal_angles = [
                30 * np.sin(test_params.omega * t - i * test_params.phase_bias)
                for i in range(len(angles))
            ]
            # 计算跟踪误差
            error = np.mean(np.abs(angles - ideal_angles))
            tracking_error += error

        tracking_error /= len(joint_angles_history)

        # 修改应度计算
        fitness = (
            50 * distance_reward +
            50 / (1 + tracking_error) +
            20 * wave_quality +
            10 * smoothness_reward -
            10 * straightness_penalty
        )

        return (max(0.1, fitness),)

    def optimize(self, max_generations=100, population_size=50):  # 降低最大代数
        """改进的优化过程"""
        self.max_generations = max_generations
        pop = self.toolbox.population(n=population_size)
        hof = tools.HallOfFame(1)

        # 评估始种群
        fitnesses = list(map(self.toolbox.evaluate, pop))
        for ind, fit in zip(pop, fitnesses):
            ind.fitness.values = fit

        # 记录最佳个体
        best_fitness = float('-inf')
        generations_no_improve = 0

        # 开始进化
        for gen in range(max_generations):
            # 选择
            offspring = self.toolbox.select(pop, len(pop))
            offspring = list(map(self.toolbox.clone, offspring))

            # 交叉（增加概率）
            for child1, child2 in zip(offspring[::2], offspring[1::2]):
                if random.random() < 0.9:  # 提高交叉概率
                    self.toolbox.mate(child1, child2)
                    del child1.fitness.values
                    del child2.fitness.values

            # 变异（增加概率和强度）
            for mutant in offspring:
                if random.random() < 0.4:  # 提高变异概率
                    self.toolbox.mutate(mutant, indpb=0.3)  # 提高变异强度
                    del mutant.fitness.values

            # 评估新个体
            invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
            fitnesses = list(map(self.toolbox.evaluate, invalid_ind))
            for ind, fit in zip(invalid_ind, fitnesses):
                ind.fitness.values = fit

            # 更新种群
            pop[:] = offspring

            # 更新最佳个体
            current_best = max([ind.fitness.values[0] for ind in pop])
            self.best_fitness_history.append(current_best)

            if current_best > best_fitness:
                best_fitness = current_best
                generations_no_improve = 0
            else:
                generations_no_improve += 1

            # 打印进度
            print(f"Generation {gen+1}: Best Fitness = {best_fitness:.6f}")

            # 检查终止条件
            if generations_no_improve >= self.max_no_improve:  # 15代没改进
                print(f"\n连续{self.max_no_improve}代有改进，停止优化")
                break

            if generations_no_improve >= self.convergence_window:
                recent_fitness = self.best_fitness_history[-self.convergence_window:]
                fitness_std = np.std(recent_fitness)
                if fitness_std < self.convergence_threshold:
                    print(f"\n算法已收敛，应度标准差: {fitness_std:.6f}")
                    break

            # 更新名堂
            hof.update(pop)

        # 输出最终结果
        print("\n优化完成:")
        print(f"总代数: {gen+1}")
        print(f"最终适应度: {best_fitness:.6f}")
        print(f"收敛过程:")
        for i, fit in enumerate(self.best_fitness_history):
            if i % 5 == 0:  # 每5代输出一次
                print(f"Generation {i+1}: {fit:.6f}")

        return hof[0], self.best_fitness_history

def plot_joint_trajectories(t_span: np.ndarray, params: CPGParams, ax=None):
    """绘制关节轨迹，添加参数显示"""
    robot = SnakeRobot(params)
    dt = t_span[1] - t_span[0]

    # 记录数据
    time_data = []
    joint_angles = [[] for _ in range(params.n_segments - 1)]

    # 运行仿真
    for t in t_span:
        time_data.append(t)
        x_pos, y_pos, angles = robot.update_state(dt)
        for i, angle in enumerate(angles):
            joint_angles[i].append(angle)

    # 创建图形
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 8))

    # 绘制每个关节的轨迹
    for i in range(len(joint_angles)):
        ax.plot(time_data, joint_angles[i], label=f'Joint {i+1}')

    # 添加参数信息
    param_text = (
        f"Frequency: {params.omega/(2*np.pi):.1f} Hz\n"
        f"Phase Bias: {params.phase_bias*180/np.pi:.1f}°\n"
        f"Amplitude: {params.amplitude:.1f}°\n"
        f"Coupling: {params.coupling_weight:.1f}"
    )
    ax.text(0.02, 0.98, param_text,
            transform=ax.transAxes,
            verticalalignment='top',
            bbox=dict(facecolor='white', alpha=0.8))

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Joint Angle (deg)')
    ax.grid(True)
    ax.legend()

    return ax

def plot_cpg_comparison(gaits: dict, simulation_time: float = 5.0):
    """比较不同步态的CPG信号
    Args:
        gaits: 包含不同步态参数的字典
        simulation_time: 仿真时间(秒)
    """
    n_gaits = len(gaits)
    fig = plt.figure(figsize=(15, 4*n_gaits))
    gs = gridspec.GridSpec(n_gaits, 1)

    for idx, (gait_name, params) in enumerate(gaits.items()):
        # 创建CPG网络
        cpg = CPGNetwork(params)

        # 仿真并收集数据
        times = np.arange(0, simulation_time, params.dt)
        joint_angles = []

        for t in times:
            angles = cpg.update(params.dt)
            joint_angles.append(angles)

        joint_angles = np.array(joint_angles)

        # 创建子图
        ax = fig.add_subplot(gs[idx])

        # 绘制每个关节的CPG信号
        for j in range(params.n_segments - 1):
            ax.plot(times, joint_angles[:, j],
                   label=f'Joint {j+1}',
                   linewidth=2)

        # 设置图形
        ax.set_title(f'{gait_name}\n'
                    f'Frequency: {params.frequency:.1f}Hz, '
                    f'Amplitude: {params.amplitude:.1f}°, '
                    f'Phase Bias: {params.phase_bias*180/np.pi:.1f}°, '
                    f'Coupling: {params.coupling_weight:.1f}',
                    pad=10)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Joint Angle (deg)')
        ax.grid(True, alpha=0.3)
        ax.legend(ncol=params.n_segments-1, loc='upper right')

        # 添加参数信息
        param_text = (
            f'v_f: {params.v_f:.1f} m/s\n'
            f'σ: {params.convergence_rate:.1f}\n'
            f'μt: {params.mu_t:.1f}, μn: {params.mu_n:.1f}'
        )
        ax.text(0.02, 0.98, param_text,
                transform=ax.transAxes,
                verticalalignment='top',
                bbox=dict(facecolor='white', alpha=0.8))

    plt.tight_layout()
    return fig

def create_motion_patterns():
    """创建不同的运动模式"""
    patterns = {
        "Forward": CPGParams(
            frequency=1.0,
            amplitude=30.0,
            phase_bias=np.pi/2,  # 90度相位差产生向前运动
            coupling_weight=1.0,
            turning_bias=0.0
        ),
        "Turn_Left": CPGParams(
            frequency=1.0,
            amplitude=30.0,
            phase_bias=np.pi/2,
            coupling_weight=1.0,
            turning_bias=15.0,    # 左转偏置
            turning_offset=-10.0
        ),
        "Turn_Right": CPGParams(
            frequency=1.0,
            amplitude=30.0,
            phase_bias=np.pi/2,
            coupling_weight=1.0,
            turning_bias=-15.0,   # 右转偏置
            turning_offset=10.0
        ),
        "S_Shape": CPGParams(
            frequency=1.0,
            amplitude=35.0,
            phase_bias=2*np.pi/3,  # 120度相位差产生S形
            coupling_weight=1.2
        )
    }
    return patterns

def create_circular_motion_params():
    """创建更明显的圆形运动参数"""
    return CPGParams(
        n_segments=8,
        segment_length=0.2,
        frequency=2.0,          # 保持较快频率
        amplitude=35.0,         # 增大振幅
        phase_bias=np.pi/2,
        coupling_weight=1.2,
        convergence_rate=1.0,
        mu_t=0.3,
        mu_n=1.0,
        control_gain=15.0,
        v_f=0.5,               # 保持较快速度
        max_angle=45,          # 增大最大角度
        min_angle=-45,
        dt=0.01,
        turning_bias=40.0,     # 显著增大转向偏置
        turning_offset=25.0    # 增大转向补偿
    )

def main():
    # 创建圆形运动参数
    # params = create_circular_motion_params()
    params = create_motion_patterns()

    # 首先显示CPG信号
    print("显示CPG信号...")
    fig_cpg = plot_cpg_comparison({"Circular Motion": params})
    plt.show()

    # 等待用户确认
    input("\n按Enter键继续运动仿真...")

    # 创建可视化器
    visualizer = SnakeVisualizer(params)

    # 设置仿真时间
    t_span = np.linspace(0, 15, 1500)  # 15秒，1500帧

    # 运行动画
    anim = visualizer.animate(t_span, interval=1)
    plt.show()

if __name__ == "__main__":
    main()
