# 🐍 Snake-Robot Simulator

🚀 **一款用于模拟蛇形机器人运动行为的 Python 仿真工具,支持 CPG 网络控制、动力学建模与动画可视化。**
本项目旨在帮助研究者和开发者更好地理解蛇形机器人运动学和控制策略,通过实验探索最佳参数。

![Python](https://img.shields.io/badge/python-3.8%2B-blue)
![NumPy](https://img.shields.io/badge/NumPy-%E2%89%A51.19.0-orange)
![Matplotlib](https://img.shields.io/badge/Matplotlib-%E2%89%A53.3.0-yellowgreen)
![DEAP](https://img.shields.io/badge/DEAP-%E2%89%A51.3.1-brightgreen)
![License](https://img.shields.io/badge/license-MIT-green)
![Build](https://img.shields.io/github/actions/workflow/status/your-username/snake-robot-simulator/python-app.yml?branch=main)
---

## 📖 项目简介

蛇形机器人（Snake Robot）是一种灵活且多功能的仿生机器人,其运动模仿了蛇的波动行为。
本项目实现了:
- 🌊 **CPG 网络控制**: 基于 Hopf 振荡器的相位耦合网络。
- ⚙️ **动力学模拟**: 考虑摩擦力和关节运动限制。
- 🎥 **动画可视化**: 实时展示蛇形机器人的运动状态。
- 🔧 **参数优化**: 通过遗传算法（GA）优化控制参数。

---

## 🔬 原理解析

### 中枢模式发生器（CPG）
- CPG 是一种神经振荡器网络,能够自主产生节律性信号,无需外部输入。
- 每个关节由一个 Hopf 振荡器控制,振荡器之间通过相位耦合连接。
- 通过调节 CPG 参数（如频率、振幅、相位差）,可以生成不同的运动模式。

### 动力学模型
- 蛇形机器人被建模为一系列刚体节段,通过旋转关节连接。
- 考虑了切向和法向摩擦力,以及关节运动限制。
- 使用数值积分方法求解运动方程,更新机器人状态。

### 运动模式
#### 直线运动
- 直线运动是蛇形机器人最基本的运动模式,通过生成向前推进的波形实现。
- 在 CPG 网络中,相邻振荡器之间的相位差设置为90°(π/2)。
- 这种相位差配置会产生一个从头到尾沿蛇身传播的正弦波,推动蛇形机器人向前移动。
- 通过调节振幅和频率参数,可以控制蛇形机器人的前进速度和波形幅度。
- 直线运动的实现逻辑如下:
  1. 初始化 CPG 网络,将相邻振荡器的相位差设置为90°。
  2. 根据 CPG 输出计算每个关节的目标角度。
  3. 使用 PID 控制器或其他控制方法,使关节角度跟踪目标角度。
  4. 不断更新 CPG 状态和关节角度,生成连续的蛇形运动。

#### 圆周运动
- 圆周运动是一种更复杂的运动模式,使蛇形机器人沿着圆形轨迹运动。
- 为了实现圆周运动,需要在 CPG 网络中引入转向偏置和相位差调整。
- 转向偏置用于控制蛇形机器人的转向方向和曲率,通过在 CPG 振荡器中添加一个恒定的偏置项实现。
- 相位差的调整有助于生成更适合圆周运动的蛇形波形,通常需要进行一定的试验和优化。
- 圆周运动的实现逻辑如下:
  1. 初始化 CPG 网络,设置合适的相位差(如120°或60°)。
  2. 引入转向偏置,根据期望的圆周半径和转向方向计算偏置值。
  3. 根据 CPG 输出和转向偏置计算每个关节的目标角度。
  4. 使用 PID 控制器或其他控制方法,使关节角度跟踪目标角度。
  5. 不断更新 CPG 状态、转向偏置和关节角度,生成圆周运动。
  6. 通过调整转向偏置和相位差,优化圆周运动的稳定性和轨迹跟踪性能。

需要注意的是,实现稳定和高效的圆周运动需要对 CPG 参数进行仔细的调试和优化。通过遗传算法等优化方法,可以自动搜索最佳的参数组合,以获得理想的圆周运动效果。

### 参数优化
- 使用遗传算法（GA）优化 CPG 和动力学参数,如振幅、频率、摩擦系数等。
- 优化目标为最大化前进距离,或实现特定的运动模式。
- 通过自然选择和遗传操作（交叉、变异）,搜索最优参数组合。

---

## 🌟 功能亮点
- **动态运动仿真**: 支持自定义节段数、频率、相位差等参数。
- **动画展示**: 实时查看机器人如何根据不同参数产生波形。
- **优化与分析**: 基于 GA 对振幅、频率等参数进行自动调优。
- **直观可视化**: 生成关节角度轨迹和理想波形对比图。

---

## 📦 目录结构

```
├── main.py            # 主程序
├── README.md          # 项目说明文件
├── requirements.txt   # 依赖库列表
└── src/
    ├── cpg.py         # CPG 网络实现
    ├── robot.py       # 蛇形机器人类
    ├── visualizer.py  # 动画与可视化
    └── optimizer.py   # 遗传算法优化
```

---

## 🎮 快速上手

### 1️⃣ 克隆仓库
```bash
git clone https://github.com/your-username/snake-robot-simulator.git
cd snake-robot-simulator
```

### 2️⃣ 安装依赖
确保已安装 Python 3.8 或更高版本,然后运行以下命令安装依赖:
```bash
pip install -r requirements.txt
```

### 3️⃣ 运行主程序
```bash
python main.py
```

运行后,程序将自动生成动画、关节轨迹图,并展示优化结果。

---

## 🛠️ 核心功能

### 1️⃣ 动画展示
🐍 **动态仿真**: 使用 `matplotlib.animation` 实现实时动画。
<img src="https://via.placeholder.com/600x300?text=Snake+Robot+Animation" alt="动画示例" width="60%">

### 2️⃣ 参数优化
🔧 **遗传算法优化**: 利用 `deap` 实现的遗传算法对控制参数进行优化。
- **优化目标**: 前进距离最大化、波形质量提升。
- **支持调整的参数**:
  - 振幅（`amplitude`）
  - 频率（`frequency`）
  - 相位差（`phase_bias`）
  - 耦合强度（`coupling_weight`）

### 3️⃣ 数据可视化
📊 **关节角度轨迹**: 展示实际与理想波形的对比。
<img src="https://via.placeholder.com/600x300?text=Joint+Trajectory+Plot" alt="关节轨迹示例" width="60%">

---

## 🔑 核心代码

### 动画展示
```python
from matplotlib.animation import FuncAnimation

# 初始化可视化器
visualizer = SnakeVisualizer(params)

# 动画运行
t_span_anim = np.linspace(0, 30, 3000)
anim = visualizer.animate(t_span_anim, interval=20)
```

### 遗传算法优化
```python
# 设置遗传算法
optimizer = GAOptimizer(params)
best_params, history = optimizer.optimize(max_generations=200, population_size=100)

print(f"最佳适应度: {best_params.fitness.values[0]:.6f}")
```

---

## 🔮 项目计划
- [ ] 添加更多复杂地形模拟（如阶梯、弯曲路径）。
- [ ] 实现 3D 蛇形机器人仿真。
- [ ] 集成 Reinforcement Learning 进行控制策略学习。

---

## ❤️ 致谢

感谢以下开源项目提供灵感与支持:
- [DEAP](https://github.com/DEAP/deap) - 遗传算法框架
- [Matplotlib](https://matplotlib.org/) - 可视化工具
- [NumPy](https://numpy.org/) - 科学计算库

---

## 📜 许可证

本项目采用 [MIT License](LICENSE) 开源,欢迎贡献与修改。

---

🎉 **Happy Coding!**
如果喜欢这个项目,请点个 ⭐️ Star 支持!
