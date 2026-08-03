% 蛇形机器人运动参数仿真
clear;
clc;
close all;

% 系统参数 - 匹配STM32代码
RATE_20_HZ = 100;              % 任务频率100Hz
controlStep = RATE_20_HZ;      % 直接使用RATE_20_HZ作为controlStep
frequency = 0.25;              % 运动频率0.25Hz
swingAngleLimit = 30;          % 摆动角度限制30度
simTime = 5;                   % 仿真5秒
NUM_SNAKE_JOINTS = 4;          % 4个关节
baseOffset = [0, 0, 0, 0];     % 基准角度

% 计算关键参数
dt = controlStep/1000;        % 转换为秒
phase_change_per_step = dt * frequency * 2 * pi;  % 每步相位变化
phase_change_degrees = phase_change_per_step * 180/pi;  % 转换为角度

% 打印关键参数分析
fprintf('===== 相位分析 =====\n');
fprintf('1. 每步更新:\n');
fprintf('   - 时间间隔: %.3f ms\n', dt*1000);
fprintf('   - 相位变化: %.3f度\n', phase_change_degrees);
fprintf('2. 每秒更新:\n');
fprintf('   - 更新次数: %d次\n', RATE_20_HZ);
fprintf('   - 总相位变化: %.1f度/秒\n', phase_change_degrees * RATE_20_HZ);
fprintf('3. 完整周期:\n');
fprintf('   - 周期时间: %.2f秒\n', 1/frequency);
fprintf('   - 总步数: %d步\n', round(RATE_20_HZ/frequency));

% 时间序列
t = 0:dt:simTime;
numSteps = length(t);

% 相位和角度数组初始化
phase = zeros(1, numSteps);
angles = zeros(NUM_SNAKE_JOINTS, numSteps);

% 计算每个时间步的相位和角度
for i = 2:numSteps
    % 相位更新 - 匹配STM32代码
    phase(i) = phase(i-1) - (controlStep/1000.0) * frequency * 2 * pi;

    % 计算每个关节的角度
    for joint = 1:NUM_SNAKE_JOINTS
        % 计算相位差
        phase_offset = (NUM_SNAKE_JOINTS - joint) * (pi/3);
        % 计算目标角度
        angles(joint,i) = baseOffset(joint) + ...
            swingAngleLimit * sin(phase(i) + phase_offset);
    end
end

% 绘图
figure('Position', [100, 100, 1200, 800]);

% 1. 所有关节角度随时间变化
subplot(2,2,1);
for joint = 1:NUM_SNAKE_JOINTS
    plot(t, angles(joint,:), 'LineWidth', 2);
    hold on;
end
grid on;
title('关节角度随时间变化');
xlabel('时间 (s)');
ylabel('角度 (度)');
legend('关节1', '关节2', '关节3', '关节4');

% 2. 相位变化
subplot(2,2,2);
plot(t, mod(phase, 2*pi), 'LineWidth', 2);
grid on;
title('相位随时间变化');
xlabel('时间 (s)');
ylabel('相位 (rad)');

% 3. 关节运动轨迹快照
subplot(2,2,3);
for i = 1:100:numSteps
    x = 1:NUM_SNAKE_JOINTS;
    plot(x, angles(:,i), 'b.-', 'MarkerSize', 20);
    hold on;
end
grid on;
title('关节运动轨迹快照');
xlabel('关节编号');
ylabel('角度 (度)');
axis([0.5 4.5 -60 60]);

% 4. 运动参数信息
subplot(2,2,4);
text(0.1, 0.9, sprintf('任务频率: %d Hz', RATE_20_HZ));
text(0.1, 0.8, sprintf('控制步长: %.3f ms', dt*1000));
text(0.1, 0.7, sprintf('运动频率: %.2f Hz', frequency));
text(0.1, 0.6, sprintf('振幅: %.1f 度', swingAngleLimit));
text(0.1, 0.5, sprintf('相位差: 60 度'));
text(0.1, 0.4, sprintf('周期: %.2f s', 1/frequency));
text(0.1, 0.3, sprintf('每步相位变化: %.3f 度', phase_change_degrees));
axis off;

% 添加角度范围分析
fprintf('\n===== 角度范围分析 =====\n');
for joint = 1:NUM_SNAKE_JOINTS
    angle_range = angles(joint,:);
    fprintf('关节%d:\n', joint);
    fprintf('  最大角度: %.1f°\n', max(angle_range));
    fprintf('  最小角度: %.1f°\n', min(angle_range));
    fprintf('  摆动范围: %.1f°\n', max(angle_range) - min(angle_range));
end
