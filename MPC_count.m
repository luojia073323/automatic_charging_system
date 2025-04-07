function [] = MPC_count()
%% MPC-CBF算法实现
clear all; clc;
figure;
set(gcf,'position',[500,200,500,400]); 

%% 初始化参数
global dt obstacle obstacleR goal;
dt = 0.5;  % 时间步长
obstacle = [3.4 7.3;
           12 7.3;
           0 2.6;
           8.5 2.4];
obstacleR = 2;  % 障碍物半径
goal = [8, 10];   % 目标点
area = [-1.8 16.305 -0.1 11.405]; % 地图范围

%% 初始化评估维度参数
total_distance = 0;       % 总路径长度
max_curvature = 0;        % 最大曲率
min_obstacle_dist = Inf; % 最小障碍距离
planning_time = 0;        % 总规划耗时
num_iterations = 0;       % 迭代次数
end_error = 0;            % 终点偏移误差

%% 机器人初始状态 [x, y, theta, v, w]
x = [6, 0, pi/2, 0, 0]'; 
result.x = []; % 轨迹存储

%% MPC参数
N = 10;   % 预测时域
Q = diag([10, 10, 0.1]); % 状态权重
R = diag([0.01, 0.01]);  % 控制量权重
u_min = [-2.0, -pi/2]'; % 控制量下限
u_max = [1.5, pi/2]';   % 控制量上限

%% 主循环
result.x = x';
for i = 1:2000
    % 求解MPC优化问题
    % 终止条件
    if norm(x(1:2) - goal') < 0.1
         % 4. 终点偏移误差
        end_error = norm(x(1:2)' - goal);
        converged = true; % 标记成功收敛
    break;
    end
    
    % 记录规划开始时间
    tic;
    u = solve_MPC_CBF(x, N, Q, R, u_min, u_max);
     
    % 累计规划时间
    planning_time = planning_time + toc;
    num_iterations = num_iterations + 1;
    
    prev_pos = x(1:2)';
    
    % 状态更新
    x = robot_model(x, u);
    result.x = [result.x; x'];
    
    %% 计算评估维度
    % 1. 路径长度（累加每步移动距离）
    current_pos = x(1:2)';
    total_distance = total_distance + norm(current_pos - prev_pos);
    
    % 2. 最大曲率（曲率 = 角速度/线速度）
    v = x(4);
    w = x(5);
    if abs(v) > 1e-3 % 避免除零
        current_curvature = abs(w / v);
        max_curvature = max(max_curvature, current_curvature);
    end
    % 3. 最小障碍距离（遍历所有障碍物）
    for obs = 1:size(obstacle,1)
        dist = norm(current_pos - obstacle(obs,:)) - obstacleR;
        min_obstacle_dist = min(min_obstacle_dist, dist);
    end
    
    if norm(x(1:2) - goal') < 0.2
         % 4. 终点偏移误差
        end_error = norm(x(1:2)' - goal);
        converged = true; % 标记成功收敛
    break;
    end
    
    % 绘图
%     plot_navigation(x, result, goal, obstacle, X_pred);
    plot_navigation(x, result, goal, obstacle);
end

%% 打印评估结果
disp('============== 评估维度 ==============');
disp(['路径长度:        ', num2str(total_distance), ' 米']);
disp(['最大曲率:        ', num2str(max_curvature), ' rad/m']);
disp(['最小障碍距离:    ', num2str(min_obstacle_dist), ' 米']);
disp(['单次规划耗时:    ', num2str(planning_time/num_iterations), ' 秒/次']);
disp(['迭代收敛次数:    ', num2str(num_iterations), ' 次']); % 新增行
disp(['终点偏移误差:    ', num2str(end_error), ' 米']);
disp('======================================');
end

function u_opt= solve_MPC_CBF(x0, N, Q, R, u_min, u_max)
    % 定义优化变量（控制序列）
    global dt;
    global goal; 
    global obstacle; 
    global obstacleR;
    U = sdpvar(2, N); 
    
    % 初始状态
    X = x0;
    cost = 0;
    constraints = [];
    
    % 构建预测模型与约束
    for k = 1:N-1
        % 状态预测（运动学模型）
        X_next = robot_model(X(:,k), U(:,k));
        X = [X, X_next];
        
        % 目标函数：跟踪误差 + 控制量惩罚
        error_term = X(1:3,k+1) - [goal(:); 0];
        cost = cost + error_term' * Q * error_term + U(:,k)' * R * U(:,k);
        
        % 添加终端代价
        terminal_error = X(1:2, end) - goal(:);
        cost = cost + terminal_error' * diag([100, 100]) * terminal_error;

        % CBF约束（避障）
        for obs = 1:size(obstacle,1)
        dx = X(1,k+1) - obstacle(obs,1);
        dy = X(2,k+1) - obstacle(obs,2);
        vx = X(4,k+1) * cos(X(3,k+1));
        vy = X(4,k+1) * sin(X(3,k+1));   

        % 安全距离公式（需包含速度影响）
        h = (dx)^2 + (dy)^2 - (obstacleR + 0.2)^2;  % 增加0.2m安全余量
        h_dot = 2*(dx*vx + dy*vy);
        constraints = [constraints, h_dot + 2.0*h >= 0];  % 类K函数参数设为2.0
        
%         h_sq = (X(1,k+1)-obstacle(obs,1))^2 + (X(2,k+1)-obstacle(obs,2))^2 - obstacleR^2;
%         constraints = [constraints, h_sq + 0.5*dt*h_sq >= 0];
        end
    end
    
    % 控制量约束
    constraints = [constraints, repmat(u_min,1,N) <= U <= repmat(u_max,1,N)];
    
    % 求解优化
    options = sdpsettings('solver','ipopt','verbose',0);
    options.ipopt.max_iter = 100; % 正确设置IPOPT的最大迭代次数
    optimize(constraints, cost, options);
    u_opt = value(U(:,1)); % 取第一个控制量

end

function x_next = robot_model(x, u)
    global dt;
    A = [1 0 0 dt*cos(x(3)) 0;
         0 1 0 dt*sin(x(3)) 0;
         0 0 1 0 dt;
         0 0 0 1 0;
         0 0 0 0 1];
    B = [0 0; 0 0; 0 0; dt 0; 0 dt];
    x_next = A*x + B*u;
end

function plot_navigation(x, result, goal, obstacle)
% global obstacleR;
    hold off;
    quiver(x(1), x(2), 0.5*cos(x(3)), 0.5*sin(x(3)), 'ok'); 
    hold on;
    show_map2;
    
    plot(result.x(:,1), result.x(:,2), '-r','LineWidth',2); hold on;
    % 在plot_navigation函数中添加：
    plot(goal(1), goal(2), '*g'); 
    start = [6, 0]; 
    plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');hold on;
    text(start(1)+0.6, start(2), 'Start', 'Color','k','FontSize',10);hold on;

    axis([-3 15 -1 12]); grid on;
    title('A*-MPC-CBF');
    drawnow;
end
