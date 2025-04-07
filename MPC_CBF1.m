function [] = MPC_CBF1()
%% MPC-CBF算法实现
clear all; clc;
figure;
set(gcf,'position',[500,200,500,400]); 
disp('MPC-CBF Navigation Start')

%% 初始化参数
global dt obstacle obstacleR goal;
dt = 0.2;  % 时间步长
obstacle = [3.5 7.3;
           12 7.3;
           0 2.6;
           8.5 2.6];
obstacleR = 1.7;  % 障碍物半径
goal = [0.870, 9.65];   % 目标点
area = [-1.8 16.305 -0.1 11.405]; % 地图范围

%% 机器人初始状态 [x, y, theta, v, w]
x = [3, 0, pi/2, 0, 0]'; 
result.x = []; % 轨迹存储

%% MPC参数
N = 5;   % 预测时域
Q = diag([10, 10, 0.1]); % 状态权重
R = diag([0.01, 0.01]);  % 控制量权重
u_min = [-0.5, -pi/2]'; % 控制量下限
u_max = [1.5, pi/2]';   % 控制量上限

%% 主循环
for i = 1:5000
    % 求解MPC优化问题
    u = solve_MPC_CBF(x, N, Q, R, u_min, u_max);
    
    % 状态更新
    x = robot_model(x, u);
    result.x = [result.x; x'];
    
    % 终止条件
%     if norm(x(1:2) - goal') < 0.2
%         disp('Arrived Goal!'); break;
%     end

    if norm(x(1:2) - goal') < 0.1
    break;
    end
    
    % 绘图
%     plot_navigation(x, result, goal, obstacle, X_pred);
    plot_navigation(x, result, goal, obstacle);
end
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
        
        % CBF约束（避障）
        for obs = 1:size(obstacle,1)
        dx = X(1,k+1) - obstacle(obs,1);
        dy = X(2,k+1) - obstacle(obs,2);
        vx = X(4,k+1) * cos(X(3,k+1));
        vy = X(4,k+1) * sin(X(3,k+1));   

        % 安全距离公式（需包含速度影响）
        h = (dx)^2 + (dy)^2 - (obstacleR + 0.2)^2;  % 增加0.2m安全余量
        h_dot = 2*(dx*vx + dy*vy);
        constraints = [constraints, h_dot + 1.0*h >= 0];  % 类K函数参数设为2.0
        
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
%     X_pred = value(X(1:2,2:end)); 
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
% function x_next = robot_model(x, u)
%     global dt;
%     % 差速驱动模型
%     v = x(4) + u(1)*dt;
%     w = x(5) + u(2)*dt;
%     theta = x(3) + w*dt;
%     x_next = [
%         x(1) + v*cos(theta)*dt;
%         x(2) + v*sin(theta)*dt;
%         theta;
%         v;
%         w
%     ];
% end


function plot_navigation(x, result, goal, obstacle)
% global obstacleR;
    hold off;
    quiver(x(1), x(2), 0.5*cos(x(3)), 0.5*sin(x(3)), 'ok'); 
    hold on;
    show_map2;
    
    plot(result.x(:,1), result.x(:,2), '-r','LineWidth',2); hold on;
    % 在plot_navigation函数中添加：
    plot(goal(1), goal(2), '*g'); 
    start = [3, 0]; 
    plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');hold on;
    text(start(1)+0.6, start(2), 'Start', 'Color','k','FontSize',10);hold on;

%     if ~isempty(X_pred)
%         plot(X_pred(1,:), X_pred(2,:), '--b', 'LineWidth',1.5);
%     end
%     for i = 1:size(obstacle,1)
%         circle(obstacle(i,:), obstacleR);
%     end
    axis([-3 15 -1 12]); grid on;
    title('A*-MPC-CBF');
    drawnow;
end
