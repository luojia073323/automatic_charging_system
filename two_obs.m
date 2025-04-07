function [] = two_obs()
%% DWA算法（静态障碍物+动态障碍物）
clear all; clc; close all;
figure
set(gcf,'position',[500,200,500,400]); 

%% 初始化参数
global dt;
dt = 0.2;
x = [1 3.5 0 0 0]'; % 初始状态
goal = [8,2];          % 目标位置

% 静态障碍物 [x, y, 半径]
static_obstacles = [
    2.5, 2.3, 0.6;
    2.5, 5, 0.6;
];

% 动态障碍物 [x, y, vx, vy, 半径]
dynamic_obstacles = [
    4.0, 5.0, 0.2, -0.2, 0.6;  % 障碍物3斜向移动
%     6.0, 1.0, 0.0, 0.25, 0.6;  % 障碍物4垂直向上
];

Kinematic = [1.0, toRadian(20.0), 0.5, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.01, 0.15, 0.1, 3.0];
result.x = [];

%% 主循环
for i = 1:2000  
    % 更新动态障碍物位置
    dynamic_obstacles(:,1:2) = dynamic_obstacles(:,1:2) + dynamic_obstacles(:,3:4)*dt;
    
    % DWA路径规划（传入两种障碍物）
    [u,traj] = DynamicWindowApproach(x, Kinematic, goal, evalParam, static_obstacles, dynamic_obstacles);
    
    x = f(x, u); % 更新状态
    result.x = [result.x; x'];
    
    % 检查是否到达目标
    if norm(x(1:2)-goal') < 0.1
        disp('Arrive Goal!'); break;
    end
    
    % 绘制场景
    clf; hold on;
    show_map;hold on;
    plot(result.x(:,1), result.x(:,2), 'r', 'LineWidth',2); % 轨迹
    plot(goal(1), goal(2), '*r', 'MarkerSize',10);          % 目标点
    text(7.5, 3, 'goal', 'Color','k','FontSize',10);hold on;
    plot(1, 3.5, 'ro', 'MarkerSize',8, 'MarkerFaceColor','r'); % 起点
    text(0.5, 4, 'Start', 'Color','k','FontSize',10);hold on;
    
    % 绘制静态障碍物（灰色圆形）
%     for j = 1:size(static_obstacles,1)
%         pos = static_obstacles(j,1:2);
%         radius = static_obstacles(j,3);
%         rectangle('Position',[pos(1)-radius, pos(2)-radius, 2*radius, 2*radius],...
%             'Curvature',[1 1], 'FaceColor',[0.6 0.6 0.6], 'EdgeColor','k');
%     end
    
    % 绘制动态障碍物（红色箭头表示运动方向）
    for j = 1:size(dynamic_obstacles,1)
        pos = dynamic_obstacles(j,1:2);
        radius = dynamic_obstacles(j,5);
        % 障碍物本体
        rectangle('Position',[pos(1)-radius, pos(2)-radius, 2*radius, 2*radius],...
            'Curvature',[1 1], 'FaceColor',[1 0.5 0.5], 'EdgeColor','r');
        % 速度方向箭头
        v = dynamic_obstacles(j,3:4);
        if norm(v) > 0
            quiver(pos(1), pos(2), v(1)*0.5, v(2)*0.5,...
                'Color','r', 'LineWidth',1.5, 'MaxHeadSize',0.5);
        end
    end
    
    axis([-1 10 -1 8]);
    drawnow;
end
end

%% 动态窗口法核心函数（支持混合障碍物）
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,static_obs,dynamic_obs)
    Vr = CalcDynamicWindow(x,model);
    [evalDB,trajDB] = Evaluation(x,Vr,goal,static_obs,dynamic_obs,model,evalParam);
    
    if isempty(evalDB)
        u=[0;0]; return;
    end
    
    % 正则化评价函数
    evalDB = NormalizeEval(evalDB);
    feval = evalDB(:,3:5)*evalParam(1:3)';
    [~,ind] = max(feval);
    u = evalDB(ind,1:2)'; 
end

%% 评价函数（混合障碍物处理）
function [evalDB,trajDB] = Evaluation(x,Vr,goal,static_obs,dynamic_obs,model,evalParam)
    evalDB = []; trajDB = [];
    for vt = Vr(1):model(5):Vr(2)
        for ot = Vr(3):model(6):Vr(4)
            [~,traj] = GenerateTrajectory(x, vt, ot, evalParam(4), model);
            
            % 计算到静态障碍物的最小距离
            static_dist = CalcStaticDist(traj(1:2,:), static_obs);
            
            % 计算到动态障碍物的最小预测距离
            dynamic_dist = CalcDynamicDist(traj, dynamic_obs);
            
            % 综合最小距离
            min_dist = min(static_dist, dynamic_dist);
            
            if min_dist > CalcBreakingDist(abs(vt),model)
                heading = CalcHeadingEval(traj(:,end), goal);
                evalDB = [evalDB; vt ot heading min_dist abs(vt)];
                trajDB = [trajDB; traj'];
            end
        end
    end
end

%% 静态障碍物距离计算
function min_dist = CalcStaticDist(traj, static_obs)
    min_dist = inf;
    for i = 1:size(static_obs,1)
        obs_pos = static_obs(i,1:2);
        radius = static_obs(i,3);
        % 计算轨迹所有点到障碍物的距离
        dists = sqrt( (traj(1,:)-obs_pos(1)).^2 + (traj(2,:)-obs_pos(2)).^2 ) - radius;
        current_min = min(dists);
        if current_min < min_dist
            min_dist = current_min;
        end
    end
    if isempty(static_obs)  % 处理无静态障碍物情况
        min_dist = inf;
    end
end

%% 动态障碍物距离计算
function min_dist = CalcDynamicDist(traj, dynamic_obs)
    global dt;
    min_dist = inf;
    if isempty(dynamic_obs), return; end
    
    % 提取轨迹时间和位置
    traj_time = (0:size(traj,2)-1)*dt;
    x_traj = traj(1,:);
    y_traj = traj(2,:);
    
    for j = 1:size(dynamic_obs,1)
        % 获取障碍物参数
        obs_x0 = dynamic_obs(j,1);
        obs_y0 = dynamic_obs(j,2);
        vx = dynamic_obs(j,3);
        vy = dynamic_obs(j,4);
        radius = dynamic_obs(j,5);
        
        % 计算障碍物在轨迹时间点的位置
        obs_x = obs_x0 + vx*traj_time;
        obs_y = obs_y0 + vy*traj_time;
        
        % 计算每个时刻的距离
        dists = sqrt( (x_traj-obs_x).^2 + (y_traj-obs_y).^2 ) - radius;
        current_min = min(dists);
        if current_min < min_dist
            min_dist = current_min;
        end
    end
end

%% 其他辅助函数保持不变（f, NormalizeEval等）
% ...（与之前提供的函数相同）...
function EvalDB=NormalizeEval(EvalDB)
% 评价函数正则化
if sum(EvalDB(:,3))~= 0
    EvalDB(:,3) = EvalDB(:,3)/sum(EvalDB(:,3));  %矩阵的数除  单列矩阵的每元素分别除以本列所有数据的和
end
if sum(EvalDB(:,4))~= 0
    EvalDB(:,4) = EvalDB(:,4)/sum(EvalDB(:,4));
end
if sum(EvalDB(:,5))~= 0
    EvalDB(:,5) = EvalDB(:,5)/sum(EvalDB(:,5));
end
end
     
function [x,traj] = GenerateTrajectory(x,vt,ot,evaldt,model)
global dt;
time = 0;
u = [vt;ot];% 输入值
traj = x;   % 机器人轨迹
while time <= evaldt   
    time = time+dt; % 时间更新
    x = f(x,u);     % 运动更新 前项模拟时间内 速度、角速度恒定
    traj = [traj x]; % 每一列代表一个轨迹点 一列一列的添加
end
end
 
%% 计算制动距离 
%根据运动学模型计算制动距离, 也可以考虑成走一段段圆弧的累积 简化可以当一段段小直线的累积
function stopDist = CalcBreakingDist(vel,model)
global dt;
MD_ACC   = 3;% 
stopDist=0;
while vel>0   %给定加速度的条件下 速度减到0所走的距离
    stopDist = stopDist + vel*dt;% 制动距离的计算 
    vel = vel - model(MD_ACC)*dt;% 
end
end
 
 
%% heading的评价函数计算
% 输入参数：当前位置、目标位置
% 输出参数：航向参数得分  当前车的航向和相对于目标点的航向 偏离程度越小 分数越高 最大180分
function heading = CalcHeadingEval(x,goal)
theta = toDegree(x(3));% 机器人朝向
goalTheta = toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% 目标点相对于机器人本身的方位 
if goalTheta > theta
    targetTheta = goalTheta-theta;% [deg]
else
    targetTheta = theta-goalTheta;% [deg]
end
 
heading = 180 - targetTheta;  
end

%% 计算动态窗口
% 返回 最小速度 最大速度 最小角速度 最大角速度速度
function Vr = CalcDynamicWindow(x,model)
V_SPD       = 4;%机器人速度
W_ANGLE_SPD = 5;%机器人角速度 
MD_MAX_V = 1;% 
MD_MAX_W = 2;% 
MD_ACC   = 3;% 
MD_VW    = 4;% 
global dt;
% 车子速度的最大最小范围 依次为：最小速度 最大速度 最小角速度 最大角速度速度
Vs=[0 model(MD_MAX_V) -model(MD_MAX_W) model(MD_MAX_W)];
 
% 根据当前速度以及加速度限制计算的动态窗口  依次为：最小速度 最大速度 最小角速度 最大角速度速度
Vd = [x(V_SPD)-model(MD_ACC)*dt x(V_SPD)+model(MD_ACC)*dt x(W_ANGLE_SPD)-model(MD_VW)*dt x(W_ANGLE_SPD)+model(MD_VW)*dt];
 
% 最终的Dynamic Window
Vtmp = [Vs;Vd];  %2 X 4  每一列依次为：最小速度 最大速度 最小角速度 最大角速度速度
Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
end

%% Motion Model 根据当前状态推算下一个控制周期（dt）的状态
 %% 示例函数补充
function degree = toDegree(radian)
    degree = radian/pi*180;
end

function radian = toRadian(degree)
    radian = degree/180*pi;
end

function x = f(x, u)
    global dt;
    F = [1 0 0 0 0; 0 1 0 0 0; 0 0 1 0 0; 0 0 0 0 0; 0 0 0 0 0];
    B = [dt*cos(x(3)) 0; dt*sin(x(3)) 0; 0 dt; 1 0; 0 1];
    x = F*x + B*u;
end