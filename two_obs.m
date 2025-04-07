function [] = two_obs()
%% DWA�㷨����̬�ϰ���+��̬�ϰ��
clear all; clc; close all;
figure
set(gcf,'position',[500,200,500,400]); 

%% ��ʼ������
global dt;
dt = 0.2;
x = [1 3.5 0 0 0]'; % ��ʼ״̬
goal = [8,2];          % Ŀ��λ��

% ��̬�ϰ��� [x, y, �뾶]
static_obstacles = [
    2.5, 2.3, 0.6;
    2.5, 5, 0.6;
];

% ��̬�ϰ��� [x, y, vx, vy, �뾶]
dynamic_obstacles = [
    4.0, 5.0, 0.2, -0.2, 0.6;  % �ϰ���3б���ƶ�
%     6.0, 1.0, 0.0, 0.25, 0.6;  % �ϰ���4��ֱ����
];

Kinematic = [1.0, toRadian(20.0), 0.5, toRadian(50.0), 0.01, toRadian(1)];
evalParam = [0.01, 0.15, 0.1, 3.0];
result.x = [];

%% ��ѭ��
for i = 1:2000  
    % ���¶�̬�ϰ���λ��
    dynamic_obstacles(:,1:2) = dynamic_obstacles(:,1:2) + dynamic_obstacles(:,3:4)*dt;
    
    % DWA·���滮�����������ϰ��
    [u,traj] = DynamicWindowApproach(x, Kinematic, goal, evalParam, static_obstacles, dynamic_obstacles);
    
    x = f(x, u); % ����״̬
    result.x = [result.x; x'];
    
    % ����Ƿ񵽴�Ŀ��
    if norm(x(1:2)-goal') < 0.1
        disp('Arrive Goal!'); break;
    end
    
    % ���Ƴ���
    clf; hold on;
    show_map;hold on;
    plot(result.x(:,1), result.x(:,2), 'r', 'LineWidth',2); % �켣
    plot(goal(1), goal(2), '*r', 'MarkerSize',10);          % Ŀ���
    text(7.5, 3, 'goal', 'Color','k','FontSize',10);hold on;
    plot(1, 3.5, 'ro', 'MarkerSize',8, 'MarkerFaceColor','r'); % ���
    text(0.5, 4, 'Start', 'Color','k','FontSize',10);hold on;
    
    % ���ƾ�̬�ϰ����ɫԲ�Σ�
%     for j = 1:size(static_obstacles,1)
%         pos = static_obstacles(j,1:2);
%         radius = static_obstacles(j,3);
%         rectangle('Position',[pos(1)-radius, pos(2)-radius, 2*radius, 2*radius],...
%             'Curvature',[1 1], 'FaceColor',[0.6 0.6 0.6], 'EdgeColor','k');
%     end
    
    % ���ƶ�̬�ϰ����ɫ��ͷ��ʾ�˶�����
    for j = 1:size(dynamic_obstacles,1)
        pos = dynamic_obstacles(j,1:2);
        radius = dynamic_obstacles(j,5);
        % �ϰ��ﱾ��
        rectangle('Position',[pos(1)-radius, pos(2)-radius, 2*radius, 2*radius],...
            'Curvature',[1 1], 'FaceColor',[1 0.5 0.5], 'EdgeColor','r');
        % �ٶȷ����ͷ
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

%% ��̬���ڷ����ĺ�����֧�ֻ���ϰ��
function [u,trajDB] = DynamicWindowApproach(x,model,goal,evalParam,static_obs,dynamic_obs)
    Vr = CalcDynamicWindow(x,model);
    [evalDB,trajDB] = Evaluation(x,Vr,goal,static_obs,dynamic_obs,model,evalParam);
    
    if isempty(evalDB)
        u=[0;0]; return;
    end
    
    % �������ۺ���
    evalDB = NormalizeEval(evalDB);
    feval = evalDB(:,3:5)*evalParam(1:3)';
    [~,ind] = max(feval);
    u = evalDB(ind,1:2)'; 
end

%% ���ۺ���������ϰ��ﴦ��
function [evalDB,trajDB] = Evaluation(x,Vr,goal,static_obs,dynamic_obs,model,evalParam)
    evalDB = []; trajDB = [];
    for vt = Vr(1):model(5):Vr(2)
        for ot = Vr(3):model(6):Vr(4)
            [~,traj] = GenerateTrajectory(x, vt, ot, evalParam(4), model);
            
            % ���㵽��̬�ϰ������С����
            static_dist = CalcStaticDist(traj(1:2,:), static_obs);
            
            % ���㵽��̬�ϰ������СԤ�����
            dynamic_dist = CalcDynamicDist(traj, dynamic_obs);
            
            % �ۺ���С����
            min_dist = min(static_dist, dynamic_dist);
            
            if min_dist > CalcBreakingDist(abs(vt),model)
                heading = CalcHeadingEval(traj(:,end), goal);
                evalDB = [evalDB; vt ot heading min_dist abs(vt)];
                trajDB = [trajDB; traj'];
            end
        end
    end
end

%% ��̬�ϰ���������
function min_dist = CalcStaticDist(traj, static_obs)
    min_dist = inf;
    for i = 1:size(static_obs,1)
        obs_pos = static_obs(i,1:2);
        radius = static_obs(i,3);
        % ����켣���е㵽�ϰ���ľ���
        dists = sqrt( (traj(1,:)-obs_pos(1)).^2 + (traj(2,:)-obs_pos(2)).^2 ) - radius;
        current_min = min(dists);
        if current_min < min_dist
            min_dist = current_min;
        end
    end
    if isempty(static_obs)  % �����޾�̬�ϰ������
        min_dist = inf;
    end
end

%% ��̬�ϰ���������
function min_dist = CalcDynamicDist(traj, dynamic_obs)
    global dt;
    min_dist = inf;
    if isempty(dynamic_obs), return; end
    
    % ��ȡ�켣ʱ���λ��
    traj_time = (0:size(traj,2)-1)*dt;
    x_traj = traj(1,:);
    y_traj = traj(2,:);
    
    for j = 1:size(dynamic_obs,1)
        % ��ȡ�ϰ������
        obs_x0 = dynamic_obs(j,1);
        obs_y0 = dynamic_obs(j,2);
        vx = dynamic_obs(j,3);
        vy = dynamic_obs(j,4);
        radius = dynamic_obs(j,5);
        
        % �����ϰ����ڹ켣ʱ����λ��
        obs_x = obs_x0 + vx*traj_time;
        obs_y = obs_y0 + vy*traj_time;
        
        % ����ÿ��ʱ�̵ľ���
        dists = sqrt( (x_traj-obs_x).^2 + (y_traj-obs_y).^2 ) - radius;
        current_min = min(dists);
        if current_min < min_dist
            min_dist = current_min;
        end
    end
end

%% ���������������ֲ��䣨f, NormalizeEval�ȣ�
% ...����֮ǰ�ṩ�ĺ�����ͬ��...
function EvalDB=NormalizeEval(EvalDB)
% ���ۺ�������
if sum(EvalDB(:,3))~= 0
    EvalDB(:,3) = EvalDB(:,3)/sum(EvalDB(:,3));  %���������  ���о����ÿԪ�طֱ���Ա����������ݵĺ�
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
u = [vt;ot];% ����ֵ
traj = x;   % �����˹켣
while time <= evaldt   
    time = time+dt; % ʱ�����
    x = f(x,u);     % �˶����� ǰ��ģ��ʱ���� �ٶȡ����ٶȺ㶨
    traj = [traj x]; % ÿһ�д���һ���켣�� һ��һ�е����
end
end
 
%% �����ƶ����� 
%�����˶�ѧģ�ͼ����ƶ�����, Ҳ���Կ��ǳ���һ�ζ�Բ�����ۻ� �򻯿��Ե�һ�ζ�Сֱ�ߵ��ۻ�
function stopDist = CalcBreakingDist(vel,model)
global dt;
MD_ACC   = 3;% 
stopDist=0;
while vel>0   %�������ٶȵ������� �ٶȼ���0���ߵľ���
    stopDist = stopDist + vel*dt;% �ƶ�����ļ��� 
    vel = vel - model(MD_ACC)*dt;% 
end
end
 
 
%% heading�����ۺ�������
% �����������ǰλ�á�Ŀ��λ��
% �����������������÷�  ��ǰ���ĺ���������Ŀ���ĺ��� ƫ��̶�ԽС ����Խ�� ���180��
function heading = CalcHeadingEval(x,goal)
theta = toDegree(x(3));% �����˳���
goalTheta = toDegree(atan2(goal(2)-x(2),goal(1)-x(1)));% Ŀ�������ڻ����˱���ķ�λ 
if goalTheta > theta
    targetTheta = goalTheta-theta;% [deg]
else
    targetTheta = theta-goalTheta;% [deg]
end
 
heading = 180 - targetTheta;  
end

%% ���㶯̬����
% ���� ��С�ٶ� ����ٶ� ��С���ٶ� �����ٶ��ٶ�
function Vr = CalcDynamicWindow(x,model)
V_SPD       = 4;%�������ٶ�
W_ANGLE_SPD = 5;%�����˽��ٶ� 
MD_MAX_V = 1;% 
MD_MAX_W = 2;% 
MD_ACC   = 3;% 
MD_VW    = 4;% 
global dt;
% �����ٶȵ������С��Χ ����Ϊ����С�ٶ� ����ٶ� ��С���ٶ� �����ٶ��ٶ�
Vs=[0 model(MD_MAX_V) -model(MD_MAX_W) model(MD_MAX_W)];
 
% ���ݵ�ǰ�ٶ��Լ����ٶ����Ƽ���Ķ�̬����  ����Ϊ����С�ٶ� ����ٶ� ��С���ٶ� �����ٶ��ٶ�
Vd = [x(V_SPD)-model(MD_ACC)*dt x(V_SPD)+model(MD_ACC)*dt x(W_ANGLE_SPD)-model(MD_VW)*dt x(W_ANGLE_SPD)+model(MD_VW)*dt];
 
% ���յ�Dynamic Window
Vtmp = [Vs;Vd];  %2 X 4  ÿһ������Ϊ����С�ٶ� ����ٶ� ��С���ٶ� �����ٶ��ٶ�
Vr = [max(Vtmp(:,1)) min(Vtmp(:,2)) max(Vtmp(:,3)) min(Vtmp(:,4))];
end

%% Motion Model ���ݵ�ǰ״̬������һ���������ڣ�dt����״̬
 %% ʾ����������
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