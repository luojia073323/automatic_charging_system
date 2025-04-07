function [] = MPC_CBF1()
%% MPC-CBF�㷨ʵ��
clear all; clc;
figure;
set(gcf,'position',[500,200,500,400]); 
disp('MPC-CBF Navigation Start')

%% ��ʼ������
global dt obstacle obstacleR goal;
dt = 0.2;  % ʱ�䲽��
obstacle = [3.5 7.3;
           12 7.3;
           0 2.6;
           8.5 2.6];
obstacleR = 1.7;  % �ϰ���뾶
goal = [0.870, 9.65];   % Ŀ���
area = [-1.8 16.305 -0.1 11.405]; % ��ͼ��Χ

%% �����˳�ʼ״̬ [x, y, theta, v, w]
x = [3, 0, pi/2, 0, 0]'; 
result.x = []; % �켣�洢

%% MPC����
N = 5;   % Ԥ��ʱ��
Q = diag([10, 10, 0.1]); % ״̬Ȩ��
R = diag([0.01, 0.01]);  % ������Ȩ��
u_min = [-0.5, -pi/2]'; % ����������
u_max = [1.5, pi/2]';   % ����������

%% ��ѭ��
for i = 1:5000
    % ���MPC�Ż�����
    u = solve_MPC_CBF(x, N, Q, R, u_min, u_max);
    
    % ״̬����
    x = robot_model(x, u);
    result.x = [result.x; x'];
    
    % ��ֹ����
%     if norm(x(1:2) - goal') < 0.2
%         disp('Arrived Goal!'); break;
%     end

    if norm(x(1:2) - goal') < 0.1
    break;
    end
    
    % ��ͼ
%     plot_navigation(x, result, goal, obstacle, X_pred);
    plot_navigation(x, result, goal, obstacle);
end
end

function u_opt= solve_MPC_CBF(x0, N, Q, R, u_min, u_max)
    % �����Ż��������������У�
    global dt;
    global goal; 
    global obstacle; 
    global obstacleR;
    U = sdpvar(2, N); 
    
    % ��ʼ״̬
    X = x0;
    cost = 0;
    constraints = [];
    
    % ����Ԥ��ģ����Լ��
    for k = 1:N-1
        % ״̬Ԥ�⣨�˶�ѧģ�ͣ�
        X_next = robot_model(X(:,k), U(:,k));
        X = [X, X_next];
        
        % Ŀ�꺯����������� + �������ͷ�
        error_term = X(1:3,k+1) - [goal(:); 0];
        cost = cost + error_term' * Q * error_term + U(:,k)' * R * U(:,k);
        
        % CBFԼ�������ϣ�
        for obs = 1:size(obstacle,1)
        dx = X(1,k+1) - obstacle(obs,1);
        dy = X(2,k+1) - obstacle(obs,2);
        vx = X(4,k+1) * cos(X(3,k+1));
        vy = X(4,k+1) * sin(X(3,k+1));   

        % ��ȫ���빫ʽ��������ٶ�Ӱ�죩
        h = (dx)^2 + (dy)^2 - (obstacleR + 0.2)^2;  % ����0.2m��ȫ����
        h_dot = 2*(dx*vx + dy*vy);
        constraints = [constraints, h_dot + 1.0*h >= 0];  % ��K����������Ϊ2.0
        
%         h_sq = (X(1,k+1)-obstacle(obs,1))^2 + (X(2,k+1)-obstacle(obs,2))^2 - obstacleR^2;
%         constraints = [constraints, h_sq + 0.5*dt*h_sq >= 0];
        end
    end
    
    % ������Լ��
    constraints = [constraints, repmat(u_min,1,N) <= U <= repmat(u_max,1,N)];
    
    % ����Ż�
    options = sdpsettings('solver','ipopt','verbose',0);
    options.ipopt.max_iter = 100; % ��ȷ����IPOPT������������
    optimize(constraints, cost, options);
    u_opt = value(U(:,1)); % ȡ��һ��������
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
%     % ��������ģ��
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
    % ��plot_navigation��������ӣ�
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
