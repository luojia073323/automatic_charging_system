function [] = MPC_count()
%% MPC-CBF�㷨ʵ��
clear all; clc;
figure;
set(gcf,'position',[500,200,500,400]); 

%% ��ʼ������
global dt obstacle obstacleR goal;
dt = 0.5;  % ʱ�䲽��
obstacle = [3.4 7.3;
           12 7.3;
           0 2.6;
           8.5 2.4];
obstacleR = 2;  % �ϰ���뾶
goal = [8, 10];   % Ŀ���
area = [-1.8 16.305 -0.1 11.405]; % ��ͼ��Χ

%% ��ʼ������ά�Ȳ���
total_distance = 0;       % ��·������
max_curvature = 0;        % �������
min_obstacle_dist = Inf; % ��С�ϰ�����
planning_time = 0;        % �ܹ滮��ʱ
num_iterations = 0;       % ��������
end_error = 0;            % �յ�ƫ�����

%% �����˳�ʼ״̬ [x, y, theta, v, w]
x = [6, 0, pi/2, 0, 0]'; 
result.x = []; % �켣�洢

%% MPC����
N = 10;   % Ԥ��ʱ��
Q = diag([10, 10, 0.1]); % ״̬Ȩ��
R = diag([0.01, 0.01]);  % ������Ȩ��
u_min = [-2.0, -pi/2]'; % ����������
u_max = [1.5, pi/2]';   % ����������

%% ��ѭ��
result.x = x';
for i = 1:2000
    % ���MPC�Ż�����
    % ��ֹ����
    if norm(x(1:2) - goal') < 0.1
         % 4. �յ�ƫ�����
        end_error = norm(x(1:2)' - goal);
        converged = true; % ��ǳɹ�����
    break;
    end
    
    % ��¼�滮��ʼʱ��
    tic;
    u = solve_MPC_CBF(x, N, Q, R, u_min, u_max);
     
    % �ۼƹ滮ʱ��
    planning_time = planning_time + toc;
    num_iterations = num_iterations + 1;
    
    prev_pos = x(1:2)';
    
    % ״̬����
    x = robot_model(x, u);
    result.x = [result.x; x'];
    
    %% ��������ά��
    % 1. ·�����ȣ��ۼ�ÿ���ƶ����룩
    current_pos = x(1:2)';
    total_distance = total_distance + norm(current_pos - prev_pos);
    
    % 2. ������ʣ����� = ���ٶ�/���ٶȣ�
    v = x(4);
    w = x(5);
    if abs(v) > 1e-3 % �������
        current_curvature = abs(w / v);
        max_curvature = max(max_curvature, current_curvature);
    end
    % 3. ��С�ϰ����루���������ϰ��
    for obs = 1:size(obstacle,1)
        dist = norm(current_pos - obstacle(obs,:)) - obstacleR;
        min_obstacle_dist = min(min_obstacle_dist, dist);
    end
    
    if norm(x(1:2) - goal') < 0.2
         % 4. �յ�ƫ�����
        end_error = norm(x(1:2)' - goal);
        converged = true; % ��ǳɹ�����
    break;
    end
    
    % ��ͼ
%     plot_navigation(x, result, goal, obstacle, X_pred);
    plot_navigation(x, result, goal, obstacle);
end

%% ��ӡ�������
disp('============== ����ά�� ==============');
disp(['·������:        ', num2str(total_distance), ' ��']);
disp(['�������:        ', num2str(max_curvature), ' rad/m']);
disp(['��С�ϰ�����:    ', num2str(min_obstacle_dist), ' ��']);
disp(['���ι滮��ʱ:    ', num2str(planning_time/num_iterations), ' ��/��']);
disp(['������������:    ', num2str(num_iterations), ' ��']); % ������
disp(['�յ�ƫ�����:    ', num2str(end_error), ' ��']);
disp('======================================');
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
        
        % ����ն˴���
        terminal_error = X(1:2, end) - goal(:);
        cost = cost + terminal_error' * diag([100, 100]) * terminal_error;

        % CBFԼ�������ϣ�
        for obs = 1:size(obstacle,1)
        dx = X(1,k+1) - obstacle(obs,1);
        dy = X(2,k+1) - obstacle(obs,2);
        vx = X(4,k+1) * cos(X(3,k+1));
        vy = X(4,k+1) * sin(X(3,k+1));   

        % ��ȫ���빫ʽ��������ٶ�Ӱ�죩
        h = (dx)^2 + (dy)^2 - (obstacleR + 0.2)^2;  % ����0.2m��ȫ����
        h_dot = 2*(dx*vx + dy*vy);
        constraints = [constraints, h_dot + 2.0*h >= 0];  % ��K����������Ϊ2.0
        
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
    % ��plot_navigation��������ӣ�
    plot(goal(1), goal(2), '*g'); 
    start = [6, 0]; 
    plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');hold on;
    text(start(1)+0.6, start(2), 'Start', 'Color','k','FontSize',10);hold on;

    axis([-3 15 -1 12]); grid on;
    title('A*-MPC-CBF');
    drawnow;
end
