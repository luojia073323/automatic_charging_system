%% A*�㷨ʵ�� (����֮ǰ�ṩ��A*��������)
function path = AStar()
%% ��ʼ������
clear all; clc;
figure;
set(gcf,'position',[500,200,500,400]);
disp('A* Path Planning');

%% ��ͼ��������
area = [-3 15 -1 12]; % ��ͼ��Χ
gridSize = 0.3;               % դ���С

%% �ϰ��ﶨ�� (��������)
obstacle = [3 7.3;
           12.5 7.3;
           -0.3 2.6;
           9 2.6];
obstacleR = 2;        % �ϰ���Ӱ��뾶

%% ����յ㶨��
start = [13, 0];         % �������
goal = [0.87,9.65];     % Ŀ�������

%% A*·���滮
path = AStarAlgorithm(start, goal, obstacle, area, gridSize,obstacleR);

show_map2;
hold on;

%% ��ʾ�滮���
if ~isempty(path)
    plot(path(:,1), path(:,2), 'r', 'LineWidth', 2);hold on;
    plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor','r');hold on;
    plot(goal(1), goal(2), 'g*', 'MarkerSize', 12);hold on;
    text(start(1)+0.6, start(2), 'Start', 'Color','k','FontSize',10);hold on;
%     text(goal(1)-1.5, goal(2), 'Goal', 'Color','k','FontSize',10);hold on;
else
    disp('No valid path found!');
end

axis(area);
xlabel('X (m)'); ylabel('Y (m)');
title('A*');
grid on;
end

function path = AStarAlgorithm(start, goal, obstacle, area, gridSize,obstacleR)
%% ����դ���ͼ
xmin = area(1); xmax = area(2);
ymin = area(3); ymax = area(4);
width = ceil((xmax-xmin)/gridSize);
height = ceil((ymax-ymin)/gridSize);

% ��ʼ����ͼ 0-���� 1-�ϰ�
map = zeros(height, width);

%% �ϰ���ӳ�䵽դ��
for i = 1:size(obstacle,1)
    % �����ϰ�������դ��
    col = ceil((obstacle(i,1)-xmin)/gridSize);
    row = ceil((obstacle(i,2)-ymin)/gridSize);
    bufferCells = ceil(obstacleR / gridSize);
    % ����ϰ����ܱ�����(�򵥴���)
%     for r = max(1,row-2):min(height,row+2)
%         for c = max(1,col-2):min(width,col+2)
    for r = max(1,row-bufferCells):min(height,row+bufferCells)
        for c = max(1,col-bufferCells):min(width,col+bufferCells)
            map(r,c) = 1;
        end
    end
end

%% A*�㷨ʵ��
% �ڵ�ṹ��
nodes = struct('pos',{},'parent',{},'g',{},'h',{},'f',{});

% ����յ�ת������
startNode = [ceil((start(1)-xmin)/gridSize), ceil((start(2)-ymin)/gridSize)];
goalNode = [ceil((goal(1)-xmin)/gridSize), ceil((goal(2)-ymin)/gridSize)];

% ��ʼ�������б�
openList = [];
closedList = [];

% �������
nodes(1).pos = startNode;
nodes(1).parent = [];
nodes(1).g = 0;
nodes(1).h = norm(goalNode - startNode);
nodes(1).f = nodes(1).g + nodes(1).h;
openList = [1];

%% ��ѭ��
found = false;
while ~isempty(openList)
    % Ѱ��fֵ��С�Ľڵ�
    [~, idx] = min([nodes(openList).f]);
    currentIdx = openList(idx);
    current = nodes(currentIdx);
    
    % ����Ŀ���
    if isequal(current.pos, goalNode)
        found = true;
        break;
    end
    
    % �ƶ������б�
    openList(idx) = [];
    closedList = [closedList currentIdx];
    
    % �������ڽڵ�
    neighbors = [];
    for i = -1:1
        for j = -1:1
            if i==0 && j==0, continue; end
            newPos = current.pos + [i,j];
            % ���߽�
            if newPos(1)<1 || newPos(1)>width || newPos(2)<1 || newPos(2)>height
                continue;
            end
            % ����ϰ���
            if map(newPos(2), newPos(1)) == 1
                continue;
            end
            neighbors = [neighbors; newPos];
        end
    end
    
    % �������ڽڵ�
    for k = 1:size(neighbors,1)
        neighborPos = neighbors(k,:);
        % ����Ƿ��ڱ��б�
        if ~isempty(closedList)
            closedPos = vertcat(nodes(closedList).pos);
            if ismember(neighborPos, closedPos, 'rows')
            continue;
            end
        end
        
        % �������
        moveCost = norm(neighborPos - current.pos);
        gNew = current.g + moveCost;
        hNew = norm(goalNode - neighborPos);
        
        % �����Ƿ��ڿ��б�
        inOpen = find(arrayfun(@(x) isequal(x.pos, neighborPos), nodes));
        if ~isempty(inOpen)
            if gNew < nodes(inOpen).g
                nodes(inOpen).g = gNew;
                nodes(inOpen).f = gNew + hNew;
                nodes(inOpen).parent = currentIdx;
            end
        else
            % �����½ڵ�
            newNode.pos = neighborPos;
            newNode.parent = currentIdx;
            newNode.g = gNew;
            newNode.h = hNew;
            newNode.f = gNew + hNew;
            nodes(end+1) = newNode;
            openList = [openList length(nodes)];
        end
    end
end

%% ·������
path = [];
if found
    % ���յ����
    node = current;
    while ~isempty(node.parent)
        % ת����ʵ������
        x = (node.pos(1)-0.5)*gridSize + xmin;
        y = (node.pos(2)-0.5)*gridSize + ymin;
        path = [x, y; path];
        node = nodes(node.parent);
    end
    % ������
    x = (startNode(1)-0.5)*gridSize + xmin;
    y = (startNode(2)-0.5)*gridSize + ymin;
    path = [x, y; path];
else
    disp('No path found!');
end
end



