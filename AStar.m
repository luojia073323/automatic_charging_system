%% A*算法实现 (保留之前提供的A*函数内容)
function path = AStar()
%% 初始化设置
clear all; clc;
figure;
set(gcf,'position',[500,200,500,400]);
disp('A* Path Planning');

%% 地图参数定义
area = [-3 15 -1 12]; % 地图范围
gridSize = 0.3;               % 栅格大小

%% 障碍物定义 (中心坐标)
obstacle = [3 7.3;
           12.5 7.3;
           -0.3 2.6;
           9 2.6];
obstacleR = 2;        % 障碍物影响半径

%% 起点终点定义
start = [13, 0];         % 起点坐标
goal = [0.87,9.65];     % 目标点坐标

%% A*路径规划
path = AStarAlgorithm(start, goal, obstacle, area, gridSize,obstacleR);

show_map2;
hold on;

%% 显示规划结果
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
%% 创建栅格地图
xmin = area(1); xmax = area(2);
ymin = area(3); ymax = area(4);
width = ceil((xmax-xmin)/gridSize);
height = ceil((ymax-ymin)/gridSize);

% 初始化地图 0-自由 1-障碍
map = zeros(height, width);

%% 障碍物映射到栅格
for i = 1:size(obstacle,1)
    % 计算障碍物所在栅格
    col = ceil((obstacle(i,1)-xmin)/gridSize);
    row = ceil((obstacle(i,2)-ymin)/gridSize);
    bufferCells = ceil(obstacleR / gridSize);
    % 标记障碍物周边区域(简单处理)
%     for r = max(1,row-2):min(height,row+2)
%         for c = max(1,col-2):min(width,col+2)
    for r = max(1,row-bufferCells):min(height,row+bufferCells)
        for c = max(1,col-bufferCells):min(width,col+bufferCells)
            map(r,c) = 1;
        end
    end
end

%% A*算法实现
% 节点结构体
nodes = struct('pos',{},'parent',{},'g',{},'h',{},'f',{});

% 起点终点转换坐标
startNode = [ceil((start(1)-xmin)/gridSize), ceil((start(2)-ymin)/gridSize)];
goalNode = [ceil((goal(1)-xmin)/gridSize), ceil((goal(2)-ymin)/gridSize)];

% 初始化开闭列表
openList = [];
closedList = [];

% 加入起点
nodes(1).pos = startNode;
nodes(1).parent = [];
nodes(1).g = 0;
nodes(1).h = norm(goalNode - startNode);
nodes(1).f = nodes(1).g + nodes(1).h;
openList = [1];

%% 主循环
found = false;
while ~isempty(openList)
    % 寻找f值最小的节点
    [~, idx] = min([nodes(openList).f]);
    currentIdx = openList(idx);
    current = nodes(currentIdx);
    
    % 到达目标点
    if isequal(current.pos, goalNode)
        found = true;
        break;
    end
    
    % 移动到闭列表
    openList(idx) = [];
    closedList = [closedList currentIdx];
    
    % 生成相邻节点
    neighbors = [];
    for i = -1:1
        for j = -1:1
            if i==0 && j==0, continue; end
            newPos = current.pos + [i,j];
            % 检查边界
            if newPos(1)<1 || newPos(1)>width || newPos(2)<1 || newPos(2)>height
                continue;
            end
            % 检查障碍物
            if map(newPos(2), newPos(1)) == 1
                continue;
            end
            neighbors = [neighbors; newPos];
        end
    end
    
    % 处理相邻节点
    for k = 1:size(neighbors,1)
        neighborPos = neighbors(k,:);
        % 检查是否在闭列表
        if ~isempty(closedList)
            closedPos = vertcat(nodes(closedList).pos);
            if ismember(neighborPos, closedPos, 'rows')
            continue;
            end
        end
        
        % 计算代价
        moveCost = norm(neighborPos - current.pos);
        gNew = current.g + moveCost;
        hNew = norm(goalNode - neighborPos);
        
        % 查找是否在开列表
        inOpen = find(arrayfun(@(x) isequal(x.pos, neighborPos), nodes));
        if ~isempty(inOpen)
            if gNew < nodes(inOpen).g
                nodes(inOpen).g = gNew;
                nodes(inOpen).f = gNew + hNew;
                nodes(inOpen).parent = currentIdx;
            end
        else
            % 创建新节点
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

%% 路径回溯
path = [];
if found
    % 从终点回溯
    node = current;
    while ~isempty(node.parent)
        % 转换回实际坐标
        x = (node.pos(1)-0.5)*gridSize + xmin;
        y = (node.pos(2)-0.5)*gridSize + ymin;
        path = [x, y; path];
        node = nodes(node.parent);
    end
    % 添加起点
    x = (startNode(1)-0.5)*gridSize + xmin;
    y = (startNode(2)-0.5)*gridSize + ymin;
    path = [x, y; path];
else
    disp('No path found!');
end
end



