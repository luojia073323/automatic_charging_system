function show_map
% 参数定义
map_length = 9;     % 场景长度
map_width = 7.2;    % 场景宽度
shelf_size = 1.2;   % 货架边长

% 创建画布
axis([-1 10 -1 8]);  % 缩小10倍
xlabel('x (m)');
ylabel('y (m)');
axis equal
grid on

% 绘制场景边界
% plot([0 map_length], [0 0], 'k-', 'LineWidth', 2);       % 下边界
% plot([0 0], [0 map_width], 'k-', 'LineWidth', 2);       % 左边界
% plot([map_length map_length], [0 map_width], 'k-', 'LineWidth', 2); % 右边界
% plot([0 map_length], [map_width map_width], 'k-', 'LineWidth', 2);  % 上边界

% 货架组绘制函数
function draw_shelf(center_x, center_y)
    half_size = shelf_size/2;
    % 绘制货架轮廓
    rectangle('Position',[center_x-half_size, center_y-half_size, shelf_size, shelf_size],...
              'EdgeColor','k','LineWidth',1.5);
     text(center_x, center_y,...
    '货架','Color','k','HorizontalAlignment','center');
    % 添加对角线增强显示
    plot([center_x-half_size, center_x+half_size],...
         [center_y+half_size, center_y-half_size],'k:','LineWidth',0.5);
    plot([center_x-half_size, center_x+half_size],...
         [center_y-half_size, center_y+half_size],'k:','LineWidth',0.5);
end

% 货架组布局（示例坐标）
draw_shelf(2.5, 2.3);     % 第一组货架
draw_shelf(2.5, 5);     % 第二组货架

% 充电区标识（示例位置）
charging_station = [7.6, 1.6];  
rectangle('Position',[charging_station(1), charging_station(2), 0.6, 0.8],...
          'EdgeColor','y','LineWidth',1.5,'LineStyle','-');
 text(charging_station(1)+0.3, charging_station(2)+11,...
    '待充电区','Color','k','HorizontalAlignment','center');

charging_pot = [7.8, 0];  
rectangle('Position',[charging_pot(1), charging_pot(2), 0.3, 0.3],...
          'EdgeColor','g','LineWidth',1.5,'LineStyle','-');
text(charging_pot(1)+0.15, charging_pot(2)+0.55,...
 '充电桩','Color','k','HorizontalAlignment','center');

% 辅助元素
% legend('货架组','待充电区','充电桩');
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3);  % 设置网格样式
hold off;
end