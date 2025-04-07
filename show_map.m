function show_map
% ��������
map_length = 9;     % ��������
map_width = 7.2;    % �������
shelf_size = 1.2;   % ���ܱ߳�

% ��������
axis([-1 10 -1 8]);  % ��С10��
xlabel('x (m)');
ylabel('y (m)');
axis equal
grid on

% ���Ƴ����߽�
% plot([0 map_length], [0 0], 'k-', 'LineWidth', 2);       % �±߽�
% plot([0 0], [0 map_width], 'k-', 'LineWidth', 2);       % ��߽�
% plot([map_length map_length], [0 map_width], 'k-', 'LineWidth', 2); % �ұ߽�
% plot([0 map_length], [map_width map_width], 'k-', 'LineWidth', 2);  % �ϱ߽�

% ��������ƺ���
function draw_shelf(center_x, center_y)
    half_size = shelf_size/2;
    % ���ƻ�������
    rectangle('Position',[center_x-half_size, center_y-half_size, shelf_size, shelf_size],...
              'EdgeColor','k','LineWidth',1.5);
     text(center_x, center_y,...
    '����','Color','k','HorizontalAlignment','center');
    % ��ӶԽ�����ǿ��ʾ
    plot([center_x-half_size, center_x+half_size],...
         [center_y+half_size, center_y-half_size],'k:','LineWidth',0.5);
    plot([center_x-half_size, center_x+half_size],...
         [center_y-half_size, center_y+half_size],'k:','LineWidth',0.5);
end

% �����鲼�֣�ʾ�����꣩
draw_shelf(2.5, 2.3);     % ��һ�����
draw_shelf(2.5, 5);     % �ڶ������

% �������ʶ��ʾ��λ�ã�
charging_station = [7.6, 1.6];  
rectangle('Position',[charging_station(1), charging_station(2), 0.6, 0.8],...
          'EdgeColor','y','LineWidth',1.5,'LineStyle','-');
 text(charging_station(1)+0.3, charging_station(2)+11,...
    '�������','Color','k','HorizontalAlignment','center');

charging_pot = [7.8, 0];  
rectangle('Position',[charging_pot(1), charging_pot(2), 0.3, 0.3],...
          'EdgeColor','g','LineWidth',1.5,'LineStyle','-');
text(charging_pot(1)+0.15, charging_pot(2)+0.55,...
 '���׮','Color','k','HorizontalAlignment','center');

% ����Ԫ��
% legend('������','�������','���׮');
set(gca, 'GridLineStyle', ':', 'GridAlpha', 0.3);  % ����������ʽ
hold off;
end