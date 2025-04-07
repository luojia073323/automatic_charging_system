function show_map2
%% AMR_D (缩小10倍)
% x1 = 2.379; y1 = 0.930;
% plot([x1,x1+3.752],[y1,y1],'b','Linewidth',2);hold on;
% plot([x1,x1+3.752],[y1+1.895,y1+1.895],'b','Linewidth',2);hold on;
% plot([x1,x1],[y1,y1+4.060],'b','Linewidth',2);hold on;
% plot([x1+1.876,x1+1.876],[y1,y1+4.060],'b','Linewidth',2);hold on;
% plot([x1+3.752,x1+3.752],[y1,y1+4.060],'b','Linewidth',2);hold on;
% 
% x2 = 0.870; y2 = 4.990;
% plot([x2,x2+5.915],[y2,y2],'b','Linewidth',2);hold on;
% plot([x2,x2],[y2,y2+4.660],'b','Linewidth',2);hold on;
% 
% x3 = 8.735; y3 = 5.900;
% plot([x3,x3],[y3,y3+0.915],'b','Linewidth',2);hold on;
% plot([x3-1.950,x3],[y3+0.915,y3+0.915],'b','Linewidth',2);hold on;
% plot([x3-1.950,x3-1.950],[y3-0.910,y3+0.915],'b','Linewidth',2);hold on;

%% AMR_S (缩小10倍)
% x4 = 7.785; y4 = 2.625;
% plot([x4,x4],[y4-1.765,y4+5.950],'b','Linewidth',2);hold on;
% plot([x4+3.310,x4+3.310],[y4-1.765,y4+2.365],'b','Linewidth',2);hold on;
% plot([x4+6.620,x4+6.620],[y4-1.765,y4+2.365],'b','Linewidth',2);hold on;
% plot([x4,x4+6.620],[y4+2.365,y4+2.365],'b','Linewidth',2);hold on;
% plot([x4,x4+6.620],[y4,y4],'b','Linewidth',2);hold on;
% plot([x4+3.310,x4+6.620],[y4-1.765,y4-1.765],'b','Linewidth',2);hold on;
% plot([x4,x4+3.310],[y4-1.765,y4-1.765],'b','Linewidth',2);hold on;

%% 充电站
plot(8,10,'ko','Markersize',8);
text(7.2,10.8,'goal','Color','k','FontSize',14);

% plot(7.785,8.570,'ko','Markersize',8);
% text(6.500,9.270,'goal2','Color','k','FontSize',14);

%% 充电范围
% x5 = 0.870; y5 = 9.650;
% x51 = 0.870+0.350; x52 = 0.870-0.350; y51 = 8.650;
% line([x5,x51],[y5,y51],'Color','red','LineStyle','--');
% line([x5,x52],[y5,y51],'Color','red','LineStyle','--');
% 
% x6 = 7.785; y6 = 8.570;
% x61 = 7.785+0.350; x62 = 7.785-0.350; y61 = 7.570;
% line([x6,x61],[y6,y61],'Color','red','LineStyle','--');
% line([x6,x62],[y6,y61],'Color','red','LineStyle','--');

%% 障碍物
x01 = 3.500; y01 = 7.300; r01 = 2.000;
theta = linspace(0, 2*pi, 100);
x11 = x01 + r01 * cos(theta);
y11 = y01 + r01 * sin(theta);
patch(x11, y11, [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
text(1.90, 7.300, 'obstacle1', 'Color', 'b', 'FontSize', 14);

x02=12.000; y02=7.300; r02=2.000;
x12 = x02 + r01 * cos(theta);
y12 = y02 + r01 * sin(theta);
patch(x12, y12, [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
text(10.400,7.300,'obstacle2','Color','b','FontSize',14);

x03=0; y03=2.600; r03=2.000;
x13 = x03 + r01 * cos(theta);
y13 = y03 + r01 * sin(theta);
patch(x13, y13, [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
text(-1.500,2.600,'obstacle3','Color','b','FontSize',14);

x04=8.500; y04=2.60; r04=2.000;
x14 = x04 + r01 * cos(theta);
y14 = y04 + r01 * sin(theta);
patch(x14, y14, [0.8 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
text(7.000,2.550,'obstacle4','Color','b','FontSize',14);

% x05=12.650; y05=3.750; r05=1.000;
% rectangle('Position',[x05-r05,y05-r05,2*r05,2*r05],'Curvature',[1 1],'EdgeColor','g');
% text(12.250,3.750,'ob5','Color','b','FontSize',14);
% 
% x06=9.400; y06=1.400; r06=1.000;
% rectangle('Position',[x06-r06,y06-r06,2*r06,2*r06],'Curvature',[1 1],'EdgeColor','g');
% text(9.000,1.550,'ob6','Color','b','FontSize',14);
% 
% x07=12.800; y07=1.700; r07=0.700;
% rectangle('Position',[x07-r07,y07-r07,2*r07,2*r07],'Curvature',[1 1],'EdgeColor','g');
% text(12.350,1.675,'ob7','Color','b','FontSize',14);

%% 其他点
% plot(0.870,6.170,'bo','Markersize',10);
% text(0.870,6.170,'A','Color','k','FontSize',6);
% 
% plot(8.735,6.170,'bo','Markersize',10);
% text(8.735,6.170,'B','Color','k','FontSize',6);
% 
% plot(6.131,3.808,'bo','Markersize',10);
% text(6.131,3.808,'C','Color','k','FontSize',6);
% 
% plot(6.131,1.653,'bo','Markersize',10);
% text(6.131,1.653,'D','Color','k','FontSize',6);
% 
% plot(7.786,3.808,'mo','Markersize',10);
% text(7.786,3.808,'E','Color','k','FontSize',6);
% 
% plot(7.786,1.653,'mo','Markersize',10);
% text(7.786,1.653,'F','Color','k','FontSize',6);

%% 坐标系设置
axis([-2.200 15.000 -1.000 12.000]);  % 缩小10倍
xlabel('x (m)');
ylabel('y (m)');
axis equal
grid on

end