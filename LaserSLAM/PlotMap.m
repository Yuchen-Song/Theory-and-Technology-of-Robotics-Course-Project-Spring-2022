%绘图(点集地图、路径、当前位姿、当前LiDAR扫描线)
function PlotMap(cfig, map, path, scan, scanIdx)
%--------------------------------------------------------------------------
%输入
%   cfig为plot绘制位置(将所有时刻的图叠加在一张图上)
%   map为全局地图
%   path为路径
%   scan为当前位置的局部笛卡尔坐标
%   scanIdx为当前扫描索引
%--------------------------------------------------------------------------
world   = map.points;
scan = Transform(scan, path(:,end)); % 将当前位置的局部笛卡尔坐标 利用路径 转化为全局笛卡尔坐标

worldColor = [0 0 0]; % 地图的颜色(黑色)
scanColor = [148/255 0 211/255]; % 当前位置颜色(深紫色)
pathColor = [0 0 1]; % 路径颜色(蓝色)
lidarColor=[205/255 38/255 38/255]; % LiDAR扫描线颜色(砖红色)

cfig(1); clf; 
set(0,'defaultfigurecolor','w')
set(gca,'box','on')
set(gca, 'color', [1,1,1]); % 设置背景颜色(白色)
hold on;  axis equal;
plot(world(:,1), world(:,2), '+', 'MarkerSize', 1, 'color', worldColor); % 全局地图点集
plot(scan(:,1),  scan(:,2),  '.', 'MarkerSize', 2, 'color', scanColor); % 扫描点图
plot(path(1,:),  path(2,:),  '-.', 'LineWidth', 1, 'color', pathColor); % 路径
for i = 1:20:length(scan)
    line([path(1,end), scan(i,1)], [path(2,end), scan(i,2)], 'color', lidarColor); % 绘制当前位置的Lidar扫描线
end
title(['Scan: ', num2str(scanIdx)]);%标题
drawnow