% 从全局地图中 提取当前扫描周围的局部地图 的全局坐标
function localMap = ExtractLocalMap(points, pose, scan, borderSize)
%--------------------------------------------------------------------------
%输入
%   points为全局地图点集
%   pose为当前位姿
%   scan为当前扫描数据的局部坐标
%   borderSize为//全局地图的范围
%--------------------------------------------------------------------------

% 将当前扫描数据坐标scan 转化为全局坐标
scan_w = Transform(scan, pose);

% 设置左上角和右下角坐标，划定局部地图范围
minX = min(scan_w(:,1) - borderSize);
minY = min(scan_w(:,2) - borderSize);
maxX = max(scan_w(:,1) + borderSize);
maxY = max(scan_w(:,2) + borderSize);

% 提取位于范围内的全局地图中的点
isAround = points(:,1) > minX...
         & points(:,1) < maxX...
         & points(:,2) > minY...
         & points(:,2) < maxY;

% 从全局地图中提取到的当前扫描点
localMap = points(isAround, :);