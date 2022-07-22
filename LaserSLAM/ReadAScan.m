%将Lidar第idx次扫描数据从极坐标转化为笛卡尔坐标(相对于小车的局部坐标)
%//将第idx次的所有数据从极坐标转化为笛卡尔坐标，并根据雷达范围，将视野外的碰撞点删除(在这为扫描角度范围)
function scan = ReadAScan(lidar_data, idx, lidar, usableRange)
%--------------------------------------------------------------------------
% 输入:
%lidar_data为读取的LiDAR扫描数据
%idx为扫描次数的索引值
%lidar为由SetLidarParameters()设置的LiDAR参数
%usableRange为可使用的范围 //由lidar参数决定
%--------------------------------------------------------------------------
    angles = lidar.angles;
    ranges = lidar_data.ranges(idx, :)'; % 选取LiDAR数据的ranges中idx索引对应的这次扫描的数据
    % 删除范围不合理的点
    maxRange = min(lidar.range_max, usableRange);
    isBad = ranges < lidar.range_min | ranges > maxRange; % ranges中小于最小角度或大于最大角度的 数据的 索引下标
    angles(isBad) = [];
    ranges(isBad) = []; % 对应索引被删除
    % 从极坐标转换为笛卡尔坐标
    [xs, ys] = pol2cart(angles, ranges);%(angles, ranges)为极坐标中的(theta,rho) //Lidar扫描的均为极坐标系下的点
    scan = [xs, ys];  
end