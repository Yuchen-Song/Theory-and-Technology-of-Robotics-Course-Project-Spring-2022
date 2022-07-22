%Lidar参数

function lidar = SetLidarParameters()

lidar.angle_min = -2.351831; % 最小扫描角
lidar.angle_max =  2.351831; % 最大扫描角
lidar.angle_increment = 0.004363; % 角度增量  即Lidar相邻线束之间的夹角
lidar.npoints   = 1079;
lidar.range_min = 0.023;
lidar.range_max = 60;
lidar.scan_time = 0.025; % 扫描时间
lidar.time_increment  = 1.736112e-05; % 时间增量
lidar.angles = (lidar.angle_min : lidar.angle_increment : lidar.angle_max)'; % 一次扫描各线束的角度集合
