%针对第一次扫描的初始化
function map = Initialize(map, pose, scan)
%--------------------------------------------------------------------------
%输入
%    map为地图(全局)
%    pose为
%    scan为
%--------------------------------------------------------------------------
% 把对于小车的局部坐标数据 转化为 全局地图坐标

map.points = Transform(scan, pose);%将转化为全局坐标后的扫描数据scan放入全局地图点集

k = length(map.keyscans);
map.keyscans(k+1).pose = pose;
map.keyscans(k+1).iBegin = 1;
map.keyscans(k+1).iEnd = size(scan, 1);
map.keyscans(k+1).loopClosed = true;
map.keyscans(k+1).loopTried = false;