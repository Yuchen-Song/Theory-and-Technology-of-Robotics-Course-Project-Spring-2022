%将预测下一位姿的地图添加到全局地图中
%或者如果判断下一位姿出现了错误，回到的距其最近的正确位姿，重新往后进行
function [map, pose] = AddAKeyScan(map,...
                                   gridMap,...
                                   scan,... 
                                   pose,... 
                                   hits,...
                                   pixelSize,... 
                                   bruteResolution,... 
                                   tmax,...
                                   rmax)
%--------------------------------------------------------------------------
%输入
%   map为全局地图
%   gridMap
%   scan为当前扫描数据的局部笛卡尔坐标
%   pose为优化后的预测的下一位姿
%   hits为占用栅格地图(按照矩阵单位索引拉长为一维)
%   pixelSize occupancy grid map的grid边长
%   bruteResolution为野蛮搜索分辨率
%   tmax
%   rmax
%输出
%   map为在当前全局地图基础上 添加了下一位姿测量数据的地图
%   pose为 如果预测的下一步位姿出现错误 返回到的距其最近的正确位姿 再重新往后进行
%--------------------------------------------------------------------------
% 评估pose和hits，确保没有大的错误
% 如果出现大的错误，则返回无错误最近的一步的位姿
lastKeyPose = map.keyscans(end).pose;
dp = DiffPose(lastKeyPose, pose); % 若下一位姿与当前位姿出现了较大的差别则判断下一位姿有错
if abs(dp(1)) > 0.5 || abs(dp(2)) > 0.5 || abs(dp(3)) > pi
    pose = lastKeyPose;
end

% 细化相对位姿,估计位姿协方差. 并将它们放入map.connections，当我们关闭一个循环时（姿势图优化）将需要它。
scan_w = Transform(scan, pose);%将当前扫描数据利用下一位姿转化为全局坐标(理解为估计的下一位姿的扫描数据)
newPoints = scan_w(hits>1.1, :);%把预测的下一位姿的扫描数据中，和当前栅格地图的距离大于1.1的数据 筛选出来 

if isempty(newPoints) % 预测的下一位姿的扫描数据，完全落在当前位姿构成的栅格地图中时触发条件
    return;
end

k = length(map.keyscans);
map.keyscans(k+1).pose = pose;
map.keyscans(k+1).iBegin = size(map.points, 1) + 1;
map.keyscans(k+1).iEnd = size(map.points, 1) + size(newPoints, 1);
map.keyscans(k+1).loopTried = false;
map.keyscans(k+1).loopClosed = false;
map.points = [map.points; newPoints];

c = length(map.connections);
map.connections(c+1).keyIdPair = [k, k+1];
map.connections(c+1).relativePose = zeros(3,1);
map.connections(c+1).covariance = zeros(3);