%根据当前位姿的栅格地图 优化预测的下一位姿 使下一位姿的栅格地图与当前位姿的栅格地图达到最大的重合度
%快速扫描匹配(请注意这可能会陷入局部最小值)
function [pose, bestHits] = FastMatch(gridmap, scan, pose, searchResolution)
%--------------------------------------------------------------------------
%输入
%   gridmap为局部栅格地图
%   scan为构成gridmap的当前扫描点集的局部笛卡尔坐标
%   pose为预测的下一位姿(预测得到的pose_guess)
%   searchResolution为搜索的分辨率(为主函数中预设的扫描匹配参数 [0.05; 0.05; deg2rad(0.5)] ) 
%输出
%   pose为优化过后的 预测下一位姿 优化目标函数是使下一位姿的栅格地图与当前位姿的栅格地图达到最大的重合度
%   bestHits 为pose对应最佳重合度score对应的原始距离矩阵
%--------------------------------------------------------------------------

% 局部栅格地图信息
metricMap = gridmap.metricMap; % 栅格地图中0元素所在的位置靠近非零元素位置的最短栅格距离构成的矩阵
ipixel = 1 / gridmap.pixelSize; % 实际距离1m对应几个栅格单元边长 (栅格单元尺寸对应的实际距离的倒数)
minX   = gridmap.topLeftCorner(1); % 栅格地图中的最左端的横坐标(全局)
minY   = gridmap.topLeftCorner(2); % 栅格地图中的最下端的纵坐标(全局)
nCols  = size(metricMap, 2); % 栅格地图列数
nRows  = size(metricMap, 1); % 栅格地图行数

% 爬山算法
maxIter = 50; % 最大循环次数
maxDepth = 3; % 提高分辨率的次数的最大值
iter = 0; % 循环变量
depth = 0; % 分辨率提高次数

pixelScan = scan * ipixel; % 将扫描数据实际坐转化为栅格坐标
bestPose  = pose;
bestScore = Inf;
t = searchResolution(1); % x, y搜索分辨率
r = searchResolution(3); % theta搜索分辨率

while iter < maxIter
    noChange = true;
    % 旋转变换
    for theta = pose(3) + [-r, 0, r]%遍历三个旋转角增量
        
         ct = cos(theta);
        st = sin(theta);
        S  = pixelScan * [ct, st; -st, ct];
        % 转换
        % Translation
        for tx = pose(1) + [-t, 0, t] % 遍历三个坐标增量
            Sx = round(S(:,1)+(tx-minX)*ipixel) + 1; % 以栅格为单位的横坐标左上角已对齐
            for ty = pose(2) + [-t, 0, t]
                Sy = round(S(:,2)+(ty-minY)*ipixel) + 1;
                
                isIn = Sx>1 & Sy>1 & Sx<nCols & Sy<nRows; % 筛选下一位姿得到的扫描栅格与当前局部栅格重合部分的坐标 1为栅格地图边长
                ix = Sx(isIn);
                iy = Sy(isIn);
                
                % 重合度判据
                idx = iy + (ix-1)*nRows; % 把下一位姿扫描栅格的二维坐标转换为一维坐标
                % metricMap为距离矩阵
                %score为得分重合部分距离加和 低数值表示重合度较高
                hits = metricMap(idx);
                score = sum(hits);
                
                if score < bestScore
                    noChange  = false;
                    bestPose  = [tx; ty; theta];
                    bestScore = score;
                    bestHits  = hits;
                end
                
            end
        end
    end
    % 找不到更好的匹配，提高分辨率
    if noChange
        r = r / 2;
        t = t / 2;
        depth = depth + 1;
        if depth > maxDepth % 分辨率提高次数不能超过maxDepth
            break;
        end
    end
    pose = bestPose; % 最佳位姿作为预测的下一位姿
    iter = iter + 1;
end