function [pose, bestHits] = BruteMatch(gridmap, scan, pose,...
                            bruteResolution, tmax, rmax)

% tmax = 4, rmax = pi/6
% 和fastMatch()类似
                        
% Grid map information
metricMap = gridmap.metricMap; % 距离Matrix,记录与false处与最近true的距离
ipixel = 1 / gridmap.pixelSize;
minX   = gridmap.topLeftCorner(1); % gridMap的最小横坐标(全局)用于标定全局位置
minY   = gridmap.topLeftCorner(2); % gridMap的最小纵坐标(全局)用于标定全局位置
nCols  = size(metricMap, 2);
nRows  = size(metricMap, 1);

% 野蛮搜索空间                          
xs = pose(1) - tmax : bruteResolution(1) : pose(1) + rmax;
ys = pose(2) - tmax : bruteResolution(2) : pose(2) + rmax;
rs = pose(3) - rmax : bruteResolution(3) : pose(3) + rmax;
nx = length(xs);    ny = length(ys);    nr = length(rs); % 三者长度

pixelScan = scan * ipixel; % scan为当此扫描的一组局部笛卡尔坐标, pixelScan为一组局部grid坐标
scores = Inf(nx, ny, nr); % 记录一组score, 包含所有搜索点,初始值被设定为Inf,目标为最小化scores
bestScore = Inf;

% 旋转局部坐标
for ir = 1 : nr
    theta = rs(ir); % 根据索引遍历
    ct = cos(theta);
    st = sin(theta);
    S  = pixelScan * [ct, st; -st, ct]; % 将这一组进行旋转变换
    % x向平移变换
    for ix = 1 : nx
        tx = xs(ix);
        Sx = round(S(:,1)+(tx-minX)*ipixel) + 1;
        
        % y向平移变换
        for iy = 1 : ny
            ty = ys(iy);
            Sy = round(S(:,2)+(ty-minY)*ipixel) + 1;
            
            % 计算重合部分栅格的重合度
            isIn = Sx>1 & Sy>1 & Sx<nCols & Sy<nRows;
            idx  = Sy(isIn) + (Sx(isIn)-1)*nRows;
            hits = metricMap(idx);
            score = sum(hits); % 点与最近命中点的距离矩阵(一维)
            
            scores(ix, iy, ir) = score;
            
            if score < bestScore
                bestScore = score;
                bestHits = hits; % 命中点的最近true距离矩阵的距离集合，包含true点，数值为0
                pose = [tx; ty; theta];
            end
            
        end  
    end
end
