function map = DetectLoopClosure(map, scan, hits, Tmax, Rmax, pixelSize)

s = 0;
k = length(map.keyscans);
while k>1 && s<50
    k = k-1;
    dT = DiffPose(map.keyscans(k).pose, map.keyscans(k+1).pose);
    s  = s + norm(dT(1:2));
end

% error过大，不认为是loopClosure
dear = 0;
pose = map.keyscans(end).pose;
for l = k : -1 : 1
    dp = DiffPose(map.keyscans(l).pose, pose);
    if norm(dp(1:2)) < Tmax+1
        dear = l;
        break;
    end
end
if dear < 1
    return;
end

% 从当前位姿处提取全局参考子地图
ndear = map.keyscans(dear).iEnd;
dearPoints = map.points(1:ndear, :);
refMap  = ExtractLocalMap(dearPoints, pose, scan, Tmax+4);

%scoreThresh = sum(hits) * 0.8;
countThresh = sum(hits==0) * 0.6;

% 搜索最佳
refGrid = OccuGrid(refMap, pixelSize);
resol = [0.05, 0.05, deg2rad(0.5)];

[bestPose, bestHits] = BruteMatch(refGrid, scan, pose, resol, Tmax, Rmax);

[map, bestPose] = AddAKeyScan(map, 1, scan, bestPose, hits, 1, 1, 1, 1);

if sum(bestHits<4) > countThresh
    disp(bestPose)
    sum(bestHits)
    pause(1);
end
        
    
    
    
    
    
    
    
    
    