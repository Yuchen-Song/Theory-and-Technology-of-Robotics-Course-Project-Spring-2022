%主函数
clear; close all; clc;
cfig = figure(1);

% 激光雷达的传感器参数
lidar = SetLidarParameters();

% 地图参数
borderSize      = 1;            % 边界尺寸
pixelSize       = 0.2;          % 栅格地图的一个单元的边长 对应 实际距离pixelSize米(这里设置为0.2米)
miniUpdated     = false;        % 
miniUpdateDT    = 0.1;          % 若机器人在x方向或y方向移动超过miniUpdateDT 则更新位姿 //-判据711
miniUpdateDR    = deg2rad(5);   % 单位rad 若机器人旋转超过miniUpdateDR 则更新位姿 //-判据711
% 如果机器人从最后一次键扫描移动了0.1米或旋转了5度，我们将添加一个新的keyscan并更新地图

% 扫描匹配参数
fastResolution  = [0.05; 0.05; deg2rad(0.5)]; % [m; m; rad]的分辨率 //0.05m, 0.05m, 0.5°
bruteResolution = [0.01; 0.01; deg2rad(0.1)]; % close loop detection中使用的位姿优化分辨率

% 读取激光雷达数据
lidar_data = load('horizental_lidar.mat');

N = size(lidar_data.timestamps, 1); % 扫描总次数(控制下面的循环次数)

% 构造一个空全局地图
map.points = []; % 地图点集 代表Lidar碰撞障碍物的点集
map.connections = []; % 用于closeloop
map.keyscans = []; % keyscans保存当前正确位姿的扫描数据 如果预测得到的下一位姿出现错误 则返回到距其最近的前一位姿重新计算 //-7存在回溯
pose = [0; 0; 0]; % 初始位姿为(x=0,y=0,theta=0)
path = pose; % 位姿并置构成路径 //path为接受的位姿集合

saveFrame=0;
if saveFrame==1
    writerObj=VideoWriter('SLAMprocess.avi');
    open(writerObj);
end

% 正式开始迭代
for scanIdx = 1 : 1 : N %//扫描5522次
    disp(['scan ', num2str(scanIdx)]); %在console展示当前扫描编号
    
    scan = ReadAScan(lidar_data, scanIdx, lidar, 24); % 得到该次扫描数据的局部笛卡尔坐标
    
    % 如果是第一次扫描 则初始化
    if scanIdx == 1
        map = Initialize(map, pose, scan); % 把扫描数据scan坐标 通过pose转换为全局地图map坐标
        miniUpdated = true; % 标记flag
        continue;
    end

    % 1. 如果执行了 miniUpdate(flag标记)，我们将更新局部点集图(笛卡尔)和局部栅格地图
    if miniUpdated
        localMap = ExtractLocalMap(map.points, pose, scan, borderSize); % 得到当前扫描的全局坐标
        gridMap1 = OccuGrid(localMap, pixelSize); % 从点集localMap 栅格单元尺寸pixelSize 创建occupancy grid
        gridMap2 = OccuGrid(localMap, pixelSize/2); % 从点集localMap 栅格单元尺寸pixelSize/2 创建occupancy
    end
    
    % 2. 使用恒定速度运动模型预测当前位姿(即用前一状态到本状态的过程 作为本状态到下一状态的过程 从而由本状态预测下一状态)
    if scanIdx > 2
        pose_guess = pose + DiffPose(path(:,end-1), pose);%预测下一位姿=当前位姿+(当前位姿与上一位姿的差) pose是一个全局坐标
    else
        pose_guess = pose;
    end
    
    % 3. 快速匹配
    if miniUpdated
        [pose, ~] = FastMatch(gridMap1, scan, pose_guess, fastResolution);%根据当前栅格地图 优化 预测的下一位姿
    else
        [pose, ~] = FastMatch(gridMap2, scan, pose_guess, fastResolution);
    end
    
    % 4. 使用较高的分辨率再细化 预测下一位姿
    % gridMap = OccuGrid(localMap, pixelSize/2);
    [pose, hits] = FastMatch(gridMap2, scan, pose, fastResolution/2);%返回进一步更新的下一位姿pose
    
    % 如果机器人移动了一定距离，则执行miniUpdate(设置flag)
    dp = abs(DiffPose(map.keyscans(end).pose, pose)); % 两次位姿的差
    if dp(1)>miniUpdateDT || dp(2)>miniUpdateDT || dp(3)>miniUpdateDR
        miniUpdated = true;
        [map, pose] = AddAKeyScan(map, gridMap2, scan, pose, hits,...
                        pixelSize, bruteResolution, 0.1, deg2rad(3));
    else
        miniUpdated = false;
    end
    
    path = [path, pose]; % 把当前pose置入path     
    
    if miniUpdated
        if TryLoopOrNot(map) % 判断是否有需求检测
            map.keyscans(end).loopTried = true;
            map = DetectLoopClosure(map, scan, hits, 4, pi/6, pixelSize);
        end
    end
    
    % 绘制
    if mod(scanIdx, 30) == 0
        PlotMap(cfig, map, path, scan, scanIdx);
        if saveFrame==1
            frame = getframe(cfig);
            writeVideo(writerObj, frame);
        end
    end
end
if saveFrame==1
    close(writerObj);
end