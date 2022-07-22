% 从点集创建占用栅格地图
function gridmap = OccuGrid(pts, pixelSize)
%--------------------------------------------------------------------------
%输入
%   pts为当前扫描得到点集的全局坐标   
%   pixelSize表示 栅格地图一个单元的边长 对应 实际距离pixelSize米
%--------------------------------------------------------------------------
% 网格尺寸
% Grid size
minXY = min(pts) - 3 * pixelSize; % 返回x的最小值和y的最小值构成的向量
maxXY = max(pts) + 3 * pixelSize; % 构成的栅格地图中占用栅格最边界离地图边界留有3个栅格单元的余量
Sgrid = round((maxXY - minXY) / pixelSize) + 1;

N = size(pts, 1);
% hits为被占用的栅格的二维坐标 (第hits(1)块,第hits(2)块)
hits = round( (pts-repmat(minXY, N, 1)) / pixelSize ) + 1;
% 上面这一步使得 得到的栅格地图会较原始地图出现一个翻转(当点集里不存在左下角时会出现翻转)
idx = (hits(:,1)-1)*Sgrid(2) + hits(:,2);%把被占用的栅格的二维坐标转化为一维坐标

%构造一个空的栅格地图
grid  = false(Sgrid(2), Sgrid(1));

grid(idx) = true;

gridmap.occGrid = grid;
gridmap.metricMap = min(bwdist(grid),10); % grid中0元素所在的位置靠近非零元素位置的最短距离构成的矩阵
gridmap.pixelSize = pixelSize;%栅格单元边长对应的实际长度
gridmap.topLeftCorner = minXY;%栅格地图的x最小值和y最小值构成的向量的全局坐标(用于栅格对齐)