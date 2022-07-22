function needLoopTrial = TryLoopOrNot(map)

% 控制CloseLoop detection频率
trialInterval = 1;
for k = length(map.keyscans)-1 : -1 : 1 % 从地图已有的所有keyscan遍历(keyscan为当前情况下所有认为合理的位姿集合)
    % keyscan遍历从后向前
    if ~map.keyscans(k).loopTried
        trialInterval = trialInterval + 1; % 如果当前keyscan(遍历到的)没有尝试过检测，则将检测区间长度加1
    else
        break;
    end
end
if trialInterval < 10
    needLoopTrial = false; % 每十次返回一个true, 即每十次keyscan检测一次close loop
    return;
end

% 若检测距离过近，则极有可能被认为存在close loop
% 移动路程累积量达到十米的点才有可能被检测是否存在close loop
traveledDistance = 0;
for k  = length(map.keyscans)-1 : -1 : 1 % 逆向遍历(同上)
    if ~map.keyscans(k).loopClosed %仅针对当前未认为存在close loop的，检测循环到上一次确认存在close loop为止
        dT = DiffPose(map.keyscans(k).pose, map.keyscans(k+1).pose);
        traveledDistance = traveledDistance + norm(dT(1:2)); % 累积距离增量
        if traveledDistance > 10
            break; % 累积距离增量
        end
    else
        break;
    end
end
if traveledDistance < 10 % 距离为当前keyscan到上一个认为存在close loop的keyscan的累计路程
    needLoopTrial = false; % 当前如果累积距离
    return;
end


needLoopTrial = true; %全未触发，认为有必要进行检测(检测频率和累计路程均达到要求)
    