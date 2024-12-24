% 用全局MOBIL得到所有车的换道决策
%{
MOBIL模型 Kesting等人于2007年提出（最小化由换道行为引发的总体减速模型）
minimizing overall braking induced by lane changes
该模型用车辆加速度值表征驾驶者所得的驾驶利益，通过比较换道实施前后，当前车道与目标
车道上受影响车辆的整体利益变化来判断是否进行换道，并针对对称和非对称换道规则其除了相应的
换道模型。
MOBIL考察3辆车利益，本车，原车道后车，目标车道的新后车。换道需求建模：加速度
驾驶人对车道的选择本质上是看其在那条车道上能获得最大加速度。
另车辆ego选择第k条车道的效用值为U_ego_k,则有U_ego_k = a_ego_k
 
换道安全准则
目标车道后车加速度约束：间隙约束的间接描述
_hat表示换道后的值

(a_ego_hat - a_ego) + p*((a_hat_fhat - a_fhat)+(a_hat_f - a_f)) > delta_a + a_bias

p为礼让系数，p=0时只考虑自身利益，p=1时平等考虑自身和他人利益
detla_a为一个阈值，a_bias反应不对称换道情况。当交通规则倾向于驾驶人靠右侧车道行驶时
从左侧车道向右侧换道的门槛较低a_bias<0,反之a_bias>0。当左右换道具有同等权利时，a_bias=0

对于换道后新的跟随车辆，换道行为需要满足如下安全条件：
a_hat_fhat = f(v_fhat,v_ego,s_hat_fhat) > -b_safe
其中f是跟驰模型
%}
function [changingLanes, targetLaneIdxs, accs] = globalMOBIL(obj)
    vehicleState = obj.vehState;
    changingLanes = vehicleState(:,obj.var_changingLane); % 继承之前的状态
    targetLaneIdxs = vehicleState(:,obj.var_targetLaneIdx); % 继承继承！
    laneIdxDevs = vehicleState(:,obj.var_laneIdxDev);
    opsStates = vehicleState(:,obj.var_opsState);
    noLC_accs = zeros(obj.vehNum,1); % 用0初始化
    spds = vehicleState(:,obj.var_spd);
    canReachNextEdges = obj.vehState(:,obj.var_canReachNextEdge);
    dirs = vehicleState(:,obj.var_dir);
    remDists = vehicleState(:,obj.var_remDist);

    % spdDesList = obj.getAllSpdDes(); % 获取所有车的期望速度
    spdDesList = vehicleState(:,obj.var_desSpd);

    accs = zeros(obj.vehNum,1); % 加速度
    surroundVehMat0 = obj.surroundVeh;
    surrVehDistMat0 = obj.surrVehDist;
    surrVehSpdMat0 = obj.surrVehSpd;

    biases = zeros(obj.vehNum,3); % 换道的偏置系数，对每辆车的都是[L K R]
    biases(:,2) = 3; % 参数，车道保持的偏置系数 13 → 3 
    utilities = zeros(obj.vehNum,3); % 换道加速度收益
    politeness = obj.politeness; % 礼貌系数

    for vNo = 1:obj.vehNum
        if opsStates(vNo)  % 如果该车没有结束
            % front_veh_No = surroundVehList_mat(bidx,2); % 前车序号
            front_veh_Spd = surrVehSpdMat0(2*vNo-1,2); % 前车速度
            front_veh_dist = surrVehDistMat0(2*vNo-1,2); % 前车距离
            % 首先对所有车辆进行一次不换道时的加速度计算与存储
            noLC_accs(vNo) = obj.vehicles{vNo}.idm(spds(vNo),front_veh_Spd,front_veh_dist,spdDesList(vNo));
        end
    end
    
    % 下面对每辆车进行MOBIL,第一步，是计算
    for vNo = 1:obj.vehNum
        if opsStates(vNo)  % 如果该车没有结束
            
            % 最开始先判断一下是不是当前无法换道
            left_front_veh_No = surroundVehMat0(2*vNo-1,1);
            right_front_veh_No = surroundVehMat0(2*vNo-1,3);
            can_LLC = left_front_veh_No ~= 999;
            can_RLC = right_front_veh_No ~= 999;
            if ~isempty(obj.egoTargetLaneIdx) && vNo == 1 % 如果是自车
                targetLaneIdxs(vNo) = obj.egoTargetLaneIdx;
                changingLanes(vNo) = 0;  % 这个地方直接默认自车换道行为不体现在这个参数上
            else
                % 顺便设置一下偏置系数
                if canReachNextEdges(vNo) % 如果本车道能到下一个edge，就不倾向于换道
                    biases(vNo,2) = biases(vNo,2) + 2; % 同样，这个参数需要调节 5 → 2
                else % 如果本车道到不了下一个edge，就根据意图来判断
                    switch dirs(vNo)
                        case {-1,2} % 停车，右转
                            biases(vNo,1) = biases(vNo,1) + 1/remDists(vNo) - 3;
                            biases(vNo,3) = biases(vNo,3) + 1/remDists(vNo) + 3;
                        case {0, 1} % 调头，左转
                            biases(vNo,1) = biases(vNo,1) + 1/remDists(vNo) + 3;
                            biases(vNo,3) = biases(vNo,3) + 1/remDists(vNo) - 3;
                        case 3 % 直行
                            if laneIdxDevs(vNo) < 1 
                                biases(vNo,1) = biases(vNo,1) + 1/remDists(vNo) + 3;
                                biases(vNo,3) = biases(vNo,3) + 1/remDists(vNo) - 3;
                            else
                                biases(vNo,1) = biases(vNo,1) + 1/remDists(vNo) - 3;
                                biases(vNo,3) = biases(vNo,3) + 1/remDists(vNo) + 3;
                            end
                    end
                end
    
                % front_veh_No = surroundVehList_mat(bidx,2); % 前车序号
                if changingLanes(vNo) % 如果这辆车现在正处在换道的情况之中，需要判断当前情况是否危险，是否要撤销换道,以及是否结束换道了
                    % 正在换道，说明targetLaneIdx和round(laneIdxDev不相同)
                    if abs(laneIdxDevs(vNo) - targetLaneIdxs(vNo)) < 0.3 % 这个地方可以纠结一下数值，判断是否完成了一次换道
                        changingLanes(vNo) = 0;
                    else
                        switch changingLanes(vNo) % 查看是否要切换成0 
                            case 1 % 正在向左换道
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 暂时这里没写，应该写撤销换道的内容
    
                            case -1 % 正在向右换道
                                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 暂时这里没写，应该写撤销换道的内容
                            otherwise
                                error(['车No.' num2str(vNo) '的changingLane状态是' num2str(changingLanes(vNo)) '，是未知状态！'])
                        end
                    end
                    
                else % 如果这辆车现在没有在换道，就需要判断：首先，当前是否可以换道
                    u_f = -999; % 初始化
                    if ~can_LLC % 如果不能向左换道
                        utilities(vNo,1) = -inf;
                    else % 计算向左换道的收益
                        % (a_ego_hat - a_ego) + p*((a_hat_fhat - a_fhat)+(a_hat_f - a_f)) > delta_a + a_bias
                        a_ego = noLC_accs(vNo);
                        a_ego_hat = obj.vehicles{vNo}.idm(spds(vNo),surrVehSpdMat0(2*vNo-1,1),surrVehDistMat0(2*vNo-1,1),spdDesList(vNo));
                        u_ego = a_ego_hat - a_ego;
                        veh_f_No = surroundVehMat0(2*vNo,2);
                        if veh_f_No % 如果存在后车
                            a_f = noLC_accs(veh_f_No);
                            a_hat_f = obj.vehicles{veh_f_No}.idm(spds(veh_f_No),surrVehSpdMat0(2*vNo-1,2), ...
                                surrVehDistMat0(2*vNo,2)+surrVehDistMat0(2*vNo-1,2),spdDesList(veh_f_No));
                            u_f = a_hat_f - a_f;
                        else
                            u_f = 0;
                        end
                        veh_hatf_No = surroundVehMat0(2*vNo,1);
                        if veh_hatf_No
                            a_hatf = noLC_accs(veh_hatf_No);
                            a_hat_hatf = obj.vehicles{veh_hatf_No}.idm(spds(veh_hatf_No),spds(vNo), ...
                                surrVehDistMat0(2*vNo,1),spdDesList(veh_hatf_No));
                            u_hatf = a_hat_hatf - a_hatf;
                        else
                            u_hatf = 0;
                        end
                        utilities(vNo,1) = u_ego + politeness(vNo)*(u_hatf + u_f);
                    end
    
                    if ~can_RLC % 如果不能向右换道
                        utilities(vNo,3) = -inf;
                    else % 计算向右换道的收益
                        % (a_ego_hat - a_ego) + p*((a_hat_fhat - a_fhat)+(a_hat_f - a_f)) > delta_a + a_bias
                        a_ego = noLC_accs(vNo);
                        a_ego_hat = obj.vehicles{vNo}.idm(spds(vNo),surrVehSpdMat0(2*vNo-1,3),surrVehDistMat0(2*vNo-1,3),spdDesList(vNo));
                        u_ego = a_ego_hat - a_ego;
                        veh_f_No = surroundVehMat0(2*vNo,2);
                        if u_f ~= -999 % 如果计算过一遍，就不要再计算啦
                            if veh_f_No % 如果存在后车
                                a_f = noLC_accs(veh_f_No);
                                a_hat_f = obj.vehicles{veh_f_No}.idm(spds(veh_f_No),surrVehSpdMat0(2*vNo-1,2), ...
                                    surrVehDistMat0(2*vNo,2)+surrVehDistMat0(2*vNo-1,2),spdDesList(veh_f_No));
                                u_f = a_hat_f - a_f;
                            else
                                u_f = 0;
                            end
                        end
                        veh_hatf_No = surroundVehMat0(2*vNo,3);
                        if veh_hatf_No
                            a_hatf = noLC_accs(veh_hatf_No);
                            a_hat_hatf = obj.vehicles{veh_hatf_No}.idm(spds(veh_hatf_No),spds(vNo), ...
                                surrVehDistMat0(2*vNo,3),spdDesList(veh_hatf_No));
                            u_hatf = a_hat_hatf - a_hatf;
                        else
                            u_hatf = 0;
                        end
                        utilities(vNo,3) = u_ego + politeness(vNo)*(u_hatf + u_f);
                    end
                    changingLanes(vNo) = getLCDecision(utilities(vNo,:)+biases(vNo,:));
                    targetLaneIdxs(vNo) = round(laneIdxDevs(vNo)) + changingLanes(vNo);
                end
            end
            accs(vNo) = noLC_accs(vNo);
        end
    end

end

function LCDecision = getLCDecision(baised_utilities) % 根据收益获取换道决策baised_utilities = [pL,pK,pR]
    [~,idx] = max(baised_utilities);
    LCDecision = 2 - idx;

end