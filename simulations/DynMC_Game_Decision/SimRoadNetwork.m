%{
    找到每条边所关注的周围所有边，形成字典edgeID ---> {edgeIDs}
    对{edgeIDs}当中的每条边，生成一个元组，就是不同的可能意图的下一/n个边{nextEdgeIDs}（及其限速）
    仿真当中，每辆车都会事先根据意图采样出一条edge路径{edge1,...,edgeN}
    车辆驶出最后的场景仿真edge后会置一个不可能到达的状态值
    判断前车、后车的方式：对每一条{edgeIDs}当中的每条边，生成2个元组,分别指示了
%}
classdef SimRoadNetwork < handle
    properties
        edgeList
        junctionList
        frindgeEdgeArray
        edge_No_dict = dictionary(string([]),[]);
       
        e_dict = dictionary(string([]),{});
        c_from_dict = dictionary(string([]),{});
        c_to_dict = dictionary(string([]),{});
        lc_from_dict = dictionary(string([]),{});
        lc_to_dict = dictionary(string([]),{});
        
        
    end
    methods
        function obj = SimRoadNetwork(varargin)
            for k = 1:2:length(varargin)
                if isprop(obj, varargin{k})
                    obj.(varargin{k}) = varargin{k+1};
                else
                    error('Property %s does not exist.', varargin{k});
                end
            end
        end
        function inNetFlag = isInNet(obj,edgeID)
            % 是否在路网内的判断，inNetFlag 
            % 0:不在路网内
            % 1:在路网内的正常边上
            % 2:在路网内的交叉口正常边上
            % 3:在路网内的交叉口二级边上(不在edgeList中)
            inNetFlag = 0;
            if strncmp(edgeID,':',1)
                lastUnderscoreIdx = find(edgeID=='_',1,'last');
                junctionID = edgeID(2:lastUnderscoreIdx-1);
                if isKey(obj.e_dict,junctionID)
                    if isKey(obj.e_dict,edgeID)
                        inNetFlag = 2;
                    else
                        inNetFlag = 3;
                    end
                end
            else
                if isKey(obj.e_dict,edgeID)
                    inNetFlag = 1;
                end
            end
        end
        
        % 这个快速判断就是直接判断实体是否在本路网中
        function inNetFlag = isInNet_fast(obj,entityID)
            inNetFlag = isKey(obj.e_dict,entityID);
        end
        function shape = getLaneShape(obj,laneID)
            shape = obj.e_dict{laneID}.shape;
        end

        function entity = getEntity(obj,entityID)
            entity = obj.e_dict{entityID};
        end
        function lc_from_connections = get_lc_from_connections(obj,laneID)
            lc_from_connections = obj.lc_from_dict{laneID};
        end
        % 可达性判断函数
        function flag = canReach_L2L(obj,fromLaneID,toLaneID)
            flag = false;
            % if isKey(obj.lc_to_dict,toLaneID)
                connections = obj.lc_to_dict{toLaneID};
                for i = 1:length(connections)
                    fromLaneID_true = connections{i}.from;
                    if strcmp(fromLaneID_true,fromLaneID)
                        flag = true;
                        break
                    end
                end
            % else
            %     error(['键' toLaneID '不在字典obj.lc_to_dict中呜呜，快检查一下！' ])
            % end

        end

        function flag = canReach_L2E(obj,fromLaneID,toEdgeID)
            flag = false;
            if isKey(obj.e_dict,toEdgeID)
                connections = obj.lc_from_dict{fromLaneID};
                for i = 1:length(connections)
                    toLaneID_true = connections{i}.to;
                    if strcmp(toLaneID_true(1:end-2),toEdgeID)
                        if strncmp(fromLaneID,':',1)
                            flag = true;
                            break
                        else
                            if isKey(obj.e_dict,connections{i}.via)
                                flag = true;
                                break
                            end
                        end
                    end
                end
            end
        end

        function flag = canReach_E2E(obj,fromEdgeID,toEdgeID)
            flag = false;
            if obj.isInNet_fast(toEdgeID)
                connections = obj.c_from_dict{fromEdgeID}.connections;
                for i = 1:obj.c_from_dict{fromEdgeID}.connection_num
                    toEdgeID_true = connections{i}.to;
                    if strcmp(toEdgeID_true,toEdgeID)
                        flag = true;
                        break
                    end
                end
            end
        end

        function flag = canReach_E2L(obj,fromEdgeID,toLaneID)
            flag = false;
            if obj.isInNet_fast(toLaneID)
                connections = obj.c_from_dict{fromEdgeID}.connections;
                for i = 1:obj.c_from_dict{fromEdgeID}.connection_num
                    toLaneID_true = connections{i}.getToLaneID;
                    if strcmp(toLaneID_true,toLaneID)
                        flag = true;
                        break
                    end
                end
            end
        end

        function dir = getDirE2E(obj,fromEdgeID,toEdgeID)
            dir = [];
            connections = obj.c_from_dict{fromEdgeID}.connections;
            for i = 1:obj.c_from_dict{fromEdgeID}.connection_num
                toEdgeID_true = connections{i}.to;
                if strcmp(toEdgeID_true,toEdgeID)
                    dir = connections{i}.dir;
                    break
                end
            end
            if isempty(dir)
                disp(['error:未找到fromEdgeID到toEdgeID的连接 ' fromEdgeID '  ---从场景字典中查找失败--->  ' toEdgeID])
                dir = connections{1}.dir;
            end
        end
    end
end