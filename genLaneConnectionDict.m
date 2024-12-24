function [lane_from_connection_dict, lane_to_connection_dict] = genLaneConnectionDict(connection_dict)
    lane_from_connection_dict = dictionary(string([]),{});
    lane_to_connection_dict = dictionary(string([]),{});
    keys = connection_dict.keys;
    for i = 1:length(keys)
        key = keys{i};
        for j = 1:connection_dict{key}.connection_num
            connection = connection_dict{key}.connections{j};
            fromLaneID = connection.getFromLaneID;
            toLaneID = connection.getToLaneID;
            viaLaneID = connection.via;
            dir = connection.dir;
            state = connection.state;
            laneConnection = LaneConnection_SUMO('from',fromLaneID,'to',toLaneID, ...
                'via',viaLaneID,'dir',dir,'state',state);
            if ~isKey(lane_to_connection_dict,toLaneID)
                lane_to_connection_dict{toLaneID} = {laneConnection};
            else
                lane_to_connection_dict{toLaneID} = [lane_to_connection_dict{toLaneID};{laneConnection}];
            end
            if ~isKey(lane_from_connection_dict,fromLaneID)
                lane_from_connection_dict{fromLaneID} = {laneConnection};
            else
                lane_from_connection_dict{fromLaneID} = [lane_from_connection_dict{fromLaneID};{laneConnection}];
            end
            

        end
        
    end


end