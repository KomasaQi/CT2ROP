function impsblVehList = getImpsblVehList(allVehList,ego_pos,inreachableRadius)
    vehiclePos = zeros(length(allVehList),2);
    for i = 1:length(allVehList)
        checkID = allVehList{i};
        vehiclePos(i,:) = traci.vehicle.getPosition(checkID);
    end
    vehDists = sqrt((vehiclePos(:,1)-ego_pos(1)).^2+(vehiclePos(:,2)-ego_pos(2)).^2);
    impsblVehList = allVehList(vehDists>inreachableRadius);

end