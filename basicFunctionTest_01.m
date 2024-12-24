% javaaddpath('D:\Program Files (x86)\Eclipse\Sumo\tools\contributed\traci4matlab-master\traci4matlab.jar')


% SimulateOneRout
% 启动SUMO仿真
sumoBinary = 'sumo-gui'; % 或者使用'sumo'进行无GUI仿真
sumoCmd = [sumoBinary ' -c "E:\SUMO_FILES\YizhuangSim.sumocfg"' ' --start'];
traci.start(sumoCmd);


% 设置SUMO GUI的视角和跟踪车辆
% 设置SUMO GUI的显式模式为'real world'
viewID = 'View #0'; % 默认视角ID
schema = 'real world';
traci.gui.setSchema(viewID, schema);

% 设置放大倍数
zoomLevel = 10000; % 根据需要调整放大倍数
traci.gui.setZoom(viewID, zoomLevel);

traci.simulationStep(); % 进行一步仿真
vehicleIDs = traci.vehicle.getIDList(); % 获取所有车辆ID
% 聚焦到指定车辆
traci.gui.trackVehicle(viewID, 't_0');

% 开始仿真
step = 0;
while step < 1000
    traci.simulationStep(); % 进行一步仿真
    vehicleIDs = traci.vehicle.getIDList(); % 获取所有车辆ID

    for i = 1:length(vehicleIDs)
        id = vehicleIDs{i};
        
        % 获取车辆位置
        position = traci.vehicle.getPosition(id);
        x = position(1); % X坐标
        y = position(2); % Y坐标
        
        % 获取车辆所在车道ID
        laneID = traci.vehicle.getLaneID(id);
        
        % 获取车辆所在边ID
        edgeID = traci.vehicle.getRoadID(id);    
        
        % 输出车辆信息
        disp(['Vehicle ' id ' is at position (' num2str(x) ',' num2str(y) '), lane ' laneID ', edge ' edgeID]);
    end
    step = step + 1;
end

% 结束仿真
traci.close();


