%% 定义路网文件位置和名称
roadnetFileName = 'my_yizhuang_refName.net.xml';
roadnetFilePath = ['E:\我的文件\大学学习\_硕士学习\毕设硕士\车辆控制算法开发\SUMO与matlab联合仿真调试\' roadnetFileName];
mapName = strrep(roadnetFileName, '.net.xml', '');
DatafileName = ['ProcessedMap_' mapName '.mat'];
if exist(DatafileName,'file') == 2
    disp(['数据文件' DatafileName '已经存在,未生成新文件嘻嘻'])
else
%% 将XML文件读取为struct格式
disp(['正在读取文件' roadnetFileName '中，请稍安勿躁，通常需要几秒到几分钟~'])
tic;
map = parseXML(roadnetFilePath);
timeReadingFile = toc;
disp(['文件读取成功！CPU时间花费：' num2str(timeReadingFile) '秒'])
%% 构建实体字典和关系字典
disp('正在构建实体字典、关系字典中~')
tic;
[entity_dict,connection_dict,to_connection_dict] = StoreMapDictionary(map);
new_entity_dict = correctInternalLanesShape(entity_dict,connection_dict);
[lane_from_connection_dict, lane_to_connection_dict] = genLaneConnectionDict(connection_dict);
timeReadingFile = toc;
disp(['字典构建完成！CPU时间花费：' num2str(timeReadingFile) '秒']);
disp('正在生成节点邻接地图~')
proxyMat = genProxyMapFromMap(map,entity_dict);
disp('生成节点邻接地图完成！')
disp('正在构建车道指示箭头字典与邻接地图中~')
tic;
dirArrowMap = genDirectionArrowMap(entity_dict,connection_dict);
timeReadingFile = toc;
disp(['车道指示箭头字典与邻接地图构建完成！CPU时间花费：' num2str(timeReadingFile) '秒']);
disp('正在进行道路绘图，转换成用于小地图显示的数据格式中~')
tic
smallMap = saveMapImage_and_Scale(entity_dict,mapName);
timeReadingFile = toc;
disp(['小地图数据构建完成！CPU时间花费：' num2str(timeReadingFile) '秒']);
%% 保存处理好的路网数据
disp('正在保存处理好的地图数据！')
save(DatafileName,'map','connection_dict','entity_dict','proxyMat','dirArrowMap', ...
    'smallMap','new_entity_dict','to_connection_dict','lane_to_connection_dict','lane_from_connection_dict')
disp(['数据文件' DatafileName '保存成功~'])
end


%% 子函数库
function [entity_dict,connection_dict,to_connection_dict] = StoreMapDictionary(map)
    entity_dict = dictionary(); % 初始化实体字典
    connection_dict = dictionary(string([]),{}); % 初始化关系字典
    to_connection_dict = dictionary(string([]),{}); % 初始化to关系字典
    processFlag = false;
    childrenNum = length(map.Children);
    if childrenNum > 1000
        processFlag =true;
    end
    for i = 1:childrenNum
        theChild = map.Children(i);
       if strcmp(theChild.Name,'junction')
            id = getAttribute(theChild,'id'); 
            if ~strncmp(id,':',1) % 这里暂时省略掉了internal Junction
                junction = Junction_SUMO('id',id,...
                    'type',getAttribute(theChild,'type'), ...
                    'x',str2double(getAttribute(theChild,'x')), ...
                    'y',str2double(getAttribute(theChild,'y')), ...
                    'incLanes',split(getAttribute(theChild,'incLanes'), ' '), ...
                    'intLanes',split(getAttribute(theChild,'intLanes'), ' '), ...
                    'shape',parseStringToMatrix(getAttribute(theChild,'shape')) ...
                    );
                entity_dict(id) = {junction};
            end

       elseif strcmp(theChild.Name,'edge')
            id = getAttribute(theChild,'id'); 
            if strncmp(id, ':', 1) % 如果第一个字符是':'，那么是internalEdge
                laneNum = 0;
                for j = 1:length(theChild.Children)
                    if strcmp(theChild.Children(j).Name,'lane')
                        laneNum = laneNum + 1;
                        LaneChild = theChild.Children(j);
                        id0 = getAttribute(LaneChild,'id');
                        lane = Lane_SUMO('id',id0, ...
                            'index',round(str2double(getAttribute(LaneChild,'index'))), ...
                            'speed',str2double(getAttribute(LaneChild,'speed')), ...
                            'length',str2double(getAttribute(LaneChild,'length')), ...
                            'shape',parseStringToMatrix(getAttribute(LaneChild,'shape')));
                        entity_dict(id0) = {lane};
                    end
                end
                edge = Edge_SUMO('id',id,'laneNum',laneNum);

            else
                laneNum = 0;
                for j = 1:length(theChild.Children)
                    if strcmp(theChild.Children(j).Name,'lane')
                        laneNum = laneNum + 1;
                        LaneChild = theChild.Children(j);
                        id0 = getAttribute(LaneChild,'id');
                        lane = Lane_SUMO('id',id0, ...
                            'index',round(str2double(getAttribute(LaneChild,'index'))), ...
                            'speed',str2double(getAttribute(LaneChild,'speed')), ...
                            'length',str2double(getAttribute(LaneChild,'length')), ...
                            'shape',parseStringToMatrix(getAttribute(LaneChild,'shape')));
                        entity_dict(id0) = {lane};
                    end
                end
                edge = Edge_SUMO('id',id,'laneNum',laneNum,...
                    'from',getAttribute(theChild,'from'), ...
                    'to',getAttribute(theChild,'to'));
            end
            entity_dict(id) = {edge};

            

       elseif strcmp(theChild.Name,'connection')   
            id = getAttribute(theChild,'from');
            if ~isKey(connection_dict,id) % 如果以这个edge为起始的connection第一次见到
                tempContainner = cell(1,20);
                connect_from_edgeID = struct('connection_num',0,...
                    'connections',{tempContainner});
                connection_dict(id) = {connect_from_edgeID};
            end
              
            connection = Connection_SUMO('from',getAttribute(theChild,'from'), ...
                'to',getAttribute(theChild,'to'), ...
                'fromLane',round(str2double(getAttribute(theChild,'fromLane'))), ...
                'toLane',round(str2double(getAttribute(theChild,'toLane'))), ...
                'via',getAttribute(theChild,'via'), ...
                'dir',getAttribute(theChild,'dir'), ...
                'state',getAttribute(theChild,'state'));
            oldConnectNum = connection_dict{id}.connection_num;
            connection_dict{id}.connection_num = oldConnectNum + 1;
            connection_dict{id}.connections{oldConnectNum+1} = connection;

            id = getAttribute(theChild,'to');
            if ~isKey(to_connection_dict,id) % 如果以这个edge为终止的connection第一次见到
                tempContainner = cell(1,20);
                connect_from_edgeID = struct('connection_num',0,...
                    'connections',{tempContainner});
                to_connection_dict(id) = {connect_from_edgeID};
            end
              
            oldConnectNum = to_connection_dict{id}.connection_num;
            to_connection_dict{id}.connection_num = oldConnectNum + 1;
            to_connection_dict{id}.connections{oldConnectNum+1} = connection;


       end
       if processFlag && mod(i,500)==0
            disp(['字典提取处理进度：' num2str(i) '节点/' num2str(childrenNum) '节点'])
       end
    end
end


function new_entity_dict = correctInternalLanesShape(entity_dict,connection_dict)
    new_entity_dict = entity_dict;
    keys = new_entity_dict.keys;
    for i = 1:length(keys)
        key = keys{i};
        entity = new_entity_dict{key};
        if isa(entity,'Edge_SUMO')
            if ~strncmp(key,':',1)
                if isKey(connection_dict,key)
                    for j = 1:connection_dict{key}.connection_num
                        connection = connection_dict{key}.connections{j};
                        fromLaneID = [connection.from '_' num2str(connection.fromLane)];
                        viaLaneID = connection.via;
    
                        toLaneID = [connection.to '_' num2str(connection.toLane)];
                        viaLaneEntity = new_entity_dict{viaLaneID};
                        viaLaneEntity.shape = genIntLaneShape(new_entity_dict{fromLaneID}.shape,new_entity_dict{toLaneID}.shape);
                        dist = xy2dist(viaLaneEntity.shape);
                        viaLaneEntity.length = dist(end,:);
                        new_entity_dict{viaLaneID} = viaLaneEntity;
                    end
                end
            end
        end
    
    end
end


function smallMap = saveMapImage_and_Scale(entity_dict,mapName)
    fig = figure;
    hold on
    keys = entity_dict.keys;
    for i  = 1:length(keys)
        entity = entity_dict{keys(i)};
        
        if isa(entity,'Lane_SUMO')
            if ~strncmp(entity.id,':',1)
                shape = entity.shape;
                plot(shape(:,1),shape(:,2),"Color",[0.1 0.1 0.05])
            end
        elseif isa(entity,'Junction_SUMO')
            shape = entity.shape;
            fill(shape(:,1),shape(:,2),'r','FaceAlpha',0.3)
        end
           
    end
    hold off
    ax = gca;
    
    axis equal
    % axis auto
    axis off
    
    color = [0.6 0.65 0.65]*1.2;
    set(gca,'Color',color) % 
    set(fig,'Color',color) % 深蓝灰,更蓝
    set(fig, 'InvertHardcopy', 'off');
    imgFileName = [mapName '.jpg'];
    exportgraphics(ax,imgFileName,Resolution=600,BackgroundColor=color);
    xlimit = get(gca, 'XLim');
    ylimit = get(gca, 'YLim');
    close(fig);
    img = imread(imgFileName);
    % 获取图像大小
    [imgHeight, imgWidth, ~] = size(img);
    
    % 计算坐标到像素的转换比例
    yScale = imgWidth / (xlimit(2) - xlimit(1));
    xScale = imgHeight / (ylimit(2) - ylimit(1));
    smallMap = struct('img',img,'xlimit',xlimit,'ylimit',ylimit,'xScale',xScale,'yScale',yScale);
end
