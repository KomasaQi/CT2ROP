%{
    根据地图生成一个交通规则的字典，将laneID映射到其可以行驶的标识,dict是用id查找箭头实体的，而NO_dict是用序号查找箭头实体的，配合xyMat实现地理位置方式查找
%}
function dirArrowMap = genDirectionArrowMap(entity_dict,connection_dict)
    
    ekeys = entity_dict.keys;
    dirArrowMap = struct('dict',dictionary(),'NO_dict',dictionary(),'xyMat',zeros(length(ekeys),2));
    dirArrowMap.dict('id') = {'capacity'};
    for i = 1:length(ekeys)
        id = ekeys{i};
        entity = entity_dict{id};
        if isa(entity,'Lane_SUMO')
            theLaneID = id;
            if ~strncmp(theLaneID,':',1)
                edgeID = entity.getEdgeID();
                if isKey(connection_dict,edgeID)
                    connectionNum = connection_dict{edgeID}.connection_num;
                    dirArrowMap.dict(theLaneID) = {cell(0)};
                    for j = 1:connectionNum
                        connection = connection_dict{edgeID}.connections{j};
                        laneID = entity_dict{edgeID}.getLaneID(connection.fromLane);
                        if strcmp(laneID,theLaneID)
                            if strncmpi(connection.dir,'r',1)
                                dir = 'r';
                            elseif strncmpi(connection.dir,'l',1)
                                dir = 'l';
                            elseif strncmpi(connection.dir,'s',1)
                                dir = 's';
                            elseif strncmpi(connection.dir,'t',1)
                                dir = 't';
                            end
                            dirArrowMap.dict(laneID) = {unique([{dir},dirArrowMap.dict{laneID}])};
                            
                        end
                    end
                end
            end
        end
    end
    dirArrowMap.dict('id') = [];
    rkey = dirArrowMap.dict.keys;
    dirArrowMap.xyMat(length(rkey)+1:end,:)=[];
    valid_arrow_num = 0;
    for i = 1:length(rkey)
        laneID = rkey(i);
        width = entity_dict{laneID}.width;
        shape = entity_dict{laneID}.shape;
        location = shape(end,:);
        % noneShape = [1,24;1,26;-1,26;-1,24];
        
        dist = norm(shape(1,:)-shape(end,:));
        if dist<1e-3
            heading = [];
        else
            heading_cos_sin = shape(end,:)-shape(end-1,:);
            heading = atan2(heading_cos_sin(2),heading_cos_sin(1))-pi/2;
        end
        dir = dirArrowMap.dict{laneID};
        dir_arrow = DirectionArrow('dir',dir);
        dir_arrow.location = location;
        
        
        if entity_dict{laneID}.length > 10
            if isequal(dir,{'l'})
                initArrowShape = [2,-25;2,8;-10,17;-10,24;-15,12;-11,0;-10.5,7;-2,0;-2,-25];
            elseif isequal(dir,{'r'})
                initArrowShape = [2,-25;2,8;-10,17;-10,24;-15,12;-11,0;-10.5,7;-2,0;-2,-25];
                initArrowShape(:,1) = -initArrowShape(:,1);
            elseif isequal(dir,{'s'})
                initArrowShape = [2,-25;2,6;5,6;0,25;-5,6;-2,6;-2,-25];
            elseif isequal(dir,{'t'})
                initArrowShape = [9,-20;9,17;8,19;5,21;3,22;-2,22;-4,21;-7,19;-8,17;-8,7;-12,7;-6.5,-10;-1,7;-4,7;-4,13;-1,16;2,16;5,13;5,-20];
            elseif isequal(dir,{'r','s'}) 
                initArrowShape = [2,-25;2,6;5,6;0,25;-5,6;-2,6;-2,-11;-10,-4;-10,3;-15,-10;-10,-20;-10,-14;-2,-21;-2,-25];
                initArrowShape(:,1) = -initArrowShape(:,1);
            elseif isequal(dir,{'l','s'}) 
                initArrowShape = [2,-25;2,6;5,6;0,25;-5,6;-2,6;-2,-11;-10,-4;-10,3;-15,-10;-10,-20;-10,-14;-2,-21;-2,-25];
            elseif isequal(dir,{'l','r'})
                initArrowShape = [2,-25;2,0;10.5,7;11,0;15,12;10,24;10,17;0,9.5;-10,17;-10,24;-15,12;-11,0;-10.5,7;-2,0;-2,-25];
            elseif isequal(dir,{'r','t'}) 
                heading = []; % 不符合交通规则，既能右转又能调头
            elseif isequal(dir,{'s','t'}) 
                initArrowShape = [7,-25;7,5;10,5;5,25;0,5;3,5;3,-3;0,-1;-2,-1;-5,-3;-7,-6;-7,-10;-10,-10;-5,-23;0,-10;-3,-10;-3,-7;-2,-5;0,-5;3,-7;3,-25];
            elseif isequal(dir,{'l','t'})
                initArrowShape = [7,-25;7,7;-5,17;-5,24;-10,11;-5,0;-5,6;3,0;3,-3;0,-1;-2,-1;-5,-3;-7,-6;-7,-10;-10,-10;-5,-23;0,-10;-3,-10;-3,-7;-2,-5;0,-5;3,-7;3,-25];
            elseif isequal(dir,{'l','r','s'})
                initArrowShape = [2,-25;2,-21;10,-14;10,-20;15,-10;10,3;10,-4;2,-11;2,6;5,6;0,25;-5,6;-2,6;-2,-11;-10,-4;-10,3;-15,-10;-10,-20;-10,-14;-2,-21;-2,-25];
            elseif isequal(dir,{'l','r','t'})
                tempShape = [7,-25;7,0;15,6;15,0;20,11;15,24;15,17;5,9;-5,17;-5,24;-10,11;-5,0;-5,6;3,0;3,-3;0,-1;-2,-1;-5,-3;-7,-6;-7,-10;-10,-10;-5,-23;0,-10;-3,-10;-3,-7;-2,-5;0,-5;3,-7;3,-25];
                initArrowShape = [tempShape(:,1)-5,tempShape(:,2)];
            elseif isequal(dir,{'l','s','t'})
                tempShape = [2,-25;2,10;5,10;0,25;-5,10;-2,10;-2,4;-10,9;-10,15;-15,4;-10,-5;-10,0;-2,-5;-2,-8;-5,-6;-7,-6;-10,-8;-12,-10;-12,-14;-15,-14;-10,-25;-5,-14;-8,-14;-8,-12;-7,-10;-5,-10;-2,-12;-2,-25];
                initArrowShape = [tempShape(:,1)+3,tempShape(:,2)];
            elseif isequal(dir,{'r','s','t'})
                initArrowShape = [2,-25;2,-5;10,0;10,-5;15,4;10,15;10,9;2,4;2,10;5,10;0,25;-5,10;-2,10;-2,-8;-5,-6;-7,-6;-10,-8;-12,-10;-12,-14;-15,-14;-10,-25;-5,-14;-8,-14;-8,-12;-7,-10;-5,-10;-2,-12;-2,-25];
            elseif isequal(dir,{'l','r','s','t'})
                initArrowShape = [2,-25;2,-5;10,0;10,-5;15,4;10,15;10,9;2,4;2,10;5,10;0,25;-5,10;-2,10;-2,4;-10,9;-10,15;-15,4;-10,-5;-10,0;-2,-5;-2,-8;-5,-6;-7,-6;-10,-8;-12,-10;-12,-14;-15,-14;-10,-25;-5,-14;-8,-14;-8,-12;-7,-10;-5,-10;-2,-12;-2,-25];
            elseif isempty(dir)
                heading = [];
            else
                error(['未知的方向表示 ' dir '键为' laneID])
            end
        else
            heading = [];
        end
        if ~isempty(heading)
            arrowShape = transformArrow(initArrowShape,width,heading,location);
            valid_arrow_num = valid_arrow_num + 1;
            dir_arrow.NO = valid_arrow_num;
            dir_arrow.shape = arrowShape;
            dirArrowMap.dict(laneID) = {dir_arrow};
            dirArrowMap.xyMat(valid_arrow_num,:) = location;
            dirArrowMap.NO_dict(valid_arrow_num) = {char(laneID)};
        else
            % arrowShape = transformArrow(noneShape,width,0,location);
            dirArrowMap.dict(laneID) = [];
        end

    end
    if valid_arrow_num < length(rkey)
        dirArrowMap.xyMat(valid_arrow_num+1:end,:)=[];
    end
end


function shape = transformArrow(initShape,width,heading,location)
    shape = initShape;
    shape(:,2) = shape(:,2)-30;
    shape = shape/34*width;
    shape = RotatePolygon(shape,location,heading);
end


function shape=RotatePolygon(shape0,location,th)
    center = [0 0];
    x = shape0(:,1);
    y = shape0(:,2);
    pointNum=length(x);
    if length(x)~=length(y)
        disp('顶点x,y坐标不一致，请检查一下')
    end
    x1=zeros(size(x));
    y1=zeros(size(y));
    for i=1:pointNum
        xc=x(i)-center(1);
        yc=y(i)-center(2);
        x1(i)=(xc*cos(th)-yc*sin(th))+center(1);
        y1(i)=(xc*sin(th)+yc*cos(th))+center(2);
    end
    shape = [x1+location(1),y1+location(2)];
end
