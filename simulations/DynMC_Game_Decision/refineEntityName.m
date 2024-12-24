% 本程序用来简化SUMO路网文件中实体的名称
netFileName = 'my_yizhuang.net.xml';


netName = strrep(netFileName,'.net.xml','');
newNetName = [netName '_refName'];
refine_name_dict = dictionary(string([]),string([]));
netXmlDoc = xmlread(netFileName);
netXmlStr = xmlwrite(netXmlDoc);



keys = entity_dict.keys;
j_counter = 0;
e_counter = 0;
for i = 1:length(keys)
    key = keys{i};
    entity = entity_dict{key};
    if ~strncmp(key,':',1)
        if isa(entity,'Junction_SUMO')
            j_counter = j_counter + 1;
            new_j_id = ['J' dec2base(j_counter,36)];
            netXmlStr = strrep(netXmlStr, key, new_j_id);
            refine_name_dict(key) = new_j_id;
        elseif isa(entity,'Edge_SUMO')
            if ~strncmp(key,'-',1)
                e_counter = e_counter + 1;
                new_e_id = ['E' dec2base(e_counter,36)];
                netXmlStr = strrep(netXmlStr, key, new_e_id);
                refine_name_dict(key) = new_e_id;
            end

        end
    end

end
newXmlFile = [newNetName '.net.xml']; % 替换为新的XML文件名
fid = fopen(newXmlFile, 'w');
if fid == -1
    error('Cannot open file for writing.');
end
fprintf(fid, '%s', netXmlStr);
fclose(fid);





dicFileName = ['RefName_Dict_' netName '.mat'];
save(dicFileName,"refine_name_dict");


routeFileName = 'my_yizhuang_routes.rou.xml';
rouXmlDoc = xmlread(routeFileName);
rouXmlStr = xmlwrite(rouXmlDoc);

keys = refine_name_dict.keys;
for i = 1:length(keys)
    key = keys{i};
    rouXmlStr = strrep(rouXmlStr, key, refine_name_dict(key));
end

newRouXmlFile = [newNetName '.rou.xml']; % 替换为新的XML文件名
fid = fopen(newRouXmlFile, 'w');
if fid == -1
    error('Cannot open file for writing.');
end
fprintf(fid, '%s', rouXmlStr);
fclose(fid);







