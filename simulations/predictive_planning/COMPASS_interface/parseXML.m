function theStruct = parseXML(filename)
    % PARSEXML Convert XML file to a MATLAB structure.
    try
       tree = xmlread(filename);
    catch
       error('Failed to read XML file %s.',filename);
    end
    
    % Recurse over child nodes. This could run into problems 
    % with very deeply nested trees.
    try
       theStruct = parseChildNodes(tree);
    catch
       error('Unable to parse XML file %s.',filename);
    end
end

% ----- Local function PARSECHILDNODES -----
function children = parseChildNodes(theNode)
    % Recurse over node children.
    children = [];
    if theNode.hasChildNodes
       childNodes = theNode.getChildNodes;
       numChildNodes = childNodes.getLength;
        childCount = 0;
        for count = 1:numChildNodes
            theChild = childNodes.item(count-1);
            if ~strcmp(theChild.getNodeName, {'#text','#comment'})
                 childCount = childCount + 1;
            end
        end
        validChildrenNum = childCount;
       allocCell = cell(1, validChildrenNum);
    
       children = struct(             ...
          'Name', allocCell, 'Attributes', allocCell,    ...
          'Data', allocCell, 'Children', allocCell);
        childCount = 0;
        processFlag = false;
        if validChildrenNum > 1000 % 如果数量特别多，读取会费时间，需要各个进度条提醒
            processFlag = true;
        end
        for count = 1:numChildNodes
            theChild = childNodes.item(count-1);
            if ~strcmp(theChild.getNodeName, {'#text','#comment'})
                childCount = childCount + 1;
                children(childCount) = makeStructFromNode(theChild);
                if processFlag && mod(childCount,200)==0
                    disp(['结构体提取处理进度：' num2str(childCount) '节点/' num2str(validChildrenNum) '节点'])
                end
            end
        end
    end
end

% ----- Local function MAKESTRUCTFROMNODE -----
function nodeStruct = makeStructFromNode(theNode)
    % Create structure of node info.
    
    nodeStruct = struct(                        ...
       'Name', char(theNode.getNodeName),       ...
       'Attributes', parseAttributes(theNode),  ...
       'Data', '',                              ...
       'Children', parseChildNodes(theNode));
    
    if any(strcmp(methods(theNode), 'getData'))
       nodeStruct.Data = char(theNode.getData); 
    else
       nodeStruct.Data = '';
    end
end

% ----- Local function PARSEATTRIBUTES -----
function attributes = parseAttributes(theNode)
    % Create attributes structure.
    
    attributes = [];
    if theNode.hasAttributes
       theAttributes = theNode.getAttributes;
       numAttributes = theAttributes.getLength;
       allocCell = cell(1, numAttributes);
       attributes = struct('Name', allocCell, 'Value', ...
                           allocCell);
    
       for count = 1:numAttributes
          attrib = theAttributes.item(count-1);
          attributes(count).Name = char(attrib.getName);
          attributes(count).Value = char(attrib.getValue);
       end
    end
end
