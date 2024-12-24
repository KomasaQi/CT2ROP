function newDict = unionDictionary(varargin)
    for k = 1:length(varargin)
        if ~isa(varargin{k},'dictionary')
            error(['第' num2str(k) '个输入不是一个dictionary,主人请检查一下喵~'])
        end
    end
    newDict = varargin{1};
    for k = 2:length(varargin)
        if varargin{k}.numEntries > 0
            newDict(varargin{k}.keys) = varargin{k}.values;
        end
    end
end