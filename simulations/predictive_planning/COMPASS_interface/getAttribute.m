function value = getAttribute(body, attribute)
    value = [];
    for i = 1:length(body.Attributes)
        if strcmp(body.Attributes(i).Name,attribute)
            value = body.Attributes(i).Value;
            break;
        end
    end
end