classdef testClass < handle
    %UNTITLED 此处显示有关此类的摘要
    %   此处显示详细说明

    properties
        a = 2;
    end
    methods
        function obj = testClass()
            %UNTITLED 构造此类的实例
            %   此处显示详细说明
            
        end

        function outputArg = calc(obj)
            %METHOD1 此处显示有关此方法的摘要
            %   此处显示详细说明
            outputArg = obj.a^5;
            obj.a = obj.a +1;
        end
        
    end

end