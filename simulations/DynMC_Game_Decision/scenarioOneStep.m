s = s.step();
for i = 1:s.vehNum
    delete(veh_text_handles{i});
    delete(veh_driveLine_handles{i});
    opsState = s.vehState(i,s.var.opsState);
    if opsState
        if opsState == 2
            veh_text_handles{i} = text(s.vehState(i,s.var.x),s.vehState(i,s.var.y),['No.' num2str(i) ', opsState=2']);
        else
            veh_text_handles{i} = text(s.vehState(i,s.var.x),s.vehState(i,s.var.y),['No.' num2str(i)]);
        end
        set(veh_dot_handles{i},"XData",s.vehState(i,s.var.x),"YData",s.vehState(i,s.var.y));
        driveLine = s.vehDriveLine{i};
        veh_driveLine_handles{i} = plot(driveLine(:,1),driveLine(:,2),'g','LineWidth',1.5);
    end
end

if ifOutGIF
    theFrame = getframe(gcf); %获取影片帧
    [I,map]=rgb2ind(theFrame.cdata,256);
    imwrite(I,map,gifName,'WriteMode','append','DelayTime',gifDelayTime) %添加到图像
end