function handle = plotGroundPlane(color,center,height)
    handle = fill3(center(1)+1e5*[1 -1 -1 1],center(2)+1e5*[1 1 -1 -1],height*ones(1,4),color);
end