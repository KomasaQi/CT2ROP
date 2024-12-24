%{
    用于生成junction的内部车道形状 
%}
function shape = genIntLaneShape(lane1shape,lane2shape)
    proLen = 50;

    dir1 = lane1shape(end,:)-lane1shape(end-1,:);
    dir1 = dir1/norm(dir1)*proLen;
    dir2 = lane2shape(2,:)-lane2shape(1,:);
    dir2 = dir2/norm(dir2)*proLen;

    k1 = interp1([0,10,20,30],[0.6 0.5 0.2 0.25],norm(lane1shape(end,:)-lane2shape(1,:)),"linear","extrap");
    k3 = interp1([0,10,20,30],[0.6 0.5 0.2 0.25],norm(lane1shape(end,:)-lane2shape(1,:)),"linear","extrap");
    k2 = 1-k1;
    k4 = 1-k3;

    x3 = lane1shape(end,1);
    x1 = x3 - dir1(1);
    x2 = k1*x3+k2*x1;
    x4 = lane2shape(1,1);
    x6 = x4 + dir2(1);
    x5 = k3*x4+k4*x6;
    y3 = lane1shape(end,2);
    y1 = y3 - dir1(2);
    y2 = k1*y3+k2*y1;
    y4 = lane2shape(1,2);
    y6 = y4 + dir2(2);
    y5 = k3*y4+k4*y6;
    shape = [makima(1:6,[x1,x2,x3,x4,x5,x6],(3:0.25:4)'),makima(1:6,[y1,y2,y3,y4,y5,y6],(3:0.25:4)')];
end