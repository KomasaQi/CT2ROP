% 曲线约束参数
tf = 4;
x0 = 0;
y0 = 0;
v0x = 20;
v0y = 0;
a0x = 0;
a0y = 0;
xf = 80;
yf = 3.5;
vfx = 22;
vfy = 0;
afx = 0;
afy = 0;
% 待控制的自由度系数变量
a6 = -0.02;
b6 = -0.03;

% 根据约束求解其他系数
c1x = 1/(tf^2)*((xf-x0) - v0x*tf - 1/2*a0x*tf^2);
c2x = 1/tf*((vfx-v0x)-a0x*tf);
c3x = (afx - a0x);
cx = [c1x c2x c3x]';

c1y = 1/(tf^2)*((yf-y0) - v0y*tf - 1/2*a0y*tf^2);
c2y = 1/tf*((vfy-v0y)-a0y*tf);
c3y = (afy - a0y);
cy = [c1y c2y c3y]';

d = [1;6;30]*tf^4;
tfs = [tf tf^2 tf^3];

A = [1 1 1; 3 4 5; 6 12 20].*tfs;
invA = 1./tfs'.*[10 -4 0.5;-15 7 -1;6 -3 0.5];

b0 = y0;
b1 = v0y;
b2 = 1/2*a0y;
b = [b0;b1;b2;A\cy-A\d*b6;b6];

a0 = x0;
a1 = v0x;
a2 = 1/2*a0x;
a = [a0;a1;a2;A\cx-A\d*a6;a6];

% 生成曲线
t = 0:0.1:tf;
x = polyval(a(end:-1:1),t);
y = polyval(b(end:-1:1),t);
figure(1)
hold on
plot(x,y)


