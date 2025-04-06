%% *****************半挂式液罐车线性模型*******************************
% 模型： 5DOF半挂车+LDP线性双摆=7DOF模型
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
%                 th1,dth1,th2,dth2,f1,vx1,Y1,X1]';16×1 Nx=16
% 观测量：observe=[Y1,X1,f1,dth1,dth2,LTR]';6×1  Ny=6


function [A,B,C]=TrailorTruck_7DOF_LDP(state0, Ts, params)
    % global c_larger_delta c_smaller_Mz   % 提前定义好的2个系数,用来使得输入范围归一化
    c_larger_delta = 1; c_smaller_Mz = 1;
    % 初始状态定义
    % 定义动力学模型参数

    
    if nargin < 3 || isempty(params)
        params = [300, 300, 470, 18.7, 9.14, 168.3, 64.4, 197.7, 0.2314, 0.9630, 0.77];
        ParamsLDP = [1.5, 0.13, 0.77, 0.4763, 0.0, 3.5, 0.33];
    else
        if length(params) <= 11
            ParamsLDP = [1.5, 0.13, 0.77, 0.4763, 0.0, 3.5, 0.33];
        else
            ParamsLDP = params(12:end);
        end
    end


    %摆模型参数
    g = 9.806;
    exh = params(11); % 调节整体模型的高度
    ParamsLDP(1:2) = ParamsLDP(1:2) + exh;
    [h0,hp,lp,m0,mp,x0,dev,cd] = LDP_getParams(ParamsLDP); 
    x1 = x0 + dev; x2 = x0 - dev;
    %车辆模型参数
    L1=(5+6.27)/2;L2=(10.15+11.5)/2;       %轴距平均
    a=1.385;b=L1-a;c=5.635-a;e=5.5;d=L2-e; %纵向几何
    Tw1=(2.03+1.863*2)/3;Tw2=1.863;        %轮距平均
    hm1s=1.02; %牵引车质心高度
    hm2s=1.00; %50%充液率时半挂车质心高度
    hhitch=1.1;%交接点高度
    
    hroll1=params(9); hroll2=params(10);            %侧倾中心高度
    h1=hm1s-hroll1;h2=hm2s-hroll2;h1c=hhitch-hroll1;h2c=hhitch-hroll2;%高度几何
    m1s=6310;m1=m1s+570+785*2;             %牵引车质量
    % m2s=5925;m2=m2s+665*2;               %半挂车空载质量
    m2s=20387;m2=m2s+665*2;
    I1xx=6879;I1xz=130;I1zz=19665;         %牵引车惯量
    I2xx=9960;I2xz=0;I2zz=179992;        %半挂车惯量

    %轮胎模型参数
    k1=-1e4*params(1);k2=-1e4*params(2);k3=-1e4*params(3);             %侧偏刚度
    kr1=params(4)*1e5;kr2=params(5)*1e5;k12=params(6)*1e5; %侧倾刚度
    c1=params(7)*1e3;c2=params(8)*1e3;                    %悬架等效阻尼
    %车辆纵向动力学参数
    tau_v = 0.5;

  
    %% 定义线性动力学模型
% 状态量：state==[vy1,df1,F1,dF1,vy2,df2,F2,dF2,...
    %             th,dth,th2,dth2,f1,vx1,Y1,X1]';16 × 1, Nx = 16
    %input = [delta, M1zp, M1z_, M2zp, M2z_, vdes]'  6 x 1, Nu = 6
    v1x = state0(13); v2x = v1x;
    f1 = state0(14);
    %线性车辆动力学模型
    % M*dX = A0*X + B0*u
  
    m14 = -m1s*h1*c-I1xz;
    m21 = m1*v1x*h1c-m1s*h1*v1x;
    m24 = I1xx+2*m1s*h1^2-m1s*h1*h1c;
    m55 = m2*v2x*h2c-m2s*h2*v2x;
    m58 = I2xx+2*m2s*h2^2-m2s*h2*h2c;
    
    M=[m1*v1x*c, I1zz, 0,   m14,     0,        0,   0,     0;
        m21,    -I1xz, 0,   m24,     0,        0,   0,     0;
       m1*v1x,      0, 0, -m1s*h1, m2*v2x,     0,   0, -m2s*h2;
         0,         0, 0,     0,  m2*v2x*e, -I2zz,  0, I2xz-m2s*h2*e;
         0,         0, 0,     0,    m55,    -I2xz,  0,    m58;
         1,    -c/v1x, 0, -h1c/v1x,  -1,   -e/v2x,  0, h2c/v2x;
         0,         0, 1,     0,     0,        0,   0,     0;
         0,         0, 0,     0,     0,        0,   1,     0   ];
     
     a11 = (c+a)*k1 + (c-b)*k2;
     a12 = a*(c+a)*k1/v1x - b*(c-b)*k2/v1x - m1*v1x*c;
     a22 = (a*k1-b*k2)*h1c/v1x + (m1s*h1-m1*h1c)*v1x;
     a23 = m1s*g*h1 - kr1 -k12;
     a32 = (a*k1-b*k2)/v1x - m1*v1x;
     a36 = -d*k3/v2x - m2*v2x;
     a46 = -d*(e+d)*k3/v2x - m2*v2x*e;
     a56 = (m2s*h2-m2*h2c)*v2x - d*k3*h2c/v2x;
     a57 = m2s*g*h2 - kr2 -k12;
     %  X=   b1     df1    F1  dF1        b2  df2, F2   dF2 
     Acm0=[ a11,    a12,    0,   0,       0,   0,  0,    0;
       (k1+k2)*h1c, a22,  a23, -c1,       0,   0, k12,   0;
         k1+k2,     a32,    0,   0,      k3,  a36, 0,    0;
            0,       0,     0,   0, (e+d)*k3, a46, 0,    0;
            0,       0,    k12,  0,   k3*h2c, a56, a57, -c2;
            0,      -1,     0,   0,       0,   1,  0,    0;
            0,       0,     0,   1,       0,   0,  0,    0;
            0,       0,     0,   0,       0,   0,  0,    1   ];
     

     cM1 = 2/(Tw1*(m1 + m2 + m0 + mp));
     cM2 = 2/(Tw2*(m1 + m2 + m0 + mp));
     Bcm0=[-(c+a)*k1, -k1*h1c, -k1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
             1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, cM1;
            -1, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, cM1;
             0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, cM2;
             0, 0, 0,  1, 0, 0, 0, 0, 0, 0, 0, 0, cM2;
             0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 1/tau_v]';
     Bcm0(:,1) = Bcm0(:,1)/c_larger_delta;
     Bcm0(:,2:5) = Bcm0(:,2:5)/c_smaller_Mz;
       
     %       1    2   3    4    5    6    7    8   9   10   11  12 13
    %  X = vy1,  df1,F1, dF1, vy2, df2,  F2, dF2, th1 dth1 th2 dth2 vx
    % dX = dvy1;ddf1;dF1;ddF1;dvy2;ddf2;dF2;ddF2;dth1;ddth1;dth2;ddth2;ax ←M_
    M_ = zeros(13);
    
    M_(9,9) = 1;
    M_(10,:) = [0,0,0,0,v2x/lp,x1/lp,0,hp/lp-2,0,1,0,0,0];
    M_(11,11) = 1;
    M_(12,:) = [0,0,0,0,v2x/lp,x2/lp,0,hp/lp-2,0,0,0,1,0];
    M_(13,13) = 1;
    m310 = 1/2*mp*lp;
    mh0p = (m0*h0+mp*hp);
    M_(3,:) = [0,0,0,0,(mp+m0)*v2x,x0*(mp+m0),0,-mh0p,0,m310,0,m310,0];
    M_(4,:) = M_(3,:)*e + ...
    [0,0,0,0,-x0*(mp+m0)*v2x,-1/2*(mp+m0)*(x1^2+x2^2),0,x0*mh0p,0,-m310*x1,0,-m310*x2,0];
    m510 = -m310*hp;
    M_(5,:) = M_(3,:)*h2c+...
    [0,0,0,0,-mh0p*v2x,-x0*mh0p,0,(m0+h0^2+mp*hp^2),0,m510,0,m510,0];



    A_=zeros(13);

    A_(9,10) = 1;
    A_(10,:) = [0,0,0,0,0,-v2x/lp,-g/lp,0,-g/lp,-cd,0,0,0];
    A_(11,12) = 1;
    A_(12,:) = [0,0,0,0,0,-v2x/lp,-g/lp,0,-g/lp,0,0,-cd,0];
    A_(13,13) = -1/tau_v;
    A_(3,:) = [0,0,0,0,0,-(mp+m0)*v2x,0,0,0,0,0,0,0];
    A_(4,:) = A_(3,:)*e + x0*(mp + m0)*v2x;
    A_(5,:) = A_(3,:)*h2c+...
    [0,0,0,0,0,mh0p*v2x,mh0p*g,0,-m310*g,0,-m310*g,0,0];



    %       1    2   3    4    5    6    7    8   9  10
    %  X =  0,   0,  0,   0,   0, df2,   0,   0, th  0
    % dX =  0;   0;  0;   0;dvy2;   0;dF2;ddF2;dth;  0
     Acm = ([M,zeros(8,5);zeros(5,13)]+M_)\([Acm0,zeros(8,5);zeros(5,13)]+A_);
     Bcm = ([M,zeros(8,5);zeros(5,13)]+M_)\Bcm0;
     
     %% 增广状态量 并转变bi为vyi
     Trans = diag([v1x,1,1,1,v2x,1,1,1,1,1,1,1,1]);
     Ac = [Trans*Acm/Trans,zeros(13,3);            %增广状态量后的矩阵
          0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
         cos(f1),0,0,0,0,0,0,0,0,0,0,0,sin(f1),0,0,0;
        -sin(f1),0,0,0,0,0,0,0,0,0,0,0,cos(f1),0,0,0];

     Bc=[Trans*Bcm;zeros(3,6)] ;
     %% 离散化线性模型
     [A,B]=c2d_zoh(Ac,Bc,Ts);             %离散化线性模型
     
    %       1   2    3    4   5    6    7    8    9  10   11  12  13  14 15 16
    % dX=[dvy1;ddf1;dF1;ddF1;dvy2;ddf2;dF2;ddF2;df1;df2;dvx1;dvx2;dY1;dX2;dY2;dX2];
    %维度：14×1 
    
    %% 观测量计算
    %观测量为 Y1 X1 f1 th dth LTR
    C=[zeros(2,14),eye(2); % Y1 X1
             zeros(1,13),1,zeros(1,2);      % f1
             zeros(1,9),1,zeros(1,6);  % dth1
             zeros(1,11),1,zeros(1,4);  % dth2
      2/(mean([Tw1,Tw2])*(m1+m2)*g)*...
                 [0,0,-kr1,-c1,0,0,-kr2,-c2],zeros(1,8)];%LTR

end
%% 根据变量获取参数
function [h0,hp,lp,m0,mp,x0,dev,cd] = LDP_getParams(Params)
    m = 14462; % TotalWeight
    h0 = Params(1);
    hp = Params(2);
    lp = Params(3);
    m0 = m*Params(4);
    mp = m - m0;
    x0 = Params(5);
    dev = Params(6);
    cd = Params(7);
end