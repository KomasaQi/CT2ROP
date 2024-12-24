% 将最优化函数直接生成代码
function [optParams,minCost]=fmincon_6order(tf,a,b,A_d,resolution,L,delta_lim,deltaRate_lim,ltr_zeta,ltr_omega_n) %#codegen

    options = optimoptions('fmincon','Algorithm','sqp');
    [optParams,minCost]=fmincon(@(params)lossfcn_6order(params,tf, ...
        a,b,A_d,resolution,L,delta_lim,deltaRate_lim, ...
        ltr_zeta,ltr_omega_n),[0 0],[],[],[],[],[],[],[],options);

end