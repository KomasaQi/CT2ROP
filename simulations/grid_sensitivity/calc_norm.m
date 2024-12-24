function norm_val = calc_norm(time,col,data1,data2,number)
data1_ = zeros(length(time),length(col));
data2_ = zeros(length(time),length(col));
for i=1:length(col)
    data1_(:,i)=interp1(data1(:,1),data1(:,col(i)),time');
    data2_(:,i)=interp1(data2(:,1),data2(:,col(i)),time');
end
if number == 1
norm_val = (mean((abs(data1_-data2_)./abs(data1_))'));
elseif number == 2
norm_val = mean(mean((abs(data1_-data2_)./abs(data1_))));
end
end