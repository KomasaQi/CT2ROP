function setAxisRange(center,boxWidth)
xlim([center(1)-boxWidth,center(1)+boxWidth]);
ylim([center(2)-boxWidth,center(2)+boxWidth]);
zlim([-boxWidth boxWidth]*0.3);

end