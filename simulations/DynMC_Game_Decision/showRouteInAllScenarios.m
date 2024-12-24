figure
hold on
for i = 1:length(scenarioSet)
    route = scenarioSet{i}.getEgoRoute();
    plot3(route(:,1),route(:,2),i*ones(size(route(:,1))),'LineWidth',1.5)

end