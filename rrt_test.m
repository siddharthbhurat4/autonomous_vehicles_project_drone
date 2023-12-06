clear;
close all;
get_map
omap = map3D;
ss = stateSpaceSE3([0 5;0 5;0 5;Inf Inf;Inf Inf;Inf Inf;Inf Inf]);

sv = validatorOccupancyMap3D(ss, ...
     Map = omap, ...
     ValidationDistance = 0.2);

planner = plannerRRT(ss,sv, ...
          MaxConnectionDistance = .4,...
          MaxIterations = 8000, ...
          MaxNumTreeNodes = 6000, ...
          GoalReachedFcn = @(~,s,g)(norm(s(1:3)-g(1:3))<.1), ...
          GoalBias = 0.3);

start = [0 0 0 0 0 0 1];
goal = [2 2 2 0 0 0 1];

[pthObj,solnInfo] = plan(planner,start,goal);

show(omap)

% view([5 45])
hold on
% Start state
scatter3(start(1,1),start(1,2),start(1,3),"g","filled")
% Goal state
scatter3(goal(1,1),goal(1,2),goal(1,3),"r","filled")
% Path
plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3), ...
      "r-",LineWidth=2)
plot3(solnInfo.TreeData(:,1),solnInfo.TreeData(:,2),solnInfo.TreeData(:,3),'.-')
disp(solnInfo.IsPathFound)