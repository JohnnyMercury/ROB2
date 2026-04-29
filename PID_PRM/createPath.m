function path = createPath(start,goal,map)
% Find path using PRM
rng(0)

prm = mobileRobotPRM(map); % create prm planner

path = prm.findpath(start, goal); % query planner for path

%figure
hold on

prm.show()

end