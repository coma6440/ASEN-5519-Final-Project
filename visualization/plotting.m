%% Housekeeping 
close all
clear
clc
addpath(genpath('../lib/yaml'));
%% Load in files
sol_folder = "../solutions/kinodynamic";
sol_listing = dir(sol_folder);
sol_listing([sol_listing.isdir]) = [];

for i = 1:length(sol_listing)
    env = strsplit(sol_listing(i).name, "_");
    env = env{1}; % Get just the workspace
    env = ReadYaml(['../configs/', env ,'.yaml']);
    A = readmatrix(sol_folder + filesep + sol_listing(i).name, 'OutputType', 'double');
end

figure
plot3(A(:,1), A(:,2), A(:,3),'k', 'LineWidth', 2)
grid on
xlim([0,15])
ylim([-10,10])
zlim([-10,10])
xlabel("X Position")
ylabel("Y Position")
zlabel("Z Position")

hold on
plot_obstacles(env.obstacles)
plot_goal(env.goal)

%% Helper Functions
function c = cuboid(size, position, orientation)
c =  [-size(1)/ 2, -size(2) / 2, -size(3) / 2;
    -size(1) / 2, -size(2) / 2, size(3) / 2;
    -size(1) / 2, +size(2) / 2, -size(3) / 2;
    -size(1) / 2, +size(2) / 2, size(3) / 2;
    size(1) / 2, -size(2) / 2, -size(3) / 2;
    size(1) / 2, -size(2) / 2, size(3) / 2;
    size(1) / 2, +size(2) / 2, -size(3) / 2;
    size(1) / 2, +size(2) / 2, size(3) / 2];
for i = 1:length(c)
    c(i,:) = quatrotate([orientation(end), orientation(1:3)],c(i,:)) + position;
end
end

function plot_obstacles(obs_struct)
obs = fieldnames(obs_struct);
for i = 1:length(obs)
    o = obs_struct.(obs{i});
    p = cell2mat(o.position);
    q = cell2mat(o.orientation);
    s = cell2mat(o.size);
    c = cuboid(s, p, q);
    [k,~] = convhull(c(:,1),c(:,2),c(:,3),'Simplify',true);
    trisurf(k,c(:,1),c(:,2),c(:,3),'FaceColor','red', 'FaceAlpha', 0.4)
end
end

function plot_goal(g)
    p = cell2mat(g.position);
    q = cell2mat(g.orientation);
    s = cell2mat(g.size);
    c = cuboid(s, p, q);
    [k,~] = convhull(c(:,1),c(:,2),c(:,3),'Simplify',true);
    trisurf(k,c(:,1),c(:,2),c(:,3),'FaceColor','green','FaceAlpha', 0.4)
end