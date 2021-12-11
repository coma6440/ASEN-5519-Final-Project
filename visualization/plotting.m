%% Housekeeping
close all
clear
clc
addpath(genpath('../lib/yaml'));
%% Load in files
p_type = "kinodynamic";
sol_folder = "../solutions/" + p_type;
sol_listing = dir(sol_folder);
sol_listing([sol_listing.isdir]) = [];

for i = 1:length(sol_listing)
    env = strsplit(sol_listing(i).name, "_");
    env = env{1}; % Get just the workspace
    env = ReadYaml(['../configs/', env ,'.yaml']);
    A = readmatrix(sol_folder + filesep + sol_listing(i).name, 'OutputType', 'double');
    name = strsplit(sol_listing(i).name,".");
    name = name{1};
    dir_name = p_type + filesep + name;
    mkdir(dir_name);
    
    plot_position(A, dir_name + filesep + "position")
    plot_orientation(A, dir_name + filesep + "orientation")
    plot_trajectory(A, env, dir_name + filesep + "trajectory") 
end

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

function plot_robot(s, p, q)
c = cuboid(s, p, q);
[k,~] = convhull(c(:,1),c(:,2),c(:,3),'Simplify',true);
trisurf(k,c(:,1),c(:,2),c(:,3),'FaceColor','blue','FaceAlpha', 0.4)
end

function plot_position(A, fname)
t = cumsum(A(:,end));
f = figure('Visible', 'Off');
tiledlayout(4,1);
nexttile
plot(t,A(:,1))
grid on
ylabel("X Position")

nexttile
plot(t,A(:,2))
grid on
ylabel("Y Position")

nexttile
plot(t,A(:,3))
grid on
ylabel("Z Position")

nexttile
plot(t,A(:,8))
grid on
ylabel("Speed")
xlabel("Time [s]")
saveas(f, fname + ".png");
saveas(f, fname + ".fig");
close
end

function plot_orientation(A, fname)
t = cumsum(A(:,end));
f = figure('Visible', 'Off');

tiledlayout(4,1)
nexttile
plot(t,A(:,4))
grid on
ylabel("q_x")

nexttile
plot(t,A(:,5))
grid on
ylabel("q_y")

nexttile
plot(t,A(:,6))
grid on
ylabel("q_z")

nexttile
plot(t,A(:,7))
grid on
xlabel("Time [s]")
ylabel("q_w")

saveas(f, fname + ".png");
saveas(f, fname + ".fig");
close
end

function plot_trajectory(A, env, fname)
% f = figure('Visible', 'Off');
f = figure();
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
plot_robot(cell2mat(env.robot.size), A(end,1:3), A(end, 4:7))
saveas(f, fname + ".png");
saveas(f, fname + ".fig");
close
end