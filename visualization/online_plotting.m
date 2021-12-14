%% Housekeeping
close all
clear
clc
addpath(genpath('../lib/yaml'));
%% Load in files
sol_folder = "../solutions/online/w2_1";
env_name = 'w2';
sol_listing = dir(sol_folder);
sol_listing([sol_listing.isdir]) = [];
env = ReadYaml(['../configs/', env_name ,'.yaml']);
seg_listing = sol_listing(contains({sol_listing.name}, '_c_'));
rem_listing = sol_listing(contains({sol_listing.name}, '_r_'));
ini_idx = find(contains({sol_listing.name}, 'init'));
opt_cost_idx = find(contains({sol_listing.name}, 'opt'));
seg_cost_idx = find(contains({sol_listing.name}, 'segment'));


init_soln = readmatrix(sol_folder + filesep + sol_listing(ini_idx).name, 'OutputType', 'double');
t_init = cumsum(init_soln(:,end));

f = figure();
tiledlayout(3,1)
ax1 = nexttile;
hold on
grid on
plot(t_init, init_soln(:,1),'LineWidth',2)
ylabel("X Position")
ax2 = nexttile;
hold on
grid on
plot(t_init, init_soln(:,2),'LineWidth',2)
ylabel("Y Position")
ax3 = nexttile;
hold on
grid on
plot(t_init, init_soln(:,3),'LineWidth',2)
ylabel("Z Position")
xlabel("Time [s]")
t_final = 0;
for i = 1:length(rem_listing)
    A = readmatrix(sol_folder + filesep + seg_listing(i).name, 'OutputType', 'double');
    t_final = t_final + sum(A(:,end));
    B = readmatrix(sol_folder + filesep + rem_listing(i).name, 'OutputType', 'double');
    t = t_final + cumsum(B(:,end));
    plot(ax1, t, B(:,1),'LineWidth',2)
    xline(ax1, t_final, '--k')
    plot(ax2, t, B(:,2),'LineWidth',2)
    xline(ax2, t_final, '--k')
    plot(ax3, t, B(:,3),'LineWidth',2)
    xline(ax3, t_final, '--k')
end

saveas(f, sol_folder + filesep + env_name + "_opt_pos.png")

opt_cost = readmatrix(sol_folder + filesep + sol_listing(opt_cost_idx).name, 'OutputType', 'double');
seg_cost = readmatrix(sol_folder + filesep + sol_listing(seg_cost_idx).name, 'OutputType', 'double');
initial_cost = opt_cost(1,2) + seg_cost(1,2);
cost_diff = initial_cost - [0;cumsum(opt_cost(:,2) - opt_cost(:,3))];

f = figure();
plot(1:length(cost_diff), cost_diff, '-o')
grid on
hold on
ylabel("Cost")
plot(1:length(seg_cost), cumsum(seg_cost(:,2)), '-o')
yline(initial_cost, '--k')
legend(["Total Solution Cost", "Cumulative Executed Cost", "Initial Solution Cost"])
xlabel("Path Segment")
saveas(f, sol_folder + filesep + env_name + "_opt_cost.png")
