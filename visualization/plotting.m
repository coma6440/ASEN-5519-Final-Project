%% Housekeeping 
close all
clear
clc
addpath(genpath('../lib/yaml'));
%% Load in files
sol_folder = "../solutions/kinodynamic";
ws_folder = "../configs";
sol_listing = dir(sol_folder);
ws_listing = dir(ws_folder);
sol_listing([sol_listing.isdir]) = [];
ws_listing([ws_listing.isdir]) = [];
ws_listing(~contains({ws_listing.name},"yaml")) = [];
for i = 1:length(sol_listing)
    A = readmatrix(sol_folder + filesep + sol_listing(i).name, 'OutputType', 'double');
end

figure
plot3(A(:,1), A(:,2), A(:,3))
grid on
xlim([0,15])
ylim([-10,10])
zlim([-10,10])
xlabel("X Position")
ylabel("Y Position")
zlabel("Z Position")