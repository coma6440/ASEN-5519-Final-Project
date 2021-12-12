%% Housekeeping
close all
clear
clc
addpath(genpath('../lib/yaml'));
%% Load in files
p_type = "geometric";
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
    if not(isfolder(dir_name))
     mkdir(dir_name);
    end
    
    plot_position(A, dir_name + filesep + "position")
    plot_orientation(A, dir_name + filesep + "orientation")
    plot_trajectory(A, env, dir_name + filesep + "trajectory") 
    make_movie(A, env, dir_name + filesep + "anim.avi")
    make_movie_topdown(A, env, dir_name + filesep + "anim_topdown.avi")
    make_movie_side(A, env, dir_name + filesep + "anim_side.avi")
end

