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