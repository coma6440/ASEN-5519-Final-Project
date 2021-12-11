function plot_goal(g)
p = cell2mat(g.position);
q = cell2mat(g.orientation);
s = cell2mat(g.size);
c = cuboid(s, p, q);
[k,~] = convhull(c(:,1),c(:,2),c(:,3),'Simplify',true);
trisurf(k,c(:,1),c(:,2),c(:,3),'FaceColor','green','FaceAlpha', 0.4)
end
