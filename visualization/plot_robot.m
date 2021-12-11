function plot_robot(s, p, q)
c = cuboid(s, p, q);
[k,~] = convhull(c(:,1),c(:,2),c(:,3),'Simplify',true);
trisurf(k,c(:,1),c(:,2),c(:,3),'FaceColor','blue','FaceAlpha', 0.4)
end