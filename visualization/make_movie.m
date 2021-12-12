function make_movie(A, env, fname)
v = VideoWriter(fname,'MPEG-4');
open(v);
f = figure();
p = plot3(A(1,1), A(1,2), A(1,3),'k','LineWidth',2);
grid on
hold on
plot_goal(env.goal)
r = plot_robot(cell2mat(env.robot.size), A(1,1:3), A(1, 4:7));
plot_obstacles(env.obstacles)
xlim([0,15])
ylim([-10,10])
zlim([-10,10])
xlabel("X Position")
ylabel("Y Position")
zlabel("Z Position")
for i = 1:length(A)
    p.XData = A(1:i,1);
    p.YData = A(1:i,2);
    p.ZData = A(1:i,3);
    hold on
    delete(r)
    r = plot_robot(cell2mat(env.robot.size), A(i,1:3), A(i, 4:7));
    frame = getframe(f);
    writeVideo(v,frame);
end
close(v);
close(f);
end