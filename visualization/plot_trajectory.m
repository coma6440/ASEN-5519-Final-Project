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

plot_goal(env.goal)
plot_robot(cell2mat(env.robot.size), A(end,1:3), A(end, 4:7))
plot_obstacles(env.obstacles)
legend(["Path", "Goal", "Robot", "Obstacles"])
saveas(f, fname + ".png");
saveas(f, fname + ".fig");
close
end