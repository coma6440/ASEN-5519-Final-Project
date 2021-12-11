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