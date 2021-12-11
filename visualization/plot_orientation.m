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