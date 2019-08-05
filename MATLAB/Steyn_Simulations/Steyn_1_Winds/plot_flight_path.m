%PLOT IN 3D Flight path with and without WIND

plot3(X_out,Y_out,Z_out,'--');  % Plot without wind disturbance
grid on
xlabel('x Distance (m)'), ylabel('y Distance (m'), zlabel('h Height (m)')
hold on
plot3(XW_out,YW_out,ZW_out);  % Plot with wind disturbance
legend('No Wind', 'Ambient Wind');
hold off