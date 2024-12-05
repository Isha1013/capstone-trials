% Simulation Parameters
m = 70.3; % Mass (kg)
g = 9.81; % Gravitational acceleration (m/s^2)
a = 0.0523; % Horizontal CoG distance (m)
b = 0.4706; % Vertical CoG distance (m)
r = sqrt(a^2 + b^2);
I = m * (a^2 + b^2); % Moment of inertia (kg·m^2)
tipping_angle = pi/2;

% Initial conditions
theta0 = deg2rad(83.66 - 11.86); % Initial angular position
omega0 = 0; % Initial angular velocity
y0 = [theta0; omega0];

% Time span for the simulation
tspan = [0, 3.0];

% Solve using ode45
[t, y] = ode45(@(t, y) wheelchair_dynamics(t, y, m, g, r, I, tipping_angle), tspan, y0);

% Extract results
theta_array = y(:, 1);
omega_array = y(:, 2);

% Stop if tipping occurs
tipping_idx = find(theta_array >= tipping_angle, 1);
if ~isempty(tipping_idx)
    fprintf('Simulation stopped: Tipping occurred at t = %.2f s\n', t(tipping_idx));
    theta_array = theta_array(1:tipping_idx);
    omega_array = omega_array(1:tipping_idx);
    t = t(1:tipping_idx);
end
initial_theta = deg2rad(83.66 - 11.86);

gravity_torque_array = m * g * r * ((pi/2) - theta_array);
initial_gt = m * g * r * (pi/2 - initial_theta);

fh = 0.8*m*g.*(pi/2 - initial_theta);
Rh = 0.2487;
Rw = 0.295;
external_torque_array = (fh*(Rh + r*(Rh/Rw)))*ones(size(theta_array));
net_torque_array = -gravity_torque_array + external_torque_array;

subplot(3, 1, 1);
plot(t, gravity_torque_array, 'r', 'LineWidth', 1.5); 
hold on;
plot(t, external_torque_array, 'b', 'LineWidth', 1.5);
plot(t, net_torque_array, 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Nm');
title('Torque vs. Time');
legend({'Gravitational Torque', 'External Torque', 'Net Torque'}, 'Location', 'best');
grid on;

% Plot angular position
subplot(3, 1, 2);
plot(t, rad2deg(theta_array), 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Degrees');
title('Angular Displacement vs. Time');
grid on;

% Plot angular velocity
subplot(3, 1, 3);
plot(t, rad2deg(omega_array), 'k', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Deg/s');
title('Angular Velocity vs. Time');
grid on;

sgtitle('Tipping point at t = 0.3s');