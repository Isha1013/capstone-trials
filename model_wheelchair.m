% Parameters and Constants
m = 70.3; % Mass of wheelchair-user system (kg)
g = 9.81; % Gravitational acceleration (m/s^2)
a = 0.0523; % Horizontal CoG distance from rear axle (m)
b = 0.4706; % Vertical CoG distance from rear axle (m)
r = sqrt(a^2 + b^2);
I = m * (a^2 + b^2); % Moment of inertia (kg·m^2)
% tipping_angle = deg2rad(83.66+18.2); % Critical tipping angle (rad)
tipping_angle = pi/2;

% final_torque_gravity = (m * g * a_initial * cos(tipping_angle)); %torque from user due to gravity 

% Time Simulation Parameters
dt = 0.001; % Time step (s)
t_total = 3.0; % Total simulation time (s)
time = 0:dt:t_total; % Time vector

% Initial Conditions
% theta = 0;
theta = deg2rad(83.66-11.86); % Initial angular position (rad)
omega = 0; % Initial angular velocity (rad/s)
alpha = 0; % Initial angular acceleration (rad/s^2)

% Arrays to Store Results
theta_array = zeros(size(time));
omega_array = zeros(size(time));
alpha_array = zeros(size(time));
net_torque_array = zeros(size(time));      % Array to store net torque values
external_torque_array = zeros(size(time)); 
gravity_torque_array = zeros(size(time));  % Array to store gravity torque values

% User-defined Torque Function or Constant

gravity_torque_initial = m * g * r * sin((pi/2)-theta);
max_ext_torque = 1.1*gravity_torque_initial;

external_torque_function = @(t) (t < 0.05) .* (101.9 + (max_ext_torque-101.9) * ...
    (1 - exp(-100 * t))) + (t >= 0.05) .* max_ext_torque ;

% external_torque_function = @(t) 120;
% external_torque_function = @(tau) 1.5*tau;
gravity_torque_function = @(theta) m * g * r * sin((pi/2)-theta);

% Simulation Loop
for i = 1:length(time)
%     external_torque = external_torque_array(i);
%     disp(rad2deg(theta));
%     if rad2deg(theta) > 90
%         gravity_torque = -gravity_torque_function(theta);
%     else
%         gravity_torque = gravity_torque_function(theta);
%     end
    disp(theta);
    gravity_torque = gravity_torque_function(theta);
    external_torque = external_torque_function(time(i));
    net_torque = -gravity_torque + external_torque;
    
    % Numerical Integration (Euler's Method)
    alpha = net_torque / I;
    omega = omega + alpha * dt;
    theta = theta + omega * dt;
    
    disp(rad2deg(theta));
   
    % Store results
    theta_array(i) = theta;
    omega_array(i) = omega;
    alpha_array(i) = alpha;
    net_torque_array(i) = net_torque;
    external_torque_array(i) = external_torque;
    gravity_torque_array(i) = gravity_torque;

    % Stop simulation if tipping angle is reached
    if theta > tipping_angle
        fprintf('Simulation stopped: Tipping occurred at t = %.2f s\n', time(i));
        break;
    end
end

% Trim unused time steps
time = time(1:i);
theta_array = theta_array(1:i);
omega_array = omega_array(1:i);
alpha_array = alpha_array(1:i);
net_torque_array = net_torque_array(1:i);
external_torque_array = external_torque_array(1:i);
gravity_torque_array = gravity_torque_array(1:i);

% Plot Results
figure;

% Angular Position Plot
subplot(4, 1, 1);
plot(time, rad2deg(theta_array), 'b', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Deg');
title('Angular Position');
yline(rad2deg(tipping_angle), '--r', 'Tipping Angle');
grid on;

% Angular Velocity Plot
subplot(4, 1, 2);
plot(time, rad2deg(omega_array), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Deg/s');
title('Angular Velocity');
grid on;

% Angular Acceleration Plot
subplot(4, 1, 3);
disp(length(alpha_array));
disp(length(time));
plot(time, rad2deg(alpha_array), 'g', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Deg/s^2)');
title('Angular Acceleration');
grid on;

% Torque Plot
subplot(4, 1, 4);
plot(time, net_torque_array, 'm', 'LineWidth', 1.5, 'DisplayName', 'Net Torque');
hold on;
plot(time, external_torque_array, 'b', 'LineWidth', 1.5, 'DisplayName', 'External Torque');
plot(time, gravity_torque_array, 'r-.', 'LineWidth', 1.5, 'DisplayName', 'Gravity Torque');
xlabel('Time (s)');
ylabel('Nm');
title('Torque Components');
legend('show');
grid on;