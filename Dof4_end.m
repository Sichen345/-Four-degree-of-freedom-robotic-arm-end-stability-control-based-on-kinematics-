% Clear all variables
clear all;
clc;

% Define inputs
d0 = 2.0;  % 初始基座高度
d1 = 4.0;  % 第一连杆长度
d2 = 4.0;  % 第二连杆长度
d3 = 4.0;  % 第三连杆长度
P = [5; 5; 5; 0];  % 目标末端位姿 [x; y; z; alpha]

% 末端位置
x = P(1);
y = P(2);
z = P(3);

% 逆运动学计算关节角度
q1 = atan2(y, x);
Ex = x / cos(q1);
Ez = z - d0;
Ex = Ex - d3;
Ez = Ez;

C = ((Ex^2) + (Ez^2) - d1^2 - d2^2) / (2 * d1 * d2);
q3 = acos(C);  % Joint 3 角度
q2 = atan2(Ez, Ex) - atan2(d2 * sin(q3), d1 + d2 * cos(q3));  % Joint 2 角度

% q4杆保持水平
q4 = 0;

Q = [q1, q2, q3, q4];  % 目标关节角度
% 连杆长度
L0 = d0;
L1 = d1;
L2 = d2;
L3 = d3;

% Time vector for animation
num_frames = 10000;  % Number of frames for the animation
time_steps = linspace(0, 200*pi, num_frames);  % Time steps for base movement (sinusoidal)

% End effector fixed position
x4_fixed = x;
y4_fixed = y;
z4_fixed = z;

% Initialize the figure
fig = figure();
ax = axes('Parent', fig);
grid(ax, 'on');
view(3);  % Set the view to 3D
a = d1 + d2 + d3;
b = d0 + d1 + d2 + d3;
xlim([-a, a]);
ylim([-a, a]);
zlim([0, b]);
xlabel('X');
ylabel('Y');
zlabel('Z');
colors = ['r', 'g', 'b', 'c'];

% Loop through each frame
for t = time_steps
    % Sinusoidal base movement in x and z directions
    base_x = 0.52 * sin(t);  % Base moves forward and backward (x-axis)
    base_z = d0 + 0.52 * cos(t);  % Base moves up and down (z-axis)

    % Calculate new joint angles to maintain the fixed end-effector position
    q1 = atan2(y4_fixed, x4_fixed - base_x);
    Ex = (x4_fixed - base_x) / cos(q1);
    Ez = z4_fixed - base_z;
    Ex = Ex - d3;
    
    C = ((Ex^2) + (Ez^2) - d1^2 - d2^2) / (2 * d1 * d2);
    q3 = acos(C);
    q2 = atan2(Ez, Ex) - atan2(d2 * sin(q3), d1 + d2 * cos(q3));
    
    % Fix q4 to keep the last link horizontal
    q4 = 0;
    
    % Define the joint angles (in radians)
    tht1 = q1;
    tht2 = q2;
    tht3 = q3;
    tht4 = q4;  % This is always 0 to keep the last link horizontal

    % Calculate the positions of the joints
    x1 = base_x;
    y1 = 0;
    z1 = base_z;

    x2 = base_x + L1 * cos(tht1) * cos(tht2);
    y2 = L1 * cos(tht2) * sin(tht1);
    z2 = base_z + L1 * sin(tht2);

    x3 = x2 - L2 * (cos(tht1) * sin(tht2) * sin(tht3) - cos(tht1) * cos(tht2) * cos(tht3));
    y3 = y2 - L2 * (sin(tht1) * sin(tht2) * sin(tht3) - cos(tht2) * cos(tht3) * sin(tht1));
    z3 = z2 + L2 * (cos(tht2) * sin(tht3) + cos(tht3) * sin(tht2));

    % End effector remains at fixed position
    x4 = x4_fixed;
    y4 = y4_fixed;
    z4 = z4_fixed;

    % Define the coordinates of the arm
    x = [base_x, x2, x3, x4];
    y = [0, y2, y3, y4];
    z = [base_z, z2, z3, z4];

    % Clear the previous plot
    cla(ax);

    % Draw the links of the arm
    for i = 1:length(x) - 1
        line([x(i), x(i + 1)], [y(i), y(i + 1)], [z(i), z(i + 1)], 'Color', colors(i), 'LineWidth', 2);
        hold on;
    end

    % Plot the joints
    scatter3(ax, x, y, z, 50, 'k', 'filled');
    hold on;

    % Plot the point
    scatter3(ax, x4_fixed, y4_fixed, z4_fixed, 50, 'filled');
    hold on;

    % Capture the frame for the animation
    frames(t == time_steps) = getframe(fig);
end

% 保存动图
filename = 'robot_arm_base_movement_sinusoidal_horizontal_link.gif';
for i = 1:length(frames)
    [imind, cm] = rgb2ind(frame2im(frames(i)), 256);
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', 0.1);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', 0.1);
    end
end
