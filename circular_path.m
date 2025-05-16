function [theta_c, dq_c, ddq_c, t_c] = circular_path(q_start, current_time)
% 圆轨迹参数
radius1 = 0.1;
circle1_pose_O = [1, 1, 0];

P1 = [1.61, 0.15, 0.15];
P2 = [-0.5 , -1 , -0.45 ];
normal_vector = cross(P1-circle1_pose_O , P2-circle1_pose_O);
a = normal_vector(1);
b = normal_vector(2);
c = normal_vector(3);
d = -dot(normal_vector, circle1_pose_O);

theta = linspace(0 , 2*pi, 51);  % 仅生成 51 个点
x_circle = circle1_pose_O(1) + radius1*cos(theta);
y_circle = circle1_pose_O(2) + radius1*sin(theta);
z_circle = (-a*x_circle - b*y_circle - d) / c;

pose_traj = zeros(51, 6);
for i = 1:51
    pose_traj(i,1:3) = [x_circle(i), y_circle(i), z_circle(i)];
    pose_traj(i,4:6) = q_start(4:6);
end

% 逐点求逆解
theta_mat = zeros(51, 7);
for i = 1:51
    q_sols = joco(pose_traj(i,:));
    theta_mat(i,:) = q_sols(1,:); % 只取第一个解
end

% 利用 liuci_single 生成轨迹
[theta_c, dq_c, ddq_c, t_c] = liuci_single(theta_mat(1,:), theta_mat(end,:), zeros(1,7));
t_c = t_c + current_time;

end
