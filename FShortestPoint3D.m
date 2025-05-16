% P1=[-3,1,0, 148*pi/180, -27*pi/180, 75*pi/180];
% P2=[-2,-2,0,150*pi/180,-30*pi/180,80*pi/180];
% O=[0,0,0,100*pi/180,0*pi/180,100*pi/180];
% r=2;
function [Q,F1,F2,E1,E2]= FShortestPoint3D(P1, P2, O, radius)
%% 已知点的坐标
O = [1, 1, 0];
P1 = [1.61, 0.15, 0.15];
P2 = [-0.5, -1, -0.45];
% P2 = [1, -0.5, -0.45];
radius = 0.1;
% % 计算平面的法向量
normal_vector = cross(P1-O, P2-O);
% 
% % 根据法向量和已知点O的坐标构建平面方程 ax + by + cz + d = 0
% a = normal_vector(1);
% b = normal_vector(2);
% c = normal_vector(3);
% d = -dot(normal_vector, O);
% plane_eq = @(x, y, z) a*x + b*y + c*z + d; 
% % 定义一个坐标范围
% [x, y] = meshgrid(-5:0.1:5, -5:0.1:5);
% 
% % 计算平面上每个点的z值
% z = (-a*x - b*y - d) / c;
% 
% % 绘制平面
% figure;
% surf(x, y, z);
% hold on;

% % 绘制已知点
% plot3(O(1), O(2), O(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
% plot3(P1(1), P1(2), P1(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
% plot3(P2(1), P2(2), P2(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
% % 绘制球体
% [x_sphere, y_sphere, z_sphere] = sphere; % 创建球体网格
% radius = 1; % 球体半径
% x_sphere = x_sphere * radius + O(1); % 平移并缩放球体网格
% y_sphere = y_sphere * radius + O(2);
% z_sphere = z_sphere * radius + O(3);
% surf(x_sphere, y_sphere, z_sphere, 'FaceAlpha', 0.3, 'EdgeAlpha', 0.1, 'FaceColor', 'b', 'EdgeColor', 'none');
% 
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Plot of the Plane');
% grid on;
% % 设置坐标轴范围
% axis([-3 3 -3 3 -3 3]);
% axis equal;
%% 计算直线P1O的方向向量
direction_vector1 = P1 - O;
% 计算直线与球的交点  
syms x y z t1; % 定义符号变量
x_line1= O(1) + t1 * direction_vector1(1);  
y_line1= O(2) + t1 * direction_vector1(2);  
z_line1= O(3) + t1 * direction_vector1(3); 
equation = (x_line1 - O(1))^2 + (y_line1 - O(2))^2 + (z_line1 - O(3))^2 - radius^2;
% 使用solve解方程组  
solutions1 = solve(equation,t1);  
% 提取t1和t2的数值解  
t11_sym=solutions1(1);
t12_sym=solutions1(2);
t11_num = double(subs(t11_sym));  
t12_num = double(subs(t12_sym)); 
% disp(solutions1); % 显示整个solutions结构  
% t1=solutions1(1);
% t2=solutions1(2);
F1 = O + t11_num * direction_vector1;
F2 = O + t12_num * direction_vector1;
hold on; 
% 绘制交点F 
plot3(F1(1), F1(2), F1(3), 'mo', 'MarkerSize', 1, 'MarkerFaceColor', 'm'); % 交点F  
 plot3(F2(1), F2(2), F2(3), 'mo', 'MarkerSize', 1, 'MarkerFaceColor', 'm'); % 交点F 
 % 绘制直线P1O  
 plot3([O(1), P1(1)], [O(2), P1(2)], [O(3), P1(3)], 'r-', 'LineWidth', 2);

%% % 计算直线P2O的方向向量  
direction_vector = P2 - O; 
% 计算直线与球的交点  
syms x y z t2; % 定义符号变量  
% 直线方程：x = O(1) + t * direction_vector(1), y = O(2) + t * direction_vector(2), z = O(3) + t * direction_vector(3)  
% 球方程：(x - O(1))^2 + (y - O(2))^2 + (z - O(3))^2 = radius^2  
% 代入求解  
x_line= O(1) + t2 * direction_vector(1);  
y_line= O(2) + t2 * direction_vector(2);  
z_line= O(3) + t2 * direction_vector(3);  
% equation4 = (x - O(1))^2 + (y - O(2))^2 + (z - O(3))^2 == 1; % 球的半径为1  
  equation = (x_line - O(1))^2 + (y_line - O(2))^2 + (z_line - O(3))^2 - radius^2;
% 使用solve解方程组  
solutions = solve(equation,t2);  
disp(solutions); % 显示整个solutions结构  
% disp(solutions.t); % 显示solutions.t的内容
t21_sym=solutions(1);
t22_sym=solutions(2);
t21_num = double(subs(t21_sym));  
t22_num = double(subs(t22_sym)); 
E1 = O + t21_num * direction_vector;
E2 = O + t22_num * direction_vector;

 hold on; 
% 绘制交点E  
plot3(E1(1), E1(2), E1(3), 'mo', 'MarkerSize', 1, 'MarkerFaceColor', 'm'); % 交点E  
 plot3(E2(1), E2(2), E2(3), 'mo', 'MarkerSize', 1, 'MarkerFaceColor', 'm'); % 交点E 
 
% 绘制直线P2O  
 plot3([O(1), P2(1)], [O(2), P2(2)], [O(3), P2(3)], 'r-', 'LineWidth', 2);

%% 
 hold on;
% 绘制直线EF  
 plot3([E2(1), F2(1)], [E2(2), F2(2)], [E2(3), F2(3)], 'r-', 'LineWidth', 2);
% 计算EF的中点和中垂线方向向量  
EF_midpoint = (E2 + F2) / 2;  
EF_vector = F2 - E2;  
EF_perp_vector = cross(EF_vector, normal_vector); % 确保中垂线在平面上  
EF_perp_vector = EF_perp_vector / norm(EF_perp_vector); % 归一化方向向量  
%中垂线方程
syms  t3; 
x_line= O(1) + t3 * EF_perp_vector(1);  
y_line= O(2) + t3 * EF_perp_vector(2);  
z_line= O(3) + t3 * EF_perp_vector(3);  
% equation4 = (x - O(1))^2 + (y - O(2))^2 + (z - O(3))^2 == 1; % 球的半径为1  
  equation = (x_line - O(1))^2 + (y_line - O(2))^2 + (z_line - O(3))^2 - radius^2;
% 使用solve解方程组  
solutions = solve(equation,t3);  
disp(solutions); % 显示整个solutions结构
t31_sym=solutions(1);
t32_sym=solutions(2);
t31_num = double(subs(t31_sym));  
t32_num = double(subs(t32_sym)); 
Q1 = O + t31_num * EF_perp_vector;
Q2 = O + t32_num * EF_perp_vector;
Q=[Q1(1),Q1(2),Q1(3)];
 hold on;
  plot3(Q(1), Q(2), Q(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm');
%  plot3(Q2(1), Q2(2), Q2(3), 'mo', 'MarkerSize', 10, 'MarkerFaceColor', 'm'); % 交点E 
  % 绘制OM线段  
% plot3([O(1), Q(1)], [O(2), Q(2)], [O(3), Q(3)], 'm--', 'LineWidth', 2);
%  Q=Q1;


end
