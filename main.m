 clear; clc;
% ��ʾΪ��ѧ��������С�������6λ
format longE;
%%
N=3;
M=200;
C=100;
Pc=0.6;             %corssover probability
Pmutation=0.15;       %%mutation probability
LWID=2;
Fsize=13;
tic;
%% ������ģ��
a1=0;a2=0;a3=0;a4=0;a5=0;a6=0;a7=0;
d1=0.3;d2=0;d3=0.7;d4=-0.3;d5=0.7;d6=0;d7=0.3;
%			 thetai    di      ai-1        alphai-1
L1 = Link([-pi/2    d1      a1           pi/2]);
L2 = Link([pi       d2      a2           pi/2]);
L3 = Link([0        d3      a3          -pi/2]);
L4 = Link([0        d4      a4           pi/2]);
L5 = Link([0        d5  	a5          -pi/2]);
L6 = Link([0        d6      a6           pi/2]);
L7 = Link([-pi/2    d7      a7              0]);
% r_arm=0.05;
robot=SerialLink([L1 L2 L3 L4 L5 L6 L7],'name','robot');

%%
% ������ʼ��
P_start=[1.61,0.15,0.15, 148*pi/180, -27*pi/180, 75*pi/180];
% P_2=[1,-0.5,-0.45, 150*pi/180, -30*pi/180, 80*pi/180];
% P_3=[1.5,0,0, 100*pi/180,0, 100*pi/180];
P_end=[0,1,1,60*pi/180,30*pi/180,120*pi/180];
circle1_pose=[1,1,0,100*pi/180,0*pi/180,100*pi/180];
radius1=0.1;
% ����ƽ��ķ�����
circle1_pose_O = [1, 1, 0];
 P1 = [1.61, 0.15, 0.15];
 P2 = [-0.5 , -1 , -0.45 ];
%  P2 = [-0.5, -1, -0.45];
% P2 = [1, -0.5, -0.45];
normal_vector = cross(P1-circle1_pose_O , P2-circle1_pose_O );
% ���ݷ���������֪��O�����깹��ƽ�淽�� ax + by + cz + d = 0
a = normal_vector(1);
b = normal_vector(2);
c = normal_vector(3);
d = -dot(normal_vector, circle1_pose_O );
plane_eq = @(x, y, z) a*x + b*y + c*z + d; 
% ����һ�����귶Χ
[x, y] = meshgrid(-5:0.1:5, -5:0.1:5);
% ����ƽ����ÿ�����zֵ
z = (-a*x - b*y - d) / c;

% % ����ƽ��
figure(1);
% surf(x, y, z);
hold on;
% ����Բ��Ϊ[1, 1, 0]���뾶Ϊ1������ƽ�湲���Բ
theta = linspace(0 , 2*pi, 100);
x_circle = circle1_pose_O (1) + radius1*cos(theta);
y_circle = circle1_pose_O (2) + radius1*sin(theta);
z_circle = (-a*x_circle - b*y_circle - d) / c;
plot3(x_circle, y_circle, z_circle, 'r', 'LineWidth', 2);

% ȷ�������ᶼ�ɼ��������ӽ�
axis equal;
view(3); % ��ʹ�������ʺϵ��ӽ����鿴Բ��ƽ��
%%
% ���������ϰ���
% p_ob1 = [1.0; 0.5; 0; 0.15];
% p_ob1 = [1.0; 0.8; 0.5; 0.15];
p_ob1 = [1.2; 0.8; 0.6; 0.15];
% �����ϰ���뾶
r = p_ob1(4);
% �����ϰ���Բ��
O = p_ob1(1:3);
%%
[Q,F1,F2,E1,E2]= FShortestPoint3D(P1, P2, circle1_pose_O, radius1);
% [E,F,H] = compute_circle_enterpoint(P_start(1:3), P_end(1:3), center1, radius1);
Q1=[Q(1),Q(2),Q(3),0,80*pi/180,80*pi/180];
pose1 = [P_start;Q1;P_end];
%% ���������
popm1=zeros(M,N);%��һ����Ⱦɫ��
for i=1:M
    popm1(i,:)=randperm(N);
end
popm1=Arrange(popm1');
popm1=popm1';
%% ���˶�ѧ�⹹��
pose=[P_start;Q1;P_end];
%joint configuration corresponding to each waypoint
for i=1:N
    jc{1,i}=joco(pose(i,:));% For waypoint i, there are 8 joint configurations, jc{1,i} is 8*7 matrix
end
% ȷ������
for i= 1:(3*N)
    popm2(1,i) = round(rand); 
end
for j=1:N % N---number of waypoints
    a=popm2(1,(j-1)*3+1)*2^2+popm2(1,(j-1)*3+2)*2^1+popm2(1,(j-1)*3+3)*2^0;
    joicon{1,j}=jc{1,j}(a+1,:);
end

% ��ʼ����Ⱥ
k=zeros(M,7*(N-1));
min_value = -500;
max_value = 500;
for i=1:M
    k(i,:)=(max_value - min_value) * rand(1, 7*(N-1)) + min_value;
end
% ����㼶���б�־����̬���������ʵ��λ�����ã�
flag = zeros(M, N);
cutin_index = 2;  % ���衰�����2��Ϊ�����
for i = 1:M
    idx = find(popm1(i,:) == cutin_index);  % �ҵ������2�ڵڼ���
    if ~isempty(idx)
        flag(i, idx) = 1;  % �ڸ����������б�־
    end
end
popm=zeros(M,N+N+7*(N-1));%��Ⱦɫ��
for i=1:M
    popm(i,:)= [popm1(i,:),flag(i, :), k(i,:)];
end
%% �Ŵ��㷨
OPTE=0; 
while C>0 
    C;
    OPTE=OPTE+1;
    disp(['��ǰ�������� OPTE: ', num2str(OPTE)]);
    %selection---according to comparison with the random number
% nn=0;% number of individuals after selection
k = popm(:, N+1:end);
M0=size(k,1);
[theta_total, dq_total, ddq_total, t_total] = multi_liuci_planning(joicon, popm, M0);
% [theta,dq,ddq,t_values] = liuci_planning(joicon,k,M0); 
n=size(t_total,2); 
q = cell(200, 1); 
for i=1:M0
    q{i}=zeros(7,n);
    % ��ÿ��7��1cellԪ��ִ�в���
    for j=1:7
        for l=1:n
            q{i}(j,l) = theta_total{i}{j}(l);
        end
    end
end
%% ��ײ���
f_co = cell(200, 1); 
for i=1:M0
    f_co{i}=zeros(1,n);
    for j=1:n %guijidian
        f_co{i}(1,j) = collision_check(q{i,1}(:,j)',p_ob1);%��i�������ж�Ӧ�ĵ�j�е�7���ؽ�ֵ1*7
    end
end
% ��һ����Ӧ�Ⱥ�������ײ��⣩
fob = zeros(200, 1); 
for i=1:M0
    for j=1:n
        if f_co{i}(1, j) ~= 1
            fob(i,1) = 0;
            break;
        else
            fob(i,1) = 1;
        end
    end
end
%% ������ؽڽǶȱ仯����f_Q�������ؽ�ת���Ƕ���С��  f_Q{l}(i,1)
f_Q = cell(200, 1); 
for l=1:M0
    f_Q{l}=zeros(7,1);
    for i=1:7
        sum_1=0;
        for j=1:n-1
            sum_1=sum_1+abs(q{l,1}(i,j+1)-q{l,1}(i,j));
        end
        f_Q{l}(i,1)=sum_1;
    end
end
% �ڶ�����Ӧ�Ⱥ��������ؽ�ת���Ƕ���С��
fit_2 = zeros(200, 1); 
for i=1:M0
    sum_2=0;
    for j=1:7
        sum_2=sum_2+f_Q{i,1}(j,1);
    end
    fit_2(i,1)=sum_2;
end
%% ����ĩ��ִ�����˶��켣����f_L
p_x=cell(200,1);
p_y=cell(200,1);
p_z=cell(200,1);
for i=1:M0
    p_x{i}=zeros(1,n);
    p_y{i}=zeros(1,n);
    p_z{i}=zeros(1,n);
    for j=1:n
        [p_x{i,1}(1,j),p_y{i,1}(1,j),p_z{i,1}(1,j)] = DHkine(q{i,1}(:,j)');
    end
end
f_L = zeros(200,1);
for i=1:M0
    sum_3=0;
    for j=1:n-1
        sum_3=sum_3+sqrt((p_x{i,1}(1,j+1)-p_x{i,1}(1,j))^2+(p_y{i,1}(1,j+1)-p_y{i,1}(1,j))^2+(p_z{i,1}(1,j+1)-p_z{i,1}(1,j))^2);
    end
    f_L(i,1)=sum_3;
end
% ��������Ӧ�Ⱥ�����ĩ��ִ�����켣��С��
fit_3 = f_L;

%%��Ӧ�Ⱥ���
f_k = zeros(200,1);
for i=1:M0
    f_k(i,1) = -fob(i,1)/(0.1*fit_2(i,1)+fit_3(i,1));
end
%% ѡ�� selection
%--ɸѡ��Ӧ��Ϊ0�ĸ���
non_zero_fitness_indices = find(f_k ~= 0);
popm_except_0 = popm(non_zero_fitness_indices,:);%----ɸѡ��0ֵ�����Ⱥ
fit_except_0 = f_k(non_zero_fitness_indices,:);
[popm_sel,fit_sel] = selection(fit_except_0,popm_except_0);
%[k_sel,fit_sel] = selection(fit_except_0,k_except_0);
nn = size(popm_sel,1);

%%ÿ��ѡ�񶼱������ŵ���Ⱥ%%Save the optimized individual after selection
% % popm_sel=popm_sel(1:nn,:);%population after selection
[fit_sel_min, fit_sel_min_index]=min(fit_sel);% optimized path length value and the corresponding individual after selection
[fit_sel_max, fit_sel_max_index]=max(fit_sel); 
popm_sel(fit_sel_max_index,:) = popm_sel(fit_sel_min_index,:);% add the optimized individua to the new population

%------crossover 
popm_sel = Cross(popm_sel, Pc);% �򵥽���

%------mutation
% �������
mutation_range=10;
popm_sel = Mutation(popm_sel, Pmutation, mutation_range);

%------���²����Ӵ�������Ⱥ
NSel=size(popm_sel,1);
NIND=size(k,1);  %200
[f,index]=sort(f_k);   %ԭ�ȵ���Ⱥ����Ӧ����С�����������
popm_new=[popm(index(1:NIND-NSel),:);popm_sel]; %���յõ�����Ⱥ200
% ========== ����ÿ������� flag�����б�־�� ========== 
M1=size(popm_new,1);
popm1_new = popm_new(:, 1:N);  % ��ȡ����˳�򲿷�
flag_new = zeros(M1, N);       % ��ʼ�� flag
for i = 1:M1
    idx = find(popm1_new(i,:) == cutin_index);  % �ҵ������2�ڸ����е�ʵ��λ��
    if ~isempty(idx)
        flag_new(i, idx) = 1;  % �ڸ�λ����������
    end
end
% �滻ԭ���� flag ����
popm_new(:, N+1:N+N) = flag_new;  % ������Ⱦɫ���е� flag
%% ����ѡ�񡢽��桢��������½�����Ӧ�ȼ���

k_new = popm_new(:, N+1:end);
[theta_total_new, dq_total_new, ddq_total_new, t_total_new] = multi_liuci_planning(joicon, popm_new, M1);
% [theta_new,dq_new,ddq_new,t_values_new] = liuci_planning(joicon,k_new,M1);
n_new=size(t_total_new,2); 
q_new = cell(200, 1); 
for i=1:M1
    q_new{i}=zeros(7,n_new);
    % ��ÿ��7��1cellԪ��ִ�в���
    for j=1:7
        for l=1:n_new
            q_new{i}(j,l) = theta_total_new{i}{j}(l);
        end
    end
end
% ��ײ���
f_co_new = cell(200, 1); 
for i=1:M1
    f_co_new{i}=zeros(1,n);
    for j=1:n_new
        f_co_new{i}(1,j) = collision_check(q_new{i,1}(:,j)',p_ob1);
    end
end
% ��һ����Ӧ�Ⱥ�������ײ��⣩
fob_new = zeros(200, 1); 
for i=1:M1
    for j=1:n_new
        if f_co_new{i}(1, j) ~= 1
            fob_new(i,1) = 0;
            break;
        else
            fob_new(i,1) = 1;
        end
    end
end
f_Q_new = cell(200, 1); 
for l=1:M1
    f_Q_new{l}=zeros(7,1);
    for i=1:7
        sum_1_new=0;
        for j=1:n_new-1
            sum_1_new=sum_1_new+abs(q_new{l,1}(i,j+1)-q_new{l,1}(i,j));
        end
        f_Q_new{l}(i,1)=sum_1_new;
    end
end
% �ڶ�����Ӧ�Ⱥ��������ؽ�ת���Ƕ���С��
fit_2_new = zeros(200, 1); 
for i=1:M1
    sum_2_new=0;
    for j=1:7
        sum_2_new=sum_2_new+f_Q_new{i,1}(j,1);
    end
    fit_2_new(i,1)=sum_2_new;
end
p_x_new=cell(200,1);
p_y_new=cell(200,1);
p_z_new=cell(200,1);
for i=1:M1
    p_x_new{i}=zeros(1,n_new);
    p_y_new{i}=zeros(1,n_new);
    p_z_new{i}=zeros(1,n_new);
    for j=1:n_new
        [p_x_new{i,1}(1,j),p_y_new{i,1}(1,j),p_z_new{i,1}(1,j)] = DHkine(q_new{i,1}(:,j)');
    end
end
f_L_new = zeros(200,1);
for i=1:M1
    sum_3_new=0;
    for j=1:n_new-1
        sum_3_new=sum_3_new+sqrt((p_x_new{i,1}(1,j+1)-p_x_new{i,1}(1,j))^2+(p_y_new{i,1}(1,j+1)-p_y_new{i,1}(1,j))^2+(p_z_new{i,1}(1,j+1)-p_z_new{i,1}(1,j))^2);
    end
    f_L_new(i,1)=sum_3_new;
end
% ��������Ӧ�Ⱥ�����ĩ��ִ�����켣��С��
fit_3_new = f_L_new;
% �µ���Ӧ�Ⱥ���
f_k_new = zeros(200,1);
for i=1:M1
    f_k_new(i,1) = -fob_new(i,1)/(0.1*fit_2_new(i,1)+fit_3_new(i,1));
end

% ѡ���µ���Ӧ�ȵ���Сֵ
minf_k_new=min(f_k_new);% minimum length of the new population
% ѡ���������������Ӧ��
f_k_new_min(OPTE,1)=minf_k_new;% save the minimum length value of the (C+1)th  iteration
% % fitness=fit(Tem1,m,maxtem1,mintem1);% the final fitness value
rr=find(f_k_new==minf_k_new);% the number of indivilual with minimum path length
% ���ŵ�k��
popmop=popm_new(rr(1,1),:); % % %the final optimal chromosome
% �������Ų���k��Ӧ�Ĺؽ�ת������f_Q_op
f_Q_op = fit_2_new(rr(1,1),:);
% �������Ų���k��Ӧ�Ĺؽ�ת������f_L_op
f_L_op = fit_3_new(rr(1,1),:);
k=[];
popm=popm_new;% the final population
C=C-1;
end
kkk=1:1:OPTE;
figure(2)
plot(kkk,f_k_new_min','LineWidth',LWID);
grid on
xlabel('��������')
ylabel ('��Ӧ��ֵ')


% �������Ÿ����Ӧ��ֵ
Mop=size(popmop,1);
kop = popmop(:, N+1:end);
[theta_total_op, dq_total_op, ddq_total_op, t_total_op] = multi_liuci_planning(joicon, popmop, Mop);
% [theta_op,dq_op,ddq_op,t_values_op] = liuci_planning(joicon,kop,Mop); % theta_op----1��1cell,cell:7��1cell,cell:1��21double
xend=t_total_op(1,end);

EEpose=DHkine_1(theta_total_op{1,1});
yy1=roundn(EEpose(1,:),-4);
yy2=roundn(EEpose(2,:),-4);
yy3=roundn(EEpose(3,:),-4);

m=1;
for i=1:N
    for j=m:size(EEpose,2)
%         if Waypo(i,1)==yy1(1,j) && Waypo(i,2)==yy2(1,j) && Waypo(i,3)==yy3(1,j)%%%% For EE position
        if pose1(i,1)==yy1(1,j) && pose1(i,2)==yy2(1,j) && pose1(i,3)==yy3(1,j)%%%% For EE attitude
              tkj(i)=j; 
            break
        end
    end
end

qs = zeros(7,size(t_total_op,2));
% ѭ������ theta_op ��ÿ��Ԫ�أ���������䵽 qs ��
for i = 1:7
    for j = 1:size(t_total_op,2)
        % ��ȡ theta_op �е����ݣ�����ֵ�� qs
        qs(i, j) = theta_total_op{1,1}{i,1}(1,j);
    end
end
qs = qs';
% % ����ؽ�2�Ĺ켣�仯
% EEpose_q2=DHkine_q2(theta_total_op{1,1});
% % ����ؽ�3�Ĺ켣�仯
% EEpose_q3=DHkine_q3(theta_total_op{1,1});
% % ����ؽ�4�Ĺ켣�仯
% EEpose_q4=DHkine_q4(theta_total_op{1,1});
% % ����ؽ�5�Ĺ켣�仯
% EEpose_q5=DHkine_q5(theta_total_op{1,1});

toc;
%% �����ؽڵ��˶��켣����
figure(1)
hold on;
plot3(P_start(1),P_start(2),P_start(3),'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
 text(P_start(1),P_start(2),P_start(3));
plot3(Q1(1),Q1(2),Q1(3),'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
plot3(P_end(1),P_end(2),P_end(3),'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
% plot3(P_4(1),P_4(2),P_4(3),'go', 'MarkerSize', 5, 'MarkerFaceColor', 'g');
text(P_end(1),P_end(2),P_end(3));
robot.plot(qs(10,:));
plot_sphere(p_ob1(1:3), p_ob1(4));
hold on;
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
hold on
robot.plot(qs);
hold on



