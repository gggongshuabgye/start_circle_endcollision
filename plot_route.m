
% pose--pose of the N waypoints
% R--first part of the chromosome （individual）//
% joc_1--joint configuration-sequence of the first choromosome (individual),7*N//
% A3_1--optimized parameter of the first chromosome (individual)--xsA3(1)
function [zzjt,  qq, dqq]=plot_route(R,joc_1,A3_1)
N1=size(joc_1,1);% number of joints,joc_1--7*N
N2=size(R,2); % number of waypoints
A=270*pi/180;% A is the bound of each joint angle
N_points=50;
for i = 1:N1
    for j = 1:N2-1
        a0(i,j) = asin(joc_1(i,j)/A);
        a1(i,j) = 0;     
        delta = a0(i,j) - asin(joc_1(i,j+1)/A);
        a3(i,j) = sign(delta) * abs(A3_1);

        % ? 防止除以0
        if abs(a3(i,j)) < 1e-6
            t(i,j) = 1; % 默认1秒时间或其它你想设置的最小时间
            a2(i,j) = 0;
        else
            t(i,j) = (2 * delta / a3(i,j))^(1/3);
            a2(i,j) = -1.5 * a3(i,j) * t(i,j);
        end
    end
end
% % for i=1:N1
% %     for j=1:N2-1
% %         a0(i,j)=asin(joc_1(i,j)/A);
% %         a1(i,j)=0;     
% %         a3(i,j)=sign(a0(i,j)-asin(joc_1(i,j+1)/A))*abs(A3_1);
% %         t(i,j)=(2*(a0(i,j)-asin(joc_1(i,j+1)/A))/a3(i,j))^(1/3);
% %         a2(i,j)=-1.5*a3(i,j)*t(i,j);
% %     end
% % end
%    q=cell(i,j); 
%    dq=cell(i,j); 
q = cell(N1, N2-1); % 初始化 q 为 N1 x (N2-1) 大小的单元格
dq = cell(N1, N2-1);
for i=1:N1 % N1--number of joints
    for j=1:N2-1 % N1--number of waypoints
%          tt{i,j}=0:0.02:t(i,j);
       tt{i,j} = linspace(0, t(i,j), N_points);  % 生成 N_points 个点
        zj=A*sin(a0(i,j)+a1(i,j)*tt{i,j}+a2(i,j)*tt{i,j}.^2+a3(i,j)*tt{i,j}.^3);
        jsd=A*sin(a0(i,j)+a1(i,j)*tt{i,j}+a2(i,j)*tt{i,j}.^2+a3(i,j)*tt{i,j}.^3).*(2*a2(i,j)*tt{i,j}+3*a3(i,j)*tt{i,j}.^2);
        q{i,j}=[q{i,j} zj];
        dq{i,j}=[dq{i,j} jsd];
        bucj(i,j)=zj(end);
    end
end
%  for j=1:N2-1
%     Tim(j)=max(t(:,j));% maximum time among the 7 joints movements times between adjacent waypoints
% %     zjt=0:0.02:Tim(j);
% %     ltm=size(zjt,2);
% %     for i=1:N1
% %         ls(i)=size(q{i,j},2);
% %         q{i,j}=[q{i,j} ones(1,(ltm-ls(i)))*bucj(i,j)];
% %         dq{i,j}=[dq{i,j} zeros(1,(ltm-ls(i)))*bucj(i,j)];
%      end
% % end
qq=cell(N1,1);
dqq=cell(N1,1);
for i=1:N1
    for j=1:N2-1
        qq{i,1}=[qq{i,1},q{i,j}]; 
        dqq{i,1}=[dqq{i,1},dq{i,j}]; 
    end
end
%Final time
% for j=1:N2-1
%     
%     Ft1{j,1}=0:0.02:Tim(j);
%     if j>1
%         for k=1:j-1
%             Ft1{j,1}=Ft1{j,1}+Tim(k);
%         end
%     end      
% end
zzjt=[];
total_time = 0;
for j=1:N2-1
     time_segment = linspace(0, t(1, j), N_points); % 每段生成 N_points 个时间点
        zzjt = [zzjt, time_segment + total_time]; % 累加时间并拼接
        total_time = total_time + t(1, j); % 更新总时间
% zzjt=[zzjt Ft1{j,1}];
end





