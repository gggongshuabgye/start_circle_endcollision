function [theta_total, dq_total, ddq_total, t_total] = multi_liuci_planning(joicon, popm, M)

% 输入
% joicon: 1×N cell，每个cell是1×7固定构型
% popm: 染色体 M × (N + 7*(N-1))，前N列是顺序，后面是每段7个K
% M: 种群数

theta_total = cell(M,1);
dq_total = cell(M,1);
ddq_total = cell(M,1);
t_total = []; 
N = length(joicon); % 任务点数量
% total_segments = N - 1; % 总段数
% points_per_segment = 51; % 每段轨迹的插值点数
% total_points = total_segments * points_per_segment; % 总时间点数 
  % 预分配时间矩阵
%  t_total = zeros(1, total_points); % 用于存储所有个体的时间点
for i = 1:M
    task_seq = popm(i, 1:N); % 取顺序
    flag_seq = popm(i, N+1:2*N); % 绕行标志
    k_all = popm(i, 2*N+1:end); % 多段k
    
    % 根据顺序排列关节配置
    q_seq = zeros(N,7);
    for j = 1:N
        q_seq(j,:) = joicon{task_seq(j)};
    end

    current_time = 0;
    theta_each = cell(7,1);
    dq_each = cell(7,1);
    ddq_each = cell(7,1);
        t_each = [];
    for j = 1:7
        theta_each{j} = [];
        dq_each{j} = [];
        ddq_each{j} = [];
    end


    for seg = 1:N-1
        q_start = q_seq(seg,:);
        q_end = q_seq(seg+1,:);
        k_seg = k_all((seg-1)*7+1 : seg*7); % 每段的k
        % ---- 正常轨迹段 ----
        [theta_seg, dq_seg, ddq_seg, t_seg] = liuci_single(q_start, q_end, k_seg);
        t_seg = t_seg + current_time;
        current_time = t_seg(end);

        for j = 1:7
            theta_each{j} = [theta_each{j}, theta_seg{j}];
            dq_each{j} = [dq_each{j}, dq_seg{j}];
            ddq_each{j} = [ddq_each{j}, ddq_seg{j}];
        end
        t_each = [t_each, t_seg];
                % ---- 若下一个任务点（终点）是切入点，则此段后接绕行轨迹 ----
        rx_flag = (flag_seq(seg+1) == 1);
        if rx_flag
            [theta_c, dq_c, ddq_c, t_c] = circular_path(q_end, current_time);  % 用终点姿态传入
            t_c = t_c + current_time;
            current_time = t_c(end);

            for j = 1:7
                theta_each{j} = [theta_each{j}, theta_c{j}];
                dq_each{j} = [dq_each{j}, dq_c{j}];
                ddq_each{j} = [ddq_each{j}, ddq_c{j}];
            end
            t_each = [t_each, t_c];
        end
    end
%         %%  绕行段
%         rx_flag =  (flag_seq(task_seq(seg)) == 1);  % 判断当前任务点是否绕行
%         if rx_flag
%             [theta_c, dq_c, ddq_c, t_c] = circular_path(q_start, current_time);
%             for j = 1:7
%                 theta_each{j} = [theta_each{j}, theta_c{j}];
%                 dq_each{j} = [dq_each{j}, dq_c{j}];
%                 ddq_each{j} = [ddq_each{j}, ddq_c{j}];
%             end
%             t_each = [t_each, t_c];
%             current_time = t_c(end);
%         end
%         %% 正常轨迹
%         
%         [theta_seg, dq_seg, ddq_seg, t_seg] = liuci_single(q_start, q_end, k_seg);
% 
%         % 时间累加
%         t_seg = t_seg + current_time;
%         current_time = t_seg(end);
% 
%         % 拼接轨迹
%         for j = 1:7
%             theta_each{j} = [theta_each{j}, theta_seg{j}];
%             dq_each{j} = [dq_each{j}, dq_seg{j}];
%             ddq_each{j} = [ddq_each{j}, ddq_seg{j}];
%         end
%         t_each = [t_each, t_seg];
%     end

    theta_total{i} = theta_each;
    dq_total{i} = dq_each;
    ddq_total{i} = ddq_each;
%     t_total{i} = t_each;
    % 记录第一个个体的时间点数
    if i == 1
        t_total = t_each;
    else
       if length(t_each) ~= length(t_total)
            error("第 %d 个体轨迹点数不一致：%d ≠ %d", i, length(t_each), length(t_total));
        end
    end
end


end

