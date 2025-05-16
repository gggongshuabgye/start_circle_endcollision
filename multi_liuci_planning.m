function [theta_total, dq_total, ddq_total, t_total] = multi_liuci_planning(joicon, popm, M)

% ����
% joicon: 1��N cell��ÿ��cell��1��7�̶�����
% popm: Ⱦɫ�� M �� (N + 7*(N-1))��ǰN����˳�򣬺�����ÿ��7��K
% M: ��Ⱥ��

theta_total = cell(M,1);
dq_total = cell(M,1);
ddq_total = cell(M,1);
t_total = []; 
N = length(joicon); % ���������
% total_segments = N - 1; % �ܶ���
% points_per_segment = 51; % ÿ�ι켣�Ĳ�ֵ����
% total_points = total_segments * points_per_segment; % ��ʱ����� 
  % Ԥ����ʱ�����
%  t_total = zeros(1, total_points); % ���ڴ洢���и����ʱ���
for i = 1:M
    task_seq = popm(i, 1:N); % ȡ˳��
    flag_seq = popm(i, N+1:2*N); % ���б�־
    k_all = popm(i, 2*N+1:end); % ���k
    
    % ����˳�����йؽ�����
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
        k_seg = k_all((seg-1)*7+1 : seg*7); % ÿ�ε�k
        % ---- �����켣�� ----
        [theta_seg, dq_seg, ddq_seg, t_seg] = liuci_single(q_start, q_end, k_seg);
        t_seg = t_seg + current_time;
        current_time = t_seg(end);

        for j = 1:7
            theta_each{j} = [theta_each{j}, theta_seg{j}];
            dq_each{j} = [dq_each{j}, dq_seg{j}];
            ddq_each{j} = [ddq_each{j}, ddq_seg{j}];
        end
        t_each = [t_each, t_seg];
                % ---- ����һ������㣨�յ㣩������㣬��˶κ�����й켣 ----
        rx_flag = (flag_seq(seg+1) == 1);
        if rx_flag
            [theta_c, dq_c, ddq_c, t_c] = circular_path(q_end, current_time);  % ���յ���̬����
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
%         %%  ���ж�
%         rx_flag =  (flag_seq(task_seq(seg)) == 1);  % �жϵ�ǰ������Ƿ�����
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
%         %% �����켣
%         
%         [theta_seg, dq_seg, ddq_seg, t_seg] = liuci_single(q_start, q_end, k_seg);
% 
%         % ʱ���ۼ�
%         t_seg = t_seg + current_time;
%         current_time = t_seg(end);
% 
%         % ƴ�ӹ켣
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
    % ��¼��һ�������ʱ�����
    if i == 1
        t_total = t_each;
    else
       if length(t_each) ~= length(t_total)
            error("�� %d ����켣������һ�£�%d �� %d", i, length(t_each), length(t_total));
        end
    end
end


end

