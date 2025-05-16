function popm_sel = Cross(popm_sel, Pc)
offspring_popm_sel = popm_sel;
NSel = size(popm_sel, 1);
N = 3;
cutin_task_id = 2; % �����2������������������
for i = 1:2:NSel-mod(NSel,2)   %如果种群的大小是奇数，就减掉1
    if rand <= Pc
        % ���ѡ������������н���
            A = popm_sel(i, :);
            B = popm_sel(i+1, :);
 
            % ��ȡ��������в���
            A1 = A(1, 1:N); % ��������дӵ�1�п�ʼ
            B1 = B(1, 1:N);
 
            % ��ȡ���� k ����
            A2 = A(1, N+N+1:end); % ���� k �����������֮��
            B2 = B(1, N+N+1:end);
 
            % ����������н��н���
            L = length(A1);
            if L < 10
                W = L;
            elseif ((L/10)-floor(L/10)) >= rand && L > 10
                W = ceil(L/10) + 8;
            else
                W = floor(L/10) + 8;
            end
            p = unidrnd(L-W+1);
            for j = 1:W
                x = find(A1 == B1(1, p+j-1));
                y = find(B1 == A1(1, p+j-1));
                [A1(1, p+j-1), B1(1, p+j-1)] = exchange(A1(1, p+j-1), B1(1, p+j-1));
                if A1(1, x) ~= 0 && B1(1, y) ~= 0
                    [A1(1, x), B1(1, y)] = exchange(A1(1, x), B1(1, y));
                end
            end
 
            % �Բ��� k ���н���
            crossover_point_k = randi(length(A2));
            A2_part1 = A2(1, 1:crossover_point_k);
            A2_part2 = A2(1, crossover_point_k+1:end);
            B2_part1 = B2(1, 1:crossover_point_k);
            B2_part2 = B2(1, crossover_point_k+1:end);
 
            % �������� k �Ĳ���
            A2 = [B2_part1, A2_part2];
            B2 = [A2_part1, B2_part2];
 % ---------- �������б�־ flag ----------
        A_flag = zeros(1, N);
        B_flag = zeros(1, N);
        cutin_pos_A = find(A1 == cutin_task_id);
        cutin_pos_B = find(B1 == cutin_task_id);
        if ~isempty(cutin_pos_A)
            A_flag(cutin_pos_A) = 1;
        end
        if ~isempty(cutin_pos_B)
            B_flag(cutin_pos_B) = 1;
        end

            % �������Ĳ���������ϵ�������
            offspring_popm_sel(i, :) = [A1,A_flag, A2];
            offspring_popm_sel(i+1, :) = [B1, B_flag, B2];
%         % 选择交叉�?
%         crossover_point = randi(size(popm1_sel, 2));
%         % 执行单点交叉
%         offspring_popm1_sel(i, crossover_point:end) = popm1_sel(i+1, crossover_point:end);
%         offspring_popm1_sel(i+1, crossover_point:end) = popm1_sel(i, crossover_point:end);
    end
   
end
 popm_sel = offspring_popm_sel;
% popm1_sel=offspring_popm1_sel;





