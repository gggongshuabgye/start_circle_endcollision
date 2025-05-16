function popm_sel = Mutation(popm_sel, Pmutation, mutation_range)
mutated_population = popm_sel;
N = 3;
cutin_task_id = 2; % �����2������������������
for i = 1:size(popm_sel, 1)
    % ��ȡ��������в���
        A1 = mutated_population(i, 1:N); % ��������дӵ�1�п�ʼ
 
        % ��ȡ���� k ����
        A2 = mutated_population(i, N+N+1:end); % ���� k �����������֮��
         % ����������н��б���
        if rand <= Pmutation
            % �����������λ��
            nnper1 = randperm(length(A1));
            index11 = nnper1(1);
            index12 = nnper1(2);
            
            % ��������λ�õ�Ԫ��
            temp = A1(index11);
            A1(index11) = A1(index12);
            A1(index12) = temp;
        end
        % �Բ��� k ���б���
        for j = 1:length(A2)
            if rand <= Pmutation
                % �������Ŷ�
                A2(j) = A2(j) + randn * mutation_range;
                % ���Ʊ�����ֵ�� -500 �� 500 ֮��
                A2(j) = max(min(A2(j), 500), -500);
            end
        end
         % ---------- �������б�־ flag ----------
        A_flag = zeros(1, N);
        cutin_pos_A = find(A1 == cutin_task_id);
        if ~isempty(cutin_pos_A)
            A_flag(cutin_pos_A) = 1;
        end

         % �������Ĳ���������ϵ�������
        mutated_population(i, :) = [A1,A_flag, A2];
%     for j = 1:size(popm1_sel, 2)
%         if rand <= Pmutation
%             % 执行随机扰动
%             mutated_population(i, j) = popm1_sel(i, j) + randn * mutation_range;
%             % 限制变异后的值在范围�?
%             mutated_population(i, j) = max(min(mutated_population(i, j), 500), -500);
        end
    

    
popm_sel = mutated_population;
end

