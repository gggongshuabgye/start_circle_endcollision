function popm_sel = Mutation(popm_sel, Pmutation, mutation_range)
mutated_population = popm_sel;
N = 3;
cutin_task_id = 2; % 假设第2号任务点是绕行切入点
for i = 1:size(popm_sel, 1)
    % 提取任务点序列部分
        A1 = mutated_population(i, 1:N); % 任务点序列从第1列开始
 
        % 提取参数 k 部分
        A2 = mutated_population(i, N+N+1:end); % 参数 k 在任务点序列之后
         % 对任务点序列进行变异
        if rand <= Pmutation
            % 生成两个随机位置
            nnper1 = randperm(length(A1));
            index11 = nnper1(1);
            index12 = nnper1(2);
            
            % 交换两个位置的元素
            temp = A1(index11);
            A1(index11) = A1(index12);
            A1(index12) = temp;
        end
        % 对参数 k 进行变异
        for j = 1:length(A2)
            if rand <= Pmutation
                % 添加随机扰动
                A2(j) = A2(j) + randn * mutation_range;
                % 限制变异后的值在 -500 到 500 之间
                A2(j) = max(min(A2(j), 500), -500);
            end
        end
         % ---------- 修正绕行标志 flag ----------
        A_flag = zeros(1, N);
        cutin_pos_A = find(A1 == cutin_task_id);
        if ~isempty(cutin_pos_A)
            A_flag(cutin_pos_A) = 1;
        end

         % 将变异后的部分重新组合到个体中
        mutated_population(i, :) = [A1,A_flag, A2];
%     for j = 1:size(popm1_sel, 2)
%         if rand <= Pmutation
%             % 鎵ц闅忔満鎵板姩
%             mutated_population(i, j) = popm1_sel(i, j) + randn * mutation_range;
%             % 闄愬埗鍙樺紓鍚庣殑鍊煎湪鑼冨洿鍐?
%             mutated_population(i, j) = max(min(mutated_population(i, j), 500), -500);
        end
    

    
popm_sel = mutated_population;
end

