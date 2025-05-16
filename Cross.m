function popm_sel = Cross(popm_sel, Pc)
offspring_popm_sel = popm_sel;
NSel = size(popm_sel, 1);
N = 3;
cutin_task_id = 2; % 假设第2号任务点是绕行切入点
for i = 1:2:NSel-mod(NSel,2)   %濡傛灉绉嶇兢鐨勫ぇ灏忔槸濂囨暟锛屽氨鍑忔帀1
    if rand <= Pc
        % 随机选择两个个体进行交叉
            A = popm_sel(i, :);
            B = popm_sel(i+1, :);
 
            % 提取任务点序列部分
            A1 = A(1, 1:N); % 任务点序列从第1列开始
            B1 = B(1, 1:N);
 
            % 提取参数 k 部分
            A2 = A(1, N+N+1:end); % 参数 k 在任务点序列之后
            B2 = B(1, N+N+1:end);
 
            % 对任务点序列进行交叉
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
 
            % 对参数 k 进行交叉
            crossover_point_k = randi(length(A2));
            A2_part1 = A2(1, 1:crossover_point_k);
            A2_part2 = A2(1, crossover_point_k+1:end);
            B2_part1 = B2(1, 1:crossover_point_k);
            B2_part2 = B2(1, crossover_point_k+1:end);
 
            % 交换参数 k 的部分
            A2 = [B2_part1, A2_part2];
            B2 = [A2_part1, B2_part2];
 % ---------- 修正绕行标志 flag ----------
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

            % 将交叉后的部分重新组合到个体中
            offspring_popm_sel(i, :) = [A1,A_flag, A2];
            offspring_popm_sel(i+1, :) = [B1, B_flag, B2];
%         % 閫夋嫨浜ゅ弶鐐?
%         crossover_point = randi(size(popm1_sel, 2));
%         % 鎵ц鍗曠偣浜ゅ弶
%         offspring_popm1_sel(i, crossover_point:end) = popm1_sel(i+1, crossover_point:end);
%         offspring_popm1_sel(i+1, crossover_point:end) = popm1_sel(i, crossover_point:end);
    end
   
end
 popm_sel = offspring_popm_sel;
% popm1_sel=offspring_popm1_sel;





