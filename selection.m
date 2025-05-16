function [popm1_sel,fit_sel] = selection(fit_except_0,k_except_0)
% 将�?�应度�?�转换成相反�?
reversed_fit_except_0 = -fit_except_0;
% 计算选择概率
selection_probabilities = reversed_fit_except_0 / sum(reversed_fit_except_0);
population_size = size(k_except_0,1);
popm1_sel = zeros(population_size, size(k_except_0,2));
fit_sel = zeros(population_size,1);
selected_index = zeros(population_size,1);
% 使用轮盘赌算法进行�?�择
for i = 1:population_size
    selected_index_1 = find(rand <= cumsum(selection_probabilities), 1);
    popm1_sel(i, :) = k_except_0(selected_index_1, :);
    selected_index(i) = selected_index_1;
%     fit_sel(i,:) = 
end

for i = 1:population_size
    fit_sel(i) = fit_except_0(selected_index(i));
end
