function circle_points = generate_circle_points(center, radius, normal, n_points)
    % 归一化法向量
    normal = normal / norm(normal);


    % 自动选择与法向量不平行的任意向量
    arbitrary = rand(1,3);
    while abs(dot(arbitrary, normal)) > 0.99
        arbitrary = rand(1,3);
    end

    % 使用叉乘生成与 normal 垂直的两个单位向量
    v1 = cross(normal, arbitrary);
    v1 = v1 / norm(v1);
    v2 = cross(normal, v1);

    % 构造圆上的点
    theta = linspace(0, 2*pi, n_points+1);
    theta(end) = []; % 去掉最后一个点避免重复

    circle_points = zeros(n_points, 3);
    for i = 1:n_points
        circle_points(i, :) = center + radius * (cos(theta(i)) * v1 + sin(theta(i)) * v2);
    end
end


