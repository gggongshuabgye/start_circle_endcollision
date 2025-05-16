function circle_points = generate_circle_points(center, radius, normal, n_points)
    % ��һ��������
    normal = normal / norm(normal);


    % �Զ�ѡ���뷨������ƽ�е���������
    arbitrary = rand(1,3);
    while abs(dot(arbitrary, normal)) > 0.99
        arbitrary = rand(1,3);
    end

    % ʹ�ò�������� normal ��ֱ��������λ����
    v1 = cross(normal, arbitrary);
    v1 = v1 / norm(v1);
    v2 = cross(normal, v1);

    % ����Բ�ϵĵ�
    theta = linspace(0, 2*pi, n_points+1);
    theta(end) = []; % ȥ�����һ��������ظ�

    circle_points = zeros(n_points, 3);
    for i = 1:n_points
        circle_points(i, :) = center + radius * (cos(theta(i)) * v1 + sin(theta(i)) * v2);
    end
end


