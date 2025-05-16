function [theta_seg, dq_seg, ddq_seg, t_values] = liuci_single(q_start, q_end, k_seg)

% ���ι켣�滮����������յ㡢����7��k
num_steps = 51; % ����
dt = 0.02; % ʱ�䲽��
t_values = (0:num_steps-1)*dt; % ʱ������

theta_seg = cell(7,1);
dq_seg = cell(7,1);
ddq_seg = cell(7,1);

for j = 1:7
    % ����յ�
    q0 = q_start(j);
    qf = q_end(j);
    % ���ٶ� �����ٶ� ȫ��0
    v0 = 0; vf = 0;
    a0 = 0; af = 0;

    % ���ζ���ʽϵ��
    T = t_values(end);
    A0 = q0;
    A1 = v0;
    A2 = 0.5*a0;
    A3 = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*a0 - af)*T^2) / (2*T^3);
    A4 = (30*(q0 - qf) + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T^2) / (2*T^4);
    A5 = (12*(qf - q0) - (6*vf + 6*v0)*T - (a0 - af)*T^2) / (2*T^5);
    A6 = k_seg(j); % ��Ĳ���6����

    % �켣����
    theta_seg{j} = A0 + A1*t_values + A2*t_values.^2 + A3*t_values.^3 + A4*t_values.^4 + A5*t_values.^5 + A6*t_values.^6;
    dq_seg{j} = A1 + 2*A2*t_values + 3*A3*t_values.^2 + 4*A4*t_values.^3 + 5*A5*t_values.^4 + 6*A6*t_values.^5;
    ddq_seg{j} = 2*A2 + 6*A3*t_values + 12*A4*t_values.^2 + 20*A5*t_values.^3 + 30*A6*t_values.^4;
end

end


