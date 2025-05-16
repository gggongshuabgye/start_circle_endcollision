function f = point_to_lineSegment(r,O, line)
%%%%%% 定义连杆线段的起点为A，终点为B，球型障碍物的圆心为O，C点为O点到连杆AB的垂足。
A = line.p1;
AB = line.p2 - line.p1;
AO = O - line.p1;
% t为比例系数
t = dot(AO, AB) / norm(AB) / norm(AB);
% 线段OC的模长
l_oc = norm(t*AB+A-O);

if l_oc >(r+0.35) %  连杆到障碍物中心的距离大于r，不发生碰撞
    f = 1;
elseif (t<0 && (norm(AB)*abs(t))^2 + l_oc^2 > (r+0.35)^2) || (t>1 && (norm(AB)*abs(t-1))^2 + l_oc^2 > (r+0.35)^2)
    f = 1;
elseif l_oc <= (r+0.35) && (t>0 && t<1)
    f = 0;
elseif (t<0 && (norm(AB)*abs(t))^2 + l_oc^2 <= (r+0.35)^2) || (t>1 && (norm(AB)*abs(t-1))^2 + l_oc^2 <= (r+0.35)^2)
    f = 0;
end
