function [p_x,p_y,p_z] = DHkine(q1)

a1=0;a2=0;a3=0;a4=0;a5=0;a6=0;a7=0;
d1=0.3;d2=0;d3=0.7;d4=-0.3;d5=0.7;d6=0;d7=0.3;

T01=[cos(q1(1)) 0 sin(q1(1)) a1.*cos(q1(1));sin(q1(1)) 0 -cos(q1(1)) a1.*sin(q1(1));0 1 0 d1; 0 0 0 1];% 0 to 1
T12=[cos(q1(2)) 0 sin(q1(2)) a2.*cos(q1(2));sin(q1(2)) 0 -cos(q1(2))  a2.*sin(q1(2));0 1 0 d2; 0 0 0 1];% 1 to 2
T23=[cos(q1(3)) 0 -sin(q1(3)) a3.*cos(q1(3));sin(q1(3)) 0 cos(q1(3))  a3.*sin(q1(3));0 -1 0 d3; 0 0 0 1];% 2 to 3
T34=[cos(q1(4)) 0 sin(q1(4)) a4.*cos(q1(4));sin(q1(4)) 0 -cos(q1(4))  a4.*sin(q1(4));0 1 0 d4; 0 0 0 1];% 3 to 4
T45=[cos(q1(5)) 0 -sin(q1(5)) a5.*cos(q1(5));sin(q1(5)) 0 cos(q1(5))  a5.*sin(q1(5));0 -1 0 d5; 0 0 0 1];% 4 to 5
T56=[cos(q1(6)) 0 sin(q1(6)) a6.*cos(q1(6));sin(q1(6)) 0 -cos(q1(6))  a6.*sin(q1(6));0 1 0 d6; 0 0 0 1];% 5 to 6
T67=[cos(q1(7)) -sin(q1(7)) 0 a7.*cos(q1(7));sin(q1(7)) cos(q1(7)) 0  a7.*sin(q1(7));0 0 1 d7; 0 0 0 1];% 6 to 7
TT=T01*T12*T23*T34*T45*T56*T67;
% EE position
p_x=TT(1,4);
p_y=TT(2,4);
p_z=TT(3,4);


