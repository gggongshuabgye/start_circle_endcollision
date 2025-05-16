% according to the pose of EE, generate the corresponding joint configurations, 8*7
function jc=joco(pose)
% physical parameters of S-R-S robot
% % theta0=[-90 180 0 0 0 0 -90]'*pi/180;
% % % % % alph=[90 90 -90 90 -90 90 0]'*pi/180;
aa=zeros(7,1);
dd=[0.3 0 0.7 -0.3 0.7 0 0.3]';
R70=Maat(pose(1,4:6));%Attitude Matrix of EE corresponding to tge base frame
P70=pose(1:3)';%Position of the end-effector corresponding to the base frame
% % R70=T70(1:3,1:3);% Rotation matrix corresponding to the base frame
omeg0=P70-[0 0 dd(1)]'-R70*[0 0 dd(7)]';%(49)--[1.2406  0.0068    -0.4011]'
momeg=norm(omeg0);
a00=[0 0 1]';%the reference axis--Z0-axis
I=[1 0 0;0 1 0; 0 0 1];
%optimized parameter-----------------arm angle
psy=0*pi/180;%--
%----------------theta4,2 solutions
theta4_1=pi+acos((dd(3)^2+dd(5)^2+dd(4)^2-momeg^2)/(2*dd(3)*dd(5)));% (50)--5.4106
theta4_2=pi-acos((dd(3)^2+dd(5)^2+dd(4)^2-momeg^2)/(2*dd(3)*dd(5)));% (50)--0.8726
%joint angle limit,so that it belongs to [-pi,pi]
theta4_1=joanli(theta4_1);% the first case of joint 4
theta4_2=joanli(theta4_2);% the second case of joint 4

ll0=cross(omeg0,a00);%(51)--[0.0068 -1.2406  0]'
alpha=acos((dd(3)^2+momeg^2-(dd(4)^2+dd(5)^2))/(2*dd(3)*momeg));%(52)--0.4907 

ll0=ll0/norm(ll0);% unit vector 单位向量（向量除以模长）
cll0=macro(ll0);
Rl0al=I+cll0*sin(alpha)+cll0^2*(1-cos(alpha));
Ops030=-Rl0al*(omeg0/momeg);% (53)
%%%%%%%%%%%%%%%%%%%%%%% For the first case of joint 4--------------------
%For the absolute reference attitude matrix R300,when psy=0
%%%%%%%%%%%%remark:there is a "-" before dd（4）????????????????
Nps030_1=(dd(5)*sin(theta4_1)*(omeg0+(dd(3)+dd(5)*cos(theta4_1))*Ops030) ...
    +dd(4)*cross(Ops030,omeg0))/(dd(4)^2+(dd(5)*sin(theta4_1))^2);
Aps030_1=(dd(4)*(omeg0+(dd(3)+dd(5)*cos(theta4_1))*Ops030) ...
    -dd(5)*sin(theta4_1)*cross(Ops030,omeg0))/(dd(4)^2+(dd(5)*sin(theta4_1))^2);
R300_1=[Nps030_1 Ops030 Aps030_1];
%----------For R30(RP0*R300) (13)
As_1=macro(omeg0)*R300_1;
Bs_1=-(macro(omeg0))^2*R300_1;
Cs_1=(I+(macro(omeg0))^2)*R300_1;
% % %---------psy is arm angle,which is the optimized parameter
R30_1=As_1*sin(psy)+Bs_1*cos(psy)+Cs_1;
%%%%%%%%%%%%%%%%%%%%%%%Based on R300 and R30,for theta2,theta1 and theta3
%-----------theta2_1, 2 solutions
theta2_11=atan2(sqrt(R30_1(1,2)^2+R30_1(2,2)^2),R30_1(3,2));
theta2_12=atan2(-sqrt(R30_1(1,2)^2+R30_1(2,2)^2),R30_1(3,2));
%-----------theta1_1, 2 solutions
theta1_11=atan2(-R30_1(2,2)*theta2_11,-R30_1(1,2)*theta2_11);
theta1_12=atan2(-R30_1(2,2)*theta2_12,-R30_1(1,2)*theta2_12);
% % % theta1_11=atan2(-(As_1(2,2)*sin(psy)+Bs_1(2,2)*cos(psy)+Cs_1(2,2))*sin(theta2_11),-(As_1(1,2)*sin(psy)+Bs_1(1,2)*cos(psy)+Cs_1(1,2))*sin(theta2_11));
% % % theta1_12=atan2(-(As_1(2,2)*sin(psy)+Bs_1(2,2)*cos(psy)+Cs_1(2,2))*sin(theta2_12),-(As_1(1,2)*sin(psy)+Bs_1(1,2)*cos(psy)+Cs_1(1,2))*sin(theta2_12));
%-----------theta3_1, 2 solutions
theta3_11=atan2(-R30_1(3,3)*theta2_11,R30_1(3,1)*theta2_11);
theta3_12=atan2(-R30_1(3,3)*theta2_12,R30_1(3,1)*theta2_12);
% % % theta3_11=atan2(-(As_1(3,3)*sin(psy)+Bs_1(3,3)*cos(psy)+Cs_1(3,3))*sin(theta2_11),(As_1(3,1)*sin(psy)+Bs_1(3,1)*cos(psy)+Cs_1(3,1))*sin(theta2_11));
% % % theta3_12=atan2(-(As_1(3,3)*sin(psy)+Bs_1(3,3)*cos(psy)+Cs_1(3,3))*sin(theta2_12),(As_1(3,1)*sin(psy)+Bs_1(3,1)*cos(psy)+Cs_1(3,1))*sin(theta2_12));

t11=[theta1_11 theta2_11 theta3_11 theta4_1];
t12=[theta1_12 theta2_12 theta3_12 theta4_1];
% % % % % % % % % % % % % % 
% % % % % % %%%%%%%%%%%%%%%%%%%%%%% For the second case of joint 4--------------------
% % % % % % %For the absolute reference attitude matrix R300,when psy=0
Nps030_2=(dd(5)*sin(theta4_2)*(omeg0+(dd(3)+dd(5)*cos(theta4_2))*Ops030) ...
    +dd(4)*cross(Ops030,omeg0))/(dd(4)^2+(dd(5)*sin(theta4_2))^2);
Aps030_2=(dd(4)*(omeg0+(dd(3)+dd(5)*cos(theta4_2))*Ops030) ...
    -dd(5)*sin(theta4_2)*cross(Ops030,omeg0))/(dd(4)^2+(dd(5)*sin(theta4_2))^2);
R300_2=[Nps030_2 Ops030 Aps030_2];% For theta4_2, when arm angle psy=0
% % % % % % %----------For R30(RP0*R300) (13)
As_2=macro(omeg0)*R300_2;
Bs_2=-(macro(omeg0))^2*R300_2;
Cs_2=(I+(macro(omeg0))^2)*R300_2;
% %---------psy is arm angle,which is the optimized parameter
R30_2=As_2*sin(psy)+Bs_2*cos(psy)+Cs_2;
%%%%%%%%%%%%%%%%%%%%%%%Based on R300 and R30,for theta2,theta1 and theta3
%-----------theta2_2, 2 solutions
theta2_21=atan2(sqrt(R30_2(1,2)^2+R30_2(2,2)^2),R30_2(3,2));
theta2_22=atan2(-sqrt(R30_2(1,2)^2+R30_2(2,2)^2),R30_2(3,2));
%-----------theta1_2, 2 solutions
theta1_21=atan2(-R30_2(2,2)*theta2_21,-R30_2(1,2)*theta2_21);
theta1_22=atan2(-R30_2(2,2)*theta2_22,-R30_2(1,2)*theta2_22);
% % theta1_21=atan2(-(As_2(2,2)*sin(psy)+Bs_2(2,2)*cos(psy)+Cs_2(2,2))*sin(theta2_21),-(As_2(1,2)*sin(psy)+Bs_2(1,2)*cos(psy)+Cs_2(1,2))*sin(theta2_21));
% % theta1_22=atan2(-(As_2(2,2)*sin(psy)+Bs_2(2,2)*cos(psy)+Cs_2(2,2))*sin(theta2_22),-(As_2(1,2)*sin(psy)+Bs_2(1,2)*cos(psy)+Cs_2(1,2))*sin(theta2_22));
%-----------theta3_2, 2 solutions
theta3_21=atan2(-R30_2(3,3)*theta2_21,R30_2(3,1)*theta2_21);
theta3_22=atan2(-R30_2(3,3)*theta2_22,R30_2(3,1)*theta2_22);
% % theta3_21=atan2(-(As_2(3,3)*sin(psy)+Bs_2(3,3)*cos(psy)+Cs_2(3,3))*sin(theta2_21),(As_2(3,1)*sin(psy)+Bs_2(3,1)*cos(psy)+Cs_2(3,1))*sin(theta2_21));
% % theta3_22=atan2(-(As_2(3,3)*sin(psy)+Bs_2(3,3)*cos(psy)+Cs_2(3,3))*sin(theta2_22),(As_2(3,1)*sin(psy)+Bs_2(3,1)*cos(psy)+Cs_2(3,1))*sin(theta2_22));

t21=[theta1_21 theta2_21 theta3_21 theta4_2];
t22=[theta1_22 theta2_22 theta3_22 theta4_2];

%--------------------------------------------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%Based on R300 and R43 ,for theta5,theta6 and theta7
%---Rotation matrix from 3 to 4(description of 4 in 3),2 solutions
%%%%%%%%%%%%%%%%%%%%%%% For the first case, with theta4_1 
R43_1=[cos(theta4_1) 0 sin(theta4_1);sin(theta4_1) 0 -cos(theta4_1);0 1 0];
Aw_1=R70'*macro(omeg0)*R300_1*R43_1;
Bw_1=-R70'*((macro(omeg0))^2)*R300_1*R43_1;
Cw_1=R70'*(I+(macro(omeg0))^2)*R300_1*R43_1;
R47_1=Aw_1*sin(psy)+Bw_1*cos(psy)+Cw_1;
%-------------------------theta6_1, 2 solutions
theta6_11=atan2(sqrt(R47_1(1,3)^2+R47_1(2,3)^2),R47_1(3,3));
theta6_12=atan2(-sqrt(R47_1(1,3)^2+R47_1(2,3)^2),R47_1(3,3));
% % % theta6_11=atan2(sqrt((Aw_1(3,1)*sin(psy)+Bw_1(3,1)*cos(psy)+Cw_1(3,1))^2+(Aw_1(3,2)*sin(psy)+Bw_1(3,2)*cos(psy)+Cw_1(3,2))^2),Aw_1(3,3)*sin(psy)+Bw_1(3,3)*cos(psy)+Cw_1(3,3));
% % % theta6_12=atan2(-sqrt((Aw_1(3,1)*sin(psy)+Bw_1(3,1)*cos(psy)+Cw_1(3,1))^2+(Aw_1(3,2)*sin(psy)+Bw_1(3,2)*cos(psy)+Cw_1(3,2))^2),Aw_1(3,3)*sin(psy)+Bw_1(3,3)*cos(psy)+Cw_1(3,3));

% % %-------------------------theta5_1, 2 solutions
theta5_11=atan2(R47_1(3,2)*sin(theta6_11),R47_1(3,1)*sin(theta6_11));%?
theta5_12=atan2(R47_1(3,2)*sin(theta6_12),R47_1(3,1)*sin(theta6_12));
% % theta5_11=atan2((Aw_1(3,2)*sin(psy)+Bw_1(3,2)*cos(psy)+Cw_1(3,2))*sin(theta6_11),(Aw_1(3,1)*sin(psy)+Bw_1(3,1)*cos(psy)+Cw_1(3,1))*sin(theta6_11));
% % theta5_12=atan2((Aw_1(3,2)*sin(psy)+Bw_1(3,2)*cos(psy)+Cw_1(3,2))*sin(theta6_12),(Aw_1(3,1)*sin(psy)+Bw_1(3,1)*cos(psy)+Cw_1(3,1))*sin(theta6_12));
% % %-------------------------theta7_1, 2 solutions
theta7_11=atan2(R47_1(2,3)*sin(theta6_11),-R47_1(1,3)*sin(theta6_11));%?
theta7_12=atan2(R47_1(2,3)*sin(theta6_12),-R47_1(1,3)*sin(theta6_12));
% % theta7_11=atan2((Aw_1(2,3)*sin(psy)+Bw_1(2,3)*cos(psy)+Cw_1(2,3))*sin(theta6_11),-(Aw_1(1,3)*sin(psy)+Bw_1(1,3)*cos(psy)+Cw_1(1,3))*sin(theta6_11));
% % theta7_12=atan2((Aw_1(2,3)*sin(psy)+Bw_1(2,3)*cos(psy)+Cw_1(2,3))*sin(theta6_12),-(Aw_1(1,3)*sin(psy)+Bw_1(1,3)*cos(psy)+Cw_1(1,3))*sin(theta6_12));

s11=[theta5_11 theta6_11 theta7_11];%theta4_1 
s12=[theta5_12 theta6_12 theta7_12];


%%%%%%%%%%%%%%%%%%%%%%% For the second case, with theta4_2
R43_2=[cos(theta4_2) 0 sin(theta4_2);sin(theta4_2) 0 -cos(theta4_2);0 1 0];
Aw_2=R70'*macro(omeg0)*R300_2*R43_2;
Bw_2=-R70'*((macro(omeg0))^2)*R300_2*R43_2;
Cw_2=R70'*(I+(macro(omeg0))^2)*R300_2*R43_2;
R47_2=Aw_2*sin(psy)+Bw_2*cos(psy)+Cw_2;
%-------------------------theta6_2, 2 solutions
theta6_21=atan2(sqrt(R47_2(1,3)^2+R47_2(2,3)^2),R47_2(3,3));
theta6_22=atan2(-sqrt(R47_2(1,3)^2+R47_2(2,3)^2),R47_2(3,3));
% % theta6_21=atan2(sqrt((Aw_2(1,3)*sin(psy)+Bw_2(1,3)*cos(psy)+Cw_2(1,3))^2+(Aw_2(2,3)*sin(psy)+Bw_2(2,3)*cos(psy)+Cw_2(2,3))^2),Aw_2(3,3)*sin(psy)+Bw_2(3,3)*cos(psy)+Cw_2(3,3));
% % theta6_22=atan2(-sqrt((Aw_2(1,3)*sin(psy)+Bw_2(1,3)*cos(psy)+Cw_2(1,3))^2+(Aw_2(2,3)*sin(psy)+Bw_2(2,3)*cos(psy)+Cw_2(2,3))^2),Aw_2(3,3)*sin(psy)+Bw_2(3,3)*cos(psy)+Cw_2(3,3));
%-------------------------theta5_2, 2 solutions
theta5_21=atan2(R47_2(3,2)*sin(theta6_21),R47_2(3,1)*sin(theta6_21));
theta5_22=atan2(R47_2(3,2)*sin(theta6_22),R47_2(3,1)*sin(theta6_22));
% % theta5_21=atan2((Aw_2(3,2)*sin(psy)+Bw_2(3,2)*cos(psy)+Cw_2(3,2))*sin(theta6_21),(Aw_2(3,1)*sin(psy)+Bw_2(3,1)*cos(psy)+Cw_2(3,1))*sin(theta6_21));
% % theta5_22=atan2((Aw_2(3,2)*sin(psy)+Bw_2(3,2)*cos(psy)+Cw_2(3,2))*sin(theta6_22),(Aw_2(3,1)*sin(psy)+Bw_2(3,1)*cos(psy)+Cw_2(3,1))*sin(theta6_22))

%-------------------------theta7_2, 2 solutions
theta7_21=atan2(R47_2(2,3)*sin(theta6_21),-R47_2(1,3)*sin(theta6_21));
theta7_22=atan2(R47_2(2,3)*sin(theta6_22),-R47_2(1,3)*sin(theta6_22));
% % % % theta7_21=atan2((Aw_2(2,3)*sin(psy)+Bw_2(2,3)*cos(psy)+Cw_2(2,3))*sin(theta6_21),-(Aw_2(1,3)*sin(psy)+Bw_2(1,3)*cos(psy)+Cw_2(1,3))*sin(theta6_21));
% % % % theta7_22=atan2((Aw_2(2,3)*sin(psy)+Bw_2(2,3)*cos(psy)+Cw_2(2,3))*sin(theta6_22),-(Aw_2(1,3)*sin(psy)+Bw_2(1,3)*cos(psy)+Cw_2(1,3))*sin(theta6_22));

s21=[theta5_21 theta6_21 theta7_21];%theta4_2 
s22=[theta5_22 theta6_22 theta7_22];

Solu1=[t11 s11];
Solu2=[t11 s12];
Solu3=[t12 s11];
Solu4=[t12 s12];
Solu5=[t21 s21];
Solu6=[t21 s22];
Solu7=[t22 s21];
Solu8=[t22 s22];
jc=[Solu1;Solu2;Solu3;Solu4;Solu5;Solu6;Solu7;Solu8];








