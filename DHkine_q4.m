function EEpose_q4=DHkine_q4(qq)
QQ=[];

for i=1:size(qq,1)
    QQ=[QQ;qq{i,1}];
end
a1=0;a2=0;a3=0;a4=0;a5=0;a6=0;a7=0;
d1=0.3;d2=0;d3=0.7;d4=-0.3;d5=0.7;d6=0;d7=0.3;

size1=size(QQ,2);
for i=1:size1
    q=QQ(:,i);
    T1=[cos(q(1)) 0 sin(q(1)) a1.*cos(q(1));sin(q(1)) 0 -cos(q(1)) a1.*sin(q(1));0 1 0 d1; 0 0 0 1];
    T2=[cos(q(2)) 0 sin(q(2)) a2.*cos(q(2));sin(q(2)) 0 -cos(q(2))  a2.*sin(q(2));0 1 0 d2; 0 0 0 1];
    T3=[cos(q(3)) 0 -sin(q(3)) a3.*cos(q(3));sin(q(3)) 0 cos(q(3))  a3.*sin(q(3));0 -1 0 d3; 0 0 0 1];
    T4=[cos(q(4)) 0 sin(q(4)) a4.*cos(q(4));sin(q(4)) 0 -cos(q(4))  a4.*sin(q(4));0 1 0 d4; 0 0 0 1];
%     T5=[cos(q(5)) 0 -sin(q(5)) a5.*cos(q(5));sin(q(5)) 0 cos(q(5))  a5.*sin(q(5));0 -1 0 d5; 0 0 0 1];
%     T6=[cos(q(6)) 0 sin(q(6)) a6.*cos(q(6));sin(q(6)) 0 -cos(q(6))  a6.*sin(q(6));0 1 0 d6; 0 0 0 1];
%     T7=[cos(q(7)) -sin(q(7)) 0 a7.*cos(q(7));sin(q(7)) cos(q(7)) 0  a7.*sin(q(7));0 0 1 d7; 0 0 0 1];
    TT=T1*T2*T3*T4;
    % EE position
    EEpose_q4(1,i)=TT(1,4);
    EEpose_q4(2,i)=TT(2,4);
    EEpose_q4(3,i)=TT(3,4);
    % EE attitude
    EEpose_q4(4,i)=atan2(TT(3,2),TT(3,3));
    EEpose_q4(5,i)=asin(-TT(3,1));
    EEpose_q4(6,i)=atan2(TT(2,1),TT(1,1));
end
