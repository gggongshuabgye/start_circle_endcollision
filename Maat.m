% the Rotation matrix of EE attitude corresponding to the base frame
function R70=Maat(att) 

a1=att(1);
b1=att(2);
c1=att(3);
Rx=[1 0 0; 0 cos(a1) -sin(a1);0 sin(a1) cos(a1)];
Ry=[cos(b1) 0 sin(b1);0 1 0;-sin(b1) 0 cos(b1)];
Rz=[cos(c1) -sin(c1) 0;sin(c1) cos(c1) 0;0 0 1];
R70=Rz*Ry*Rx;