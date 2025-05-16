function [theta,dq,ddq,t_values] = liuci_planning(joicon,k,M)
joc = zeros(2, 7);
for i=1:size(joicon,2)
    joc(i,:) = joicon{1,i}; 
end
joc_1=joc';
N1=size(joc_1,1); 
N2=size(joicon,2); 
omega=zeros(7,2);
alpha=zeros(7,2);
a0=cell(M,1);a1=cell(M,1);a2=cell(M,1);
a3=cell(M,1);a4=cell(M,1);a5=cell(M,1);

for l=1:M
    a0{l} = zeros(N1, N2-1);
    a1{l} = zeros(N1, N2-1);
    a2{l} = zeros(N1, N2-1);
    a3{l} = zeros(N1, N2-1);
    a4{l} = zeros(N1, N2-1);
    a5{l} = zeros(N1, N2-1);
    for i=1:N1
        t=1;
        for j=1:N2-1
            a0{l}(i)=joc_1(i,j);
            a1{l}(i)=omega(i,j);  
            a2{l}(i)=1.5*alpha(i,j);
            a3{l}(i)=(20*(joc_1(i,j+1)-joc_1(i,j))-(8*omega(i,j+1)+omega(i,j))*t-(3*alpha(i,j)-alpha(i,j+1))*t.^2-2*k(l,i)*t.^6)/(2*t.^3);
            a4{l}(i)=(15*(joc_1(i,j)-joc_1(i,j+1))+(7*omega(i,j+1)+8*omega(i,j))*t+(0.5*alpha(i,j)-alpha(i,j+1))*t.^2+3*k(l,i)*t.^6)/(t.^4);
            a5{l}(i)=(12*(joc_1(i,j+1)-joc_1(i,j))-(6*omega(i,j+1)+6*omega(i,j))*t-(alpha(i,j)-alpha(i,j+1))*t.^2-6*k(l,i)*t.^6)/(2*t.^5);
        end
    end
end

theta=cell(M,1); 
dq=cell(M,1); 
ddq=cell(M,1); 
% 设置时间间隔和时间步数
time_interval = 0.02;
num_steps = 51; 
t_values = zeros(1, num_steps);
for l=1:M
    theta{l}=cell(N1, N2-1);
    dq{l}=cell(N1, N2-1);
    ddq{l}=cell(N1, N2-1);
    for j=1:N1 % N1--number of joints
        theta{l}{j} = zeros(1, num_steps);
        for step = 1:num_steps 
            tt = (step-1) * time_interval; %当前时间
            t_values(step) = tt;
            theta{l}{j}(step) = a0{l}(j)+a1{l}(j)*tt+a2{l}(j)*tt.^2+a3{l}(j)*tt.^3+a4{l}(j)*tt.^4+a5{l}(j)*tt.^5+k(l,j)*tt.^6;
            dq{l}{j}(step) = a1{l}(j)+2*a2{l}(j)*tt+3*a3{l}(j)*tt.^2+4*a4{l}(j)*tt.^3+5*a5{l}(j)*tt.^4+6*k(l,j)*tt.^5;
            ddq{l}{j}(step) = 2*a2{l}(j)+6*a3{l}(j)*tt+12*a4{l}(j)*tt.^2+20*a5{l}(j)*tt.^3+30*k(l,j)*tt.^4;
        end 
    end
end







