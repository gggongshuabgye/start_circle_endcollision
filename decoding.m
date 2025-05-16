function xsA3=decoding(M,p3cd,popm3)
b=0;
upper_bound=pi;% upper bound of the parameter
lower_bound=-pi; % lower bound of the parameter
xsA3=zeros(M,1);
for i=1:M
    for j=1:size(popm3,2)
        %value first
        b=b+popm3(i,j)*2^(p3cd-j);% value of the binary code
    end
    b=lower_bound + b*(upper_bound-lower_bound)/(2^23-1); %the final value of the ith parameter, belonging to [lower_bound,upper_bound]
    xsA3(i)=b;
    b=0;
end