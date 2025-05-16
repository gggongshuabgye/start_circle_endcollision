function angle=joanli(ang)
%Limit the ang belong to[-pi,pi]
if ang>pi
    angle=ang-2*pi;
else
    angle=ang;
end
%++++
if ang<-pi
    angle=ang+2*pi;
else
end

    