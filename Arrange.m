% 把每列元素中最小的一个放在第一行，其余先后顺序不变

function Group=Arrange(Group)
    [x y]=size(Group);
    [NO1,index]=min(Group',[],2); %找到最小值1
    for i=1:y
      pop=Group(:,i);
      temp1=pop([1: index(i)-1]);
      temp2=pop([index(i): x]);
      Group(:,i)=[temp2' temp1']';
    end