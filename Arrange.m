% ��ÿ��Ԫ������С��һ�����ڵ�һ�У������Ⱥ�˳�򲻱�

function Group=Arrange(Group)
    [x y]=size(Group);
    [NO1,index]=min(Group',[],2); %�ҵ���Сֵ1
    for i=1:y
      pop=Group(:,i);
      temp1=pop([1: index(i)-1]);
      temp2=pop([index(i): x]);
      Group(:,i)=[temp2' temp1']';
    end