clc
clear
close all
load map.mat
dbstop if error
%% 输入起点与终点
% 起点
% start = input("请输入起点: ");
% 终点
% goal=input("请输入终点: ");
start=[3,5];
goal=[8,8];
%% 标记起点与终点，绘制地图
figure(1)
imshow(map)
hold on
xlabel('x')
ylabel('y')
%标记起点与终点
% plot(start(1),500-start(2)+1,'g.','MarkerSize',20)
% plot(goal(1),500-goal(2)+1,'r.','MarkerSize',20)
plot(start(1),start(2),'g.','MarkerSize',20)
plot(goal(1),goal(2),'r.','MarkerSize',20)

%% 计算速度矩阵V与时间矩阵T
mat_V=ones(size(map));

mat_T=ones(size(map))*inf;
for i=1:size(map,1)
    for j=1:size(map,2)
        if map(i,j)==0
            mat_T(i,j)=NaN;
        end
    end
end

%初始化mat_T(start)为0
mat_T(goal(1),goal(2))=0;
%初始化Far和Known
Far=[];
Known=[];
for i=1:size(map,1)
    for j=1:size(map,2)
        if i==goal(1)&&j==goal(2)
            Known=[i,j];
        else
            Far=[Far;i,j];
        end
    end
end

%起点的邻居放入Trial并计算其T
start_adjecent=[goal+[0,1];goal+[0,-1];goal+[1,0];goal+[-1,0]];
Trial=start_adjecent;

for i=1:4
    T=get_T(start_adjecent(i,:),mat_T,mat_V);
    mat_T(start_adjecent(i,1),start_adjecent(i,2))=T;
    Trial(i,3)=T;
    % 如果p_now_adjecent是Far
        [~,ind]=ismember(Far,[start_adjecent(i,1),start_adjecent(i,2)],'rows');
        if size(find(ind==1),1)==1
            Far(find(ind==1),:)=[];
        end

end

%% while Trial is not empty
while size(Trial,1)>0
    size(Trial,1)
    sortrows(Trial,3);
    p_now=Trial(1,1:2);
    Trial(1,:)=[];
    now_adjecent=[p_now+[0,1];p_now+[0,-1];p_now+[1,0];p_now+[-1,0]];
    for i=1:4
        p_now_adjecent=now_adjecent(i,:);
        if p_now_adjecent(1)>1 && p_now_adjecent(1)<10 && ...
                p_now_adjecent(2)>1 && p_now_adjecent(2)<10 && ~isnan(mat_T(p_now_adjecent(1),p_now_adjecent(2)))
            T=get_T(p_now_adjecent,mat_T,mat_V);
        else
            continue
        end
        % 如果p_now_adjecent是Far
        [~,ind]=ismember(Far,p_now_adjecent,'rows');
        if size(find(ind==1),1)==1
            Far(find(ind==1),:)=[];
            Trial=[Trial;p_now_adjecent T];
            mat_T(p_now_adjecent(1),p_now_adjecent(2))=T;
        end
        % 如果p_now_adjecent是Trial
        [~,ind]=ismember(Trial(:,1:2),p_now_adjecent,'rows');
        if size(find(ind==1),1)==1
            temp=min(T,Trial(find(ind==1),3));
            Trial(find(ind==1),3)=temp;
            mat_T(p_now_adjecent(1),p_now_adjecent(2))=temp;
        end
    end
end











