clc
clear
close all
load mat_T.mat
load map.mat
dbstop if error
tic
% %% 输入起点与终点
% % 起点
% % start = input("请输入起点: ");
% 终点
% goal=input("请输入终点: ");
start=[50,20];
goal=[150,190];
max_vel = 2.5;
%% 计算速度矩阵V与时间矩阵T
mat_V=ones(size(map));

mat_T=ones(size(map))*inf;
for i=1:size(map,1)
    for j=1:size(map,2)
        if map(i,j)==0
            mat_T(i,j)=NaN;%定义障碍物所在处为无效值
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
            Known=[i,j];%设定起点（实际中为终点）为不可更改点
        else
            Far=[Far;i,j];
        end
    end
end

%起点的邻居放入Trial并计算其T
start_adjecent=[goal+[0,1];goal+[0,-1];goal+[1,0];goal+[-1,0]];
Trial=start_adjecent;

for i=1:4
    distance = pdist([start_adjecent(i,:); start], 'euclidean');
    T=get_T(start_adjecent(i,:),mat_T,mat_V);
    mat_T(start_adjecent(i,1),start_adjecent(i,2))=T;
    Trial(i,3)=T+distance/max_vel;%将Trial矩阵进行列扩展，增加时间变量；并加入启发式，注意不要影响到mat_T
    % 如果p_now_adjecent是Far
        [~,ind]=ismember(Far,[start_adjecent(i,1),start_adjecent(i,2)],'rows');%返回行检索
        if size(find(ind==1),1)==1
            Far(find(ind==1),:)=[];%将adjacent从far中删除，表示已经扩展过该点
        end

end

%% while Trial is not empty
while size(Trial,1)>0
    %size(Trial,1);
    sortrows(Trial,3);%对时间列进行排序
    p_now=Trial(1,1:2);
    Trial(1,:)=[];
    now_adjecent=[p_now+[0,1];p_now+[0,-1];p_now+[1,0];p_now+[-1,0]];%对耗时最短的位置进行下一步搜索
    for i=1:4
        p_now_adjecent=now_adjecent(i,:);
        if p_now_adjecent(1)>1 && p_now_adjecent(1)<200 && ...
                p_now_adjecent(2)>1 && p_now_adjecent(2)<200 && ~isnan(mat_T(p_now_adjecent(1),p_now_adjecent(2)))
            distance = pdist([p_now_adjecent; start], 'euclidean');
            T=get_T(p_now_adjecent,mat_T,mat_V);
        else
            continue
        end
        % 如果p_now_adjecent是Far
        [~,ind]=ismember(Far,p_now_adjecent,'rows');
        if size(find(ind==1),1)==1
            Far(find(ind==1),:)=[];
            Trial=[Trial;p_now_adjecent T+distance/max_vel];%加入启发式,用T*代替原有的单T
            mat_T(p_now_adjecent(1),p_now_adjecent(2))=T;
        end
        % 如果p_now_adjecent是Trial
        [~,ind]=ismember(Trial(:,1:2),p_now_adjecent,'rows');
        if size(find(ind==1),1)==1%等同于判断上述是否找到了相同的列元素
            temp=min(T,Trial(find(ind==1),3));
            Trial(find(ind==1),3)=temp;
            mat_T(p_now_adjecent(1),p_now_adjecent(2))=temp;
        end
    end
end
using_time = toc

%% 势值图
mat_T_plot=mat_T;
for i=1:size(mat_T,1)
    for j=1:size(mat_T,2)
        if isnan(mat_T(i,j)) || mat_T(i,j)==inf
            mat_T(i,j)=inf;
            mat_T_plot(i,j)=0;
        end
    end
end
imagesc(mat_T_plot)
colormap('jet')
%% 起点与终点
hold on
% start=[30,50];
% goal=[180,180];
title('y')
ylabel('x')
plot(start(2),start(1),'g.','MarkerSize',20)
plot(goal(2),goal(1),'r.','MarkerSize',20)

%% 梯度下降法求路径
path=[start];
while path(end,1)~=goal(1) || path(end,2)~=goal(2)
    node=path(end,:);
%     if node(1)==112
%         1
%     end
    temp=[node(1)-1,node(2); ...
        node(1)+1,node(2); ...
        node(1),node(2)-1; ...
        node(1),node(2)+1; ...
        node(1)-1,node(2)-1; ...
        node(1)-1,node(2)+1; ...
        node(1)+1,node(2)-1; ...
        node(1)+1,node(2)+1; ...
        ];
    temp_T=[mat_T(node(1)-1,node(2)); ...
        mat_T(node(1)+1,node(2)); ...
        mat_T(node(1),node(2)-1); ...
        mat_T(node(1),node(2)+1); ...
        mat_T(node(1)-1,node(2)-1); ...
        mat_T(node(1)-1,node(2)+1); ...
        mat_T(node(1)+1,node(2)-1); ...
        mat_T(node(1)+1,node(2)+1); ...
        ];
    index_temp=find(temp_T==min(temp_T));
    index=index_temp(1);
    path=[path;temp(index,:)];
end
hold on
plot(path(:,2),path(:,1),'g-','LineWidth',2)


%% 标记起点与终点，绘制地图
figure(2)
imshow(map)
hold on
title('y')
ylabel('x')
%标记起点与终点
plot(start(2),start(1),'g.','MarkerSize',20)
plot(goal(2),goal(1),'r.','MarkerSize',20)
hold on
plot(path(:,2),path(:,1),'g-','LineWidth',2)

%% 单独势场图
figure
% 颜色分布范围在0-1
mat_T_plot_0_1=mat_T_plot/max(max(mat_T_plot));
imagesc(mat_T_plot_0_1)
colormap('jet')
