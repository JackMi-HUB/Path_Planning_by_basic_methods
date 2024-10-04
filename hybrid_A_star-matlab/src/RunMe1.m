clear; close all; clc
ObstList = []; % Obstacle point list
global param
% for i = -25:25
%     ObstList(end+1,:) = [i,30];
% end
% for i = -10:10
%     ObstList(end+1,:) = [i, 0];
% end
% for i = -25:-10
%     ObstList(end+1,:) = [i, 5];
% end
% for i = 10:25
%     ObstList(end+1,:) = [i, 5];
% end
% for i = 0:5
%     ObstList(end+1,:) = [10, i];
% end
% for i = 0:5
%     ObstList(end+1,:) = [-10, i];
% end
for i = -26:26
    ObstList(end+1,:) = [i,26];
end
for i = -26:26
    ObstList(end+1,:) = [i,-26];
end

ObstLine = []; % Park lot line for collision check
% tLine = [-25, 30 , 25, 30]; %start_x start_y end_x end_y
% ObstLine(end+1,:) = tLine;
% tLine = [-25, 5, -10, 5];
% ObstLine(end+1,:) = tLine;
% tLine = [-10, 5, -10, 0];
% ObstLine(end+1,:) = tLine;
% tLine = [-10, 0, 10, 0];
% ObstLine(end+1,:) = tLine;
% tLine = [10, 0, 10, 5];
% ObstLine(end+1,:) = tLine;
% tLine = [10, 5, 25, 5];
% ObstLine(end+1,:) = tLine;
% tLine = [-25, 5, -25, 30];
% ObstLine(end+1,:) = tLine;
% tLine = [25, 5, 25, 30];
% ObstLine(end+1,:) = tLine;
tLine = [-26, 26 , 26, 26]; %start_x start_y end_x end_y
ObstLine(end+1,:) = tLine;
tLine = [-26, -26, 26, -26];
ObstLine(end+1,:) = tLine;
x=[];
y=[];
th=[];
D=[];
delta=[];
Vehicle.WB = 3.7;  % [m] wheel base: rear to front steer
Vehicle.W = 2.6; % [m] width of vehicle
Vehicle.LF = 4.5; % [m] distance from rear to vehicle front end of vehicle
Vehicle.LB = 1.0; % [m] distance from rear to vehicle back end of vehicle
Vehicle.MAX_STEER = 0.6; % [rad] maximum steering angle 
Vehicle.MIN_CIRCLE = Vehicle.WB/tan(Vehicle.MAX_STEER); % [m] mininum steering circle radius,roughly calculate

% ObstList and ObstLine
Configure.ObstList = ObstList;
Configure.ObstLine = ObstLine;

% Motion resolution define
Configure.MOTION_RESOLUTION = 0.1; % [m] path interporate resolution, stand velocity？
Configure.N_STEER = 20.0; % number of steer command
Configure.EXTEND_AREA = 0; % [m] map extend length
Configure.XY_GRID_RESOLUTION = 2.0; % [m]
Configure.YAW_GRID_RESOLUTION = deg2rad(15.0); % [rad]
% Grid bound
Configure.MINX = min(ObstList(:,1))-Configure.EXTEND_AREA;
Configure.MAXX = max(ObstList(:,1))+Configure.EXTEND_AREA;
Configure.MINY = min(ObstList(:,2))-Configure.EXTEND_AREA;
Configure.MAXY = max(ObstList(:,2))+Configure.EXTEND_AREA;
Configure.MINYAW = -pi;
Configure.MAXYAW = pi;
% Cost related define
Configure.SB_COST = 0; % switch back penalty cost
Configure.BACK_COST = 1.5; % backward penalty cost
Configure.STEER_CHANGE_COST = 1.5; % steer angle change penalty cost
Configure.STEER_COST = 1.5; % steer angle change penalty cost
Configure.H_COST = 10; % Heuristic cost
% 
% Start = [-15, 13, 0];
% End = [0, 13, pi];

Start = [-2.68965517241379,	9.86206896551724,	0];
End = [1.49425287356322,	9.86206896551724,	0];

% 使用完整约束有障碍情况下用A*搜索的最短路径为hybrid A*的启发值
ObstMap = GridAStar(Configure.ObstList,End,Configure.XY_GRID_RESOLUTION);
Configure.ObstMap = ObstMap;
%cla %  从当前坐标区删除包含可见句柄的所有图形对象。
% param = [-17.0344827586207	-3.08812260536398	0.785400000000000	-7.27203065134100	5.67816091954023	0.785398163397448
% -7.27203065134100	5.67816091954023	0.785398163397448	-4.48275862068966	8.46743295019157	0.785398163397448
% -4.48275862068966	8.46743295019157	0.785398163397448	-2.68965517241379	9.86206896551724	0
% -2.68965517241379	9.86206896551724	0	1.49425287356322	9.86206896551724	0
% 1.49425287356322	9.86206896551724	0	9.46360153256705	10.2605363984674	-0.785398163397448
% 9.46360153256705	10.2605363984674	-0.785398163397448	11.0574712643678	9.86206896551724	0
% 11.0574712643678	9.86206896551724	0	16.8352490421456	9.86206896551724	-1.57080000000000];
param =[ 0	5	0	10	15	pi/2];
% [x,y,th,D,delta] = HybridAStar(Start,End,Vehicle,Configure);
figure(1)
axis equal; box on; grid on; axis([-26 26 -26 26]);
set(gcf,'outerposition',get(0,'screensize'));
hold on;
for i=1:size(param,1)
    [x0,y0,th0,D0,delta0] = HybridAStar(param(i,1:3),param(i,4:6),Vehicle,Configure);
    plot(x0,y0,'r'); drawnow
    x = [x,x0];
    y = [y,y0];
    th = [th,th0];
end
[x, y, th, v, a, phy, w, tf] = ResamplePath(x0, y0, th0);
% GridAStar(ObstList,End,2);
if isempty(x)
    disp("Failed to find path!")
else
    VehicleAnimation(x,y,th,Configure,Vehicle)
end