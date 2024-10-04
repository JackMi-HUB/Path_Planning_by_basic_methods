VehicleAnimation2(x,y,th,Configure,Vehicle);
function VehicleAnimation2(x,y,theta,cfg,veh)
    global param
    sz=get(0,'screensize');
    figure('outerposition',sz);
    videoFWriter = VideoWriter('Parking.mp4','MPEG-4');
    open(videoFWriter);
    ObstList = cfg.ObstList;
    scatter(ObstList(:,1),ObstList(:,2),10,'r') % 画散点图
    hold on
    axis equal
    xlim([cfg.MINX,cfg.MAXX]);
    ylim([cfg.MINY,cfg.MAXY]);
    Arrow([param(1), param(2)], [param(1) + cos(param(3)), param(2) + sin(param(3))], 'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    Arrow([param(4), param(5)], [param(4) + cos(param(6)), param(5)+ sin(param(6))],  'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    drawnow
    V = CreateVehiclePolygon(param(1), param(2), param(3),veh);
    plot(V.x, V.y, 'g--', 'LineWidth', 2);

    V = CreateVehiclePolygon(param(4), param(5), param(6),veh);
    plot(V.x, V.y, 'r--', 'LineWidth', 2);
    
    X = [];
    Y = [];
    px = x(1);
    py = y(1);
    X = [X,px];
    Y = [Y,py];
    pth = theta(1);
    h3 = plot(X,Y,'b'); % 规划出来的轨迹，蓝色曲线  
    [vehx,vehy] = getVehTran(px,py,pth,veh); % 根据后轴中心的位姿计算车辆边框的位姿
    [wheelx1,wheely1] = Wheel_point1(px,py,pth,veh,phy);
    [wheelx2,wheely2] = Wheel_point2(px,py,pth,veh,phy);    
    [wheelx3,wheely3] = Wheel_point3(px,py,pth,veh);
    [wheelx4,wheely4] = Wheel_point4(px,py,pth,veh);
    h1 = plot(vehx,vehy,'k'); % 车辆边框
    h2 = plot(px,px,'rx','MarkerSize',10); % 车辆后轴中心
    h4 = plot(wheelx3,wheely3,'k');
    h5 = plot(wheelx4,wheely4,'k');
    h6 = plot(wheelx1,wheely1,'k');
    h7 = plot(wheelx2,wheely2,'k');   
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2:length(theta)
        px = x(i);
        py = y(i);
        X = [X,px];
        Y = [Y,py];
        pth = theta(i);
        [vehx,vehy] = getVehTran(px,py,pth,veh);
        [wheelx1,wheely1] = Wheel_point1(px,py,pth,veh,phy);
        [wheelx2,wheely2] = Wheel_point2(px,py,pth,veh,phy);
        [wheelx3,wheely3] = Wheel_point3(px,py,pth,veh);
        [wheelx4,wheely4] = Wheel_point4(px,py,pth,veh);
        h1.XData = vehx; %更新h1图像句柄,把车辆边框四个角点的x坐标添加进去
        h1.YData = vehy;
        h2.XData = px; %更新h2图像句柄,把车辆边框四个角点的y坐标添加进去
        h2.YData = py;
        h3.XData = X;
        h3.YData = Y;
        h4.XData = wheelx3; 
        h4.YData = wheely3;    
        h5.XData = wheelx4; 
        h5.YData = wheely4;
        h6.XData = wheelx1; 
        h6.YData = wheely1;    
        h7.XData = wheelx2; 
        h7.YData = wheely2;        
        img = getframe(gcf);%只提取图窗里面的内容，gcf是图窗句柄
        writeVideo(videoFWriter,img);
%         pause(0.005)
    end
    close(videoFWriter);
end

 % 根据后轴中心的位姿计算车辆边框的位姿
function [x,y] = getVehTran(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [LF, W/2]; % 左前方角点
    Cornerfr = [LF, -W/2]; % 右前方角点
    Cornerrl = [-LB, W/2]; % 左后方角点
    Cornerrr = [-LB, -W/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function [x,y] = Wheel_point1(x,y,theta,veh,phy)
    W = veh.W;
    WB = veh.WB;
    wheel_width = 0.4;
    wheel_length = 1;
    theta = theta+phy;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [WB+wheel_length/2, W/2+wheel_width/2]; % 左前方角点
    Cornerfr = [WB+wheel_length/2, W/2-wheel_width/2]; % 右前方角点
    Cornerrl = [WB-wheel_length/2, W/2+wheel_width/2]; % 左后方角点
    Cornerrr = [WB-wheel_length/2, W/2-wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function [x,y] = Wheel_point2(x,y,theta,veh,phy)
    W = veh.W;
    WB = veh.WB;
    wheel_width = 0.4;
    wheel_length = 1;
    theta = theta+phy;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [WB+wheel_length/2, -W/2+wheel_width/2]; % 左前方角点
    Cornerfr = [WB+wheel_length/2, -W/2-wheel_width/2]; % 右前方角点
    Cornerrl = [WB-wheel_length/2, -W/2+wheel_width/2]; % 左后方角点
    Cornerrr = [WB-wheel_length/2, -W/2-wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

%构建四个轮子(左后)
function [x,y] = Wheel_point3(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    wheel_width = 0.4;
    wheel_length = 1;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [wheel_length/2, W/2+wheel_width/2]; % 左前方角点
    Cornerfr = [wheel_length/2, W/2-wheel_width/2]; % 右前方角点
    Cornerrl = [-wheel_length/2, W/2+wheel_width/2]; % 左后方角点
    Cornerrr = [-wheel_length/2, W/2-wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

%构建四个轮子(左后)
function [x,y] = Wheel_point4(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    wheel_width = 0.4;
    wheel_length = 1;
    
    % 车辆的边框由四个角点确定
    Cornerfl = [wheel_length/2, -W/2+wheel_width/2]; % 左前方角点
    Cornerfr = [wheel_length/2, -W/2-wheel_width/2]; % 右前方角点
    Cornerrl = [-wheel_length/2, -W/2+wheel_width/2]; % 左后方角点
    Cornerrr = [-wheel_length/2, -W/2-wheel_width/2]; % 右后方角点
    Pos = [x,y]; % 后轴中心坐标
    dcm = angle2dcm(-theta, 0, 0); % 计算四个角点的旋转矩阵,由于是刚体的一部分，旋转矩阵相同，将角度转换为方向余弦矩阵，旋转顺序是ZYX
    
    tvec = dcm*[Cornerfl';0]; % 旋转变换，Cornerfl旋转后形成的列向量，位置向量3*1，最后一个是z坐标
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % 平移变换
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % 返回车辆边框四个角点的x,y坐标
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function V = CreateVehiclePolygon(x, y, theta,Vehicle)
cos_theta = cos(theta);
sin_theta = sin(theta);
vehicle_half_width = Vehicle.W * 0.5;
AX = x + (Vehicle.LF) * cos_theta - vehicle_half_width * sin_theta;
BX = x + (Vehicle.LF) * cos_theta + vehicle_half_width * sin_theta;
CX = x - Vehicle.LB * cos_theta + vehicle_half_width * sin_theta;
DX = x - Vehicle.LB * cos_theta - vehicle_half_width * sin_theta;
AY = y + (Vehicle.LF) * sin_theta + vehicle_half_width * cos_theta;
BY = y + (Vehicle.LF) * sin_theta - vehicle_half_width * cos_theta;
CY = y - Vehicle.LB * sin_theta - vehicle_half_width * cos_theta;
DY = y - Vehicle.LB * sin_theta + vehicle_half_width * cos_theta;
V.x = [AX, BX, CX, DX, AX];
V.y = [AY, BY, CY, DY, AY];
end