param =[ -17.0344827586207	-3.08812260536398	0.785400000000000	-7.27203065134100	5.67816091954023	0.785398163397448];
VehicleAnimation1(x,y,th,Configure,Vehicle);
function VehicleAnimation1(x,y,theta,cfg,veh)
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
    Arrow([-17.0344827586207, -3.08812260536398], [-17.0344827586207 + cos(0.7854), -3.08812260536398 + sin(0.7854)], 'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    Arrow([-7.27203065134100, 5.67816091954023], [-7.27203065134100 + cos(0.785398163397448), 5.67816091954023+ sin(0.785398163397448)],  'Length',16,'BaseAngle',90,'TipAngle',16,'Width',2);
    drawnow
    X = [];
    Y = [];
    px = x(1);
    py = y(1);
    X = [X,px];
    Y = [Y,py];
    pth = theta(1);
    h3 = plot(X,Y,'b'); % 规划出来的轨迹，蓝色曲线  
    [vehx,vehy] = getVehTran(px,py,pth,veh); % 根据后轴中心的位姿计算车辆边框的位姿
    h1 = plot(vehx,vehy,'k'); % 车辆边框
    h2 = plot(px,px,'rx','MarkerSize',10); % 车辆后轴中心
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2:length(theta)
        px = x(i);
        py = y(i);
        X = [X,px];
        Y = [Y,py];
        pth = theta(i);
        [vehx,vehy] = getVehTran(px,py,pth,veh);
        h1.XData = vehx; %更新h1图像句柄,把车辆边框四个角点的x坐标添加进去
        h1.YData = vehy;
        h2.XData = px; %更新h2图像句柄,把车辆边框四个角点的y坐标添加进去
        h2.YData = py;
        h3.XData = X;
        h3.YData = Y;
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