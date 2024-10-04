VehicleAnimation2(x,y,th,Configure,Vehicle);
function VehicleAnimation2(x,y,theta,cfg,veh)
    global param
    sz=get(0,'screensize');
    figure('outerposition',sz);
    videoFWriter = VideoWriter('Parking.mp4','MPEG-4');
    open(videoFWriter);
    ObstList = cfg.ObstList;
    scatter(ObstList(:,1),ObstList(:,2),10,'r') % ��ɢ��ͼ
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
    h3 = plot(X,Y,'b'); % �滮�����Ĺ켣����ɫ����  
    [vehx,vehy] = getVehTran(px,py,pth,veh); % ���ݺ������ĵ�λ�˼��㳵���߿��λ��
    [wheelx1,wheely1] = Wheel_point1(px,py,pth,veh,phy);
    [wheelx2,wheely2] = Wheel_point2(px,py,pth,veh,phy);    
    [wheelx3,wheely3] = Wheel_point3(px,py,pth,veh);
    [wheelx4,wheely4] = Wheel_point4(px,py,pth,veh);
    h1 = plot(vehx,vehy,'k'); % �����߿�
    h2 = plot(px,px,'rx','MarkerSize',10); % ������������
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
        h1.XData = vehx; %����h1ͼ����,�ѳ����߿��ĸ��ǵ��x������ӽ�ȥ
        h1.YData = vehy;
        h2.XData = px; %����h2ͼ����,�ѳ����߿��ĸ��ǵ��y������ӽ�ȥ
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
        img = getframe(gcf);%ֻ��ȡͼ����������ݣ�gcf��ͼ�����
        writeVideo(videoFWriter,img);
%         pause(0.005)
    end
    close(videoFWriter);
end

 % ���ݺ������ĵ�λ�˼��㳵���߿��λ��
function [x,y] = getVehTran(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    
    % �����ı߿����ĸ��ǵ�ȷ��
    Cornerfl = [LF, W/2]; % ��ǰ���ǵ�
    Cornerfr = [LF, -W/2]; % ��ǰ���ǵ�
    Cornerrl = [-LB, W/2]; % ��󷽽ǵ�
    Cornerrr = [-LB, -W/2]; % �Һ󷽽ǵ�
    Pos = [x,y]; % ������������
    dcm = angle2dcm(-theta, 0, 0); % �����ĸ��ǵ����ת����,�����Ǹ����һ���֣���ת������ͬ�����Ƕ�ת��Ϊ�������Ҿ�����ת˳����ZYX
    
    tvec = dcm*[Cornerfl';0]; % ��ת�任��Cornerfl��ת���γɵ���������λ������3*1�����һ����z����
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % ƽ�Ʊ任
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % ���س����߿��ĸ��ǵ��x,y����
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function [x,y] = Wheel_point1(x,y,theta,veh,phy)
    W = veh.W;
    WB = veh.WB;
    wheel_width = 0.4;
    wheel_length = 1;
    theta = theta+phy;
    
    % �����ı߿����ĸ��ǵ�ȷ��
    Cornerfl = [WB+wheel_length/2, W/2+wheel_width/2]; % ��ǰ���ǵ�
    Cornerfr = [WB+wheel_length/2, W/2-wheel_width/2]; % ��ǰ���ǵ�
    Cornerrl = [WB-wheel_length/2, W/2+wheel_width/2]; % ��󷽽ǵ�
    Cornerrr = [WB-wheel_length/2, W/2-wheel_width/2]; % �Һ󷽽ǵ�
    Pos = [x,y]; % ������������
    dcm = angle2dcm(-theta, 0, 0); % �����ĸ��ǵ����ת����,�����Ǹ����һ���֣���ת������ͬ�����Ƕ�ת��Ϊ�������Ҿ�����ת˳����ZYX
    
    tvec = dcm*[Cornerfl';0]; % ��ת�任��Cornerfl��ת���γɵ���������λ������3*1�����һ����z����
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % ƽ�Ʊ任
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % ���س����߿��ĸ��ǵ��x,y����
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

function [x,y] = Wheel_point2(x,y,theta,veh,phy)
    W = veh.W;
    WB = veh.WB;
    wheel_width = 0.4;
    wheel_length = 1;
    theta = theta+phy;
    
    % �����ı߿����ĸ��ǵ�ȷ��
    Cornerfl = [WB+wheel_length/2, -W/2+wheel_width/2]; % ��ǰ���ǵ�
    Cornerfr = [WB+wheel_length/2, -W/2-wheel_width/2]; % ��ǰ���ǵ�
    Cornerrl = [WB-wheel_length/2, -W/2+wheel_width/2]; % ��󷽽ǵ�
    Cornerrr = [WB-wheel_length/2, -W/2-wheel_width/2]; % �Һ󷽽ǵ�
    Pos = [x,y]; % ������������
    dcm = angle2dcm(-theta, 0, 0); % �����ĸ��ǵ����ת����,�����Ǹ����һ���֣���ת������ͬ�����Ƕ�ת��Ϊ�������Ҿ�����ת˳����ZYX
    
    tvec = dcm*[Cornerfl';0]; % ��ת�任��Cornerfl��ת���γɵ���������λ������3*1�����һ����z����
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % ƽ�Ʊ任
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % ���س����߿��ĸ��ǵ��x,y����
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

%�����ĸ�����(���)
function [x,y] = Wheel_point3(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    wheel_width = 0.4;
    wheel_length = 1;
    
    % �����ı߿����ĸ��ǵ�ȷ��
    Cornerfl = [wheel_length/2, W/2+wheel_width/2]; % ��ǰ���ǵ�
    Cornerfr = [wheel_length/2, W/2-wheel_width/2]; % ��ǰ���ǵ�
    Cornerrl = [-wheel_length/2, W/2+wheel_width/2]; % ��󷽽ǵ�
    Cornerrr = [-wheel_length/2, W/2-wheel_width/2]; % �Һ󷽽ǵ�
    Pos = [x,y]; % ������������
    dcm = angle2dcm(-theta, 0, 0); % �����ĸ��ǵ����ת����,�����Ǹ����һ���֣���ת������ͬ�����Ƕ�ת��Ϊ�������Ҿ�����ת˳����ZYX
    
    tvec = dcm*[Cornerfl';0]; % ��ת�任��Cornerfl��ת���γɵ���������λ������3*1�����һ����z����
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % ƽ�Ʊ任
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % ���س����߿��ĸ��ǵ��x,y����
    x = [Cornerfl(1),Cornerfr(1),Cornerrr(1),Cornerrl(1),Cornerfl(1)];
    y = [Cornerfl(2),Cornerfr(2),Cornerrr(2),Cornerrl(2),Cornerfl(2)];
end

%�����ĸ�����(���)
function [x,y] = Wheel_point4(x,y,theta,veh)
    W = veh.W;
    LF = veh.LF;
    LB = veh.LB;
    wheel_width = 0.4;
    wheel_length = 1;
    
    % �����ı߿����ĸ��ǵ�ȷ��
    Cornerfl = [wheel_length/2, -W/2+wheel_width/2]; % ��ǰ���ǵ�
    Cornerfr = [wheel_length/2, -W/2-wheel_width/2]; % ��ǰ���ǵ�
    Cornerrl = [-wheel_length/2, -W/2+wheel_width/2]; % ��󷽽ǵ�
    Cornerrr = [-wheel_length/2, -W/2-wheel_width/2]; % �Һ󷽽ǵ�
    Pos = [x,y]; % ������������
    dcm = angle2dcm(-theta, 0, 0); % �����ĸ��ǵ����ת����,�����Ǹ����һ���֣���ת������ͬ�����Ƕ�ת��Ϊ�������Ҿ�����ת˳����ZYX
    
    tvec = dcm*[Cornerfl';0]; % ��ת�任��Cornerfl��ת���γɵ���������λ������3*1�����һ����z����
    tvec = tvec';
    Cornerfl = tvec(1:2)+Pos; % ƽ�Ʊ任
    
    tvec = dcm*[Cornerfr';0];
    tvec = tvec';
    Cornerfr = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrl';0];
    tvec = tvec';
    Cornerrl = tvec(1:2)+Pos;
    
    tvec = dcm*[Cornerrr';0];
    tvec = tvec';
    Cornerrr = tvec(1:2)+Pos;
    
    % ���س����߿��ĸ��ǵ��x,y����
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