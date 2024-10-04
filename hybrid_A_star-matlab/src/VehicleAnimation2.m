param =[ -17.0344827586207	-3.08812260536398	0.785400000000000	-7.27203065134100	5.67816091954023	0.785398163397448];
VehicleAnimation1(x,y,th,Configure,Vehicle);
function VehicleAnimation1(x,y,theta,cfg,veh)
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
    h3 = plot(X,Y,'b'); % �滮�����Ĺ켣����ɫ����  
    [vehx,vehy] = getVehTran(px,py,pth,veh); % ���ݺ������ĵ�λ�˼��㳵���߿��λ��
    h1 = plot(vehx,vehy,'k'); % �����߿�
    h2 = plot(px,px,'rx','MarkerSize',10); % ������������
    img = getframe(gcf);
    writeVideo(videoFWriter,img);
    for i = 2:length(theta)
        px = x(i);
        py = y(i);
        X = [X,px];
        Y = [Y,py];
        pth = theta(i);
        [vehx,vehy] = getVehTran(px,py,pth,veh);
        h1.XData = vehx; %����h1ͼ����,�ѳ����߿��ĸ��ǵ��x������ӽ�ȥ
        h1.YData = vehy;
        h2.XData = px; %����h2ͼ����,�ѳ����߿��ĸ��ǵ��y������ӽ�ȥ
        h2.YData = py;
        h3.XData = X;
        h3.YData = Y;
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