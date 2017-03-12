function Vehicle_dynamics_model_2
    global m a b Cy J delta Fxf Fxr     
    m = 1;                    % Mass of the vehicle [kg]
    a = 1;                    % Distance from front axle to COG [m]
    b = 1;                    % Distance from rear axle to COG [m]
    Cy = 50;                  % lateral tire stiffness (Achsenschräglaufsteifigkeit) [N/rad]
    J = m*0.25*(a+b)^2;       % moment of inertia (Trägheitsmoment) [kg*m^2]
    delta = pi/180*1;         % steering angle [rad]
    Fxf = 1;                  % longitudinal force on front tires [N]
    Fxr = 1;                  % longitudinal force on rear tires [N]  

    t = linspace(0, 60,1000);             % time
    X0 = [0, 0, 1, 0, 0, 0];              % [theta, theta_d1, vx, vy, x, y]
    [t,XY] = ode45(@vehicleDGL,t,X0);     % solving ODE
    
    alpha_f = delta-atan(XY(:,4)+a*XY(:,2)./XY(:,3));    % front wheel tire slip angel (Schräglaufwinkel) [rad]
    alpha_r = -atan(XY(:,4)-b*XY(:,2)./XY(:,3));         % rear wheel tire slip angel (Schräglaufwinkel) [rad]     
    Fyf =  Cy*alpha_f;                                   % lateral tire force on front tires [N]
    Fyr = -Cy*alpha_r;                                   % lateral tire force on rear tires [N]
    
    DGL3 = 1/m*(Fxf*cos(delta)-Fyf*sin(delta)+Fxr)+XY(:,4).*XY(:,2);
    DGL4 = 1/m*(Fxf*sin(delta)+Fyf*cos(delta)+Fyr)-XY(:,3).*XY(:,2);
    DGL5 = XY(:,3).*cos(XY(:,1))-XY(:,4).*sin(XY(:,1));
    DGL6 = XY(:,3).*sin(XY(:,1))+XY(:,4).*cos(XY(:,1));
    xy1=sqrt(XY(:,5).^2+XY(:,6).^2);
    v1=sqrt(XY(:,3).^2+XY(:,4).^2);
    v2=sqrt(DGL5.^2+DGL6.^2);
    a1=sqrt(DGL3.^2+DGL4.^2);

    figure(1); clf; hold on;  %axis equal;   
%     plot(t,xy1,'b','linewidth',1);
    plot(t,v1,'g','linewidth',1);
%     plot(t,v2+1,'r','linewidth',1);
%     plot(t,a1,'r','linewidth',1);

%     plot(t,alpha_f,'b','linewidth',1);
%     plot(t,alpha_r,'r','linewidth',1);
%     plot(t,Fyf,'b','linewidth',1);
%     plot(t,Fyr,'r','linewidth',1)
%     plot(t,Fyf+Fyr,'r','linewidth',1)

%     plot(t,XY(:,1),'r','linewidth',1);
%     plot(t,XY(:,2),'b','linewidth',1);
      
%     plot(t,XY(:,3),'r','linewidth',1);
%     plot(t,DGL3,'b','linewidth',1);

%     plot(t,XY(:,4),'g','linewidth',1);
%     plot(t,DGL4,'b','linewidth',1);

%     plot(t,XY(:,5),'g','linewidth',1);
%     plot(t,DGL5,'b','linewidth',1);

%     plot(t,XY(:,6),'r','linewidth',1);
%     plot(t,DGL6,'b','linewidth',1);

%     plot(XY(:,3),XY(:,4),'r','linewidth',1);
    plot(XY(:,5),XY(:,6),'b','linewidth',1);
%     plot(XY(:,7),XY(:,8),'k','linewidth',1);
%     plot(DGL3,DGL4,'b','linewidth',1);
    plot(DGL5,DGL6,'r','linewidth',1);
      grid on; hold off;
end

function dgl=vehicleDGL(t,x)
    global m a b Cy J delta Fxf Fxr 
    Fyf =  Cy*(delta-atan(x(4)+a*x(2)./x(3)));   % lateral tire force on front tires [N]
    Fyr = -Cy*atan(x(4)-b*x(2)./x(3));           % lateral tire force on rear tires [N]
    
    dgl = zeros(6,1);
    dgl(1) = x(2);
    dgl(2) = 1/J*(a*(Fxf*sin(delta)+Fyf*cos(delta))-b*Fyr);
    dgl(3) = 1/m*(Fxf*cos(delta)-Fyf*sin(delta)+Fxr)+x(4).*x(2);
%     dgl(3) = 1/m*(Fxf*cos(delta)-Fyf*sin(delta)+Fxr-0.5*x(3)^2)+x(4).*x(2);
    dgl(4) = 1/m*(Fxf*sin(delta)+Fyf*cos(delta)+Fyr)-x(3).*x(2);
    dgl(5) = x(3).*cos(x(1))-x(4).*sin(x(1)); 
    dgl(6) = x(3).*sin(x(1))+x(4).*cos(x(1)); 
end

% figure('Name','Simulation Validation','NumberTitle','off','units','normalized','outerposition',[0 0 1 1])
% 
%     J = m*0.25*(a+b)^2;
%     (a*(u(2)*sin(u(1))+Fyf*cos(u(1)))-b*Fyr);
%     subplot (3,1,1);
%     plot(t,y.OutputData(:,1),'b','linewidth',1);
%     axis tight;
%     title('Equation 1');
%     xlabel('Time [s]');
%     ylabel('Moment of a torque [Nm]');
%     grid on;

%     subplot (2,4,3);
%     h3 = animatedline('Color','g','LineWidth',1);
%     [x1,x2] = calc_min_max(t);
%     [y1,y2] = calc_min_max(y.OutputData(:,1));
%     axis([x1,x2,y1,y2]);
%     title('Yaw Angle');
%     xlabel('Time [s]');
%     ylabel('Angle [rad]');
%     grid on;
% 
%     subplot (2,4,7);
%     h7 = animatedline('Color','y','LineWidth',1);
%     [x1,x2] = calc_min_max(t);
%     [y1,y2] = calc_min_max(y.OutputData(:,2));
%     axis([x1,x2,y1,y2]);
%     title('Yaw Angle Velocity');
%     xlabel('Time [s]');
%     ylabel('Angle Velocity [rad/s]');
%     grid on;
% 
%     addpoints(h3,t(k),y.OutputData(k,1));
%     addpoints(h7,t(k),y.OutputData(k,2));



%     plot([-COG_distance,COG_distance],[0,0],'k',0,0,'ko','linewidth',1);

%  fun = @(t) t*v(t);
%     position = integral(fun,t_min,t_max)

%         v_abs = sqrt(y.OutputData(k,3).^2+y.OutputData(k,4).^2);
%     draw_arrow = @(x,y,varargin) quiver(x(1),y(1),x(2)-x(1),y(2)-y(1),0, varargin{:} );  

%         fw_pos = [0.5 0.5 1.5 1.5 0.5;0.1 -0.1 -0.1 0.1 0.1]
%         fw_pos=fw_pos-[1;0]
%         fw_new_pos = [cos(Delta(1)),-sin(Delta(1));sin(Delta(1)),cos(Delta(1))]*fw_pos
%         fw = fw_new_pos+[1;0]       
%         fw_pos=fw_pos-[1;0]
%         fw_new_pos = [cos(Delta(1)),-sin(Delta(1));sin(Delta(1)),cos(Delta(1))]*fw_pos
%         fw = fw_new_pos+[1;0]
%     p2 = fw_pos-fw_pos(:,2)+fw_new_pos+fw_pos(:,2)
%     p3 = -fw_pos(:,3)+fw_new_pos+fw_pos(:,3)
%     p4 = -fw_pos(:,4)+fw_new_pos+fw_pos(:,4)
%         plot(fw(1,:),fw(2,:),'k');
%     plot(fw(1,:),fw(2,:),'k');
%         plot([1 1],[1 -1],'k');
%     fw = rectangle('Position',[0.5 -0.1 1 0.2]);
%     fw.FaceColor = [1 1 1];
%     rw = rectangle('Position',[-1.5 -0.1 1 0.2]);
%     rw.FaceColor = [1 1 1];
%         draw_arrow([0 0],[0 1],'linewidth',1,'color','r');
%         x = [0 -1];
%         y = [0 1];
%         quiver(x(1),y(1),x(2)-x(1),y(2)-y(1),0);
%     drawArrow([0 -1],[0 1],'linewidth',1,'color','r');
%         draw_arrow([0 0],[0 1],'linewidth',1,'color','r');

%         addpoints(h5,[t(k),t(k)+1],[v(k),v(k)+1]);

%          v_abs*vk(1)
%         v_abs*vk(2)
%         vk = 1/v_abs*[Y.OutputData(k,3);Y.OutputData(k,4)];
%         v_abs
%         Y.OutputData(k,3)
%         Y.OutputData(k,4)
%         abs(vk)
%         addpoints(h6_2,[0,vk(1)],[0,vk(2)]);
%         fw_pos = [0.5 0.5 1.5 1.5 0.5;0.1 -0.1 -0.1 0.1 0.1]
%     p2 = fw_pos-fw_pos(:,2)+fw_new_pos+fw_pos(:,2)
%     p3 = -fw_pos(:,3)+fw_new_pos+fw_pos(:,3)
%     p4 = -fw_pos(:,4)+fw_new_pos+fw_pos(:,4)
%         plot(fw(1,:),fw(2,:),'k');
%     plot(fw(1,:),fw(2,:),'k');
%         plot([1 1],[1 -1],'k');
%     fw = rectangle('Position',[0.5 -0.1 1 0.2]);
%     fw.FaceColor = [1 1 1];
%     rw = rectangle('Position',[-1.5 -0.1 1 0.2]);
%     rw.FaceColor = [1 1 1];
%         draw_arrow([0 0],[0 1],'linewidth',1,'color','r');
%     drawArrow([0 -1],[0 1],'linewidth',1,'color','r');
%     fw.Position = [-1 -1 0.2 0.5];

% p = pi*[0:5]+pi/2;
% A = [[pi*[0:6]; zeros(1,7)], [p; sin(p)]];
% mapObj = containers.Map(A(1,:),A(2,:));
% ks = cell2mat(keys(mapObj));
% val = cell2mat(values(mapObj));

% t = 0:pi/4:6*pi
% v = sin(t)
% vq1 = interp1(t,v,x);
% plot(t,v,'o',x,vq1,':.');
% xlim([0 2*pi]);
% title('(Default) Linear Interpolation');

% x = 0:pi/4:2*pi;
% v = sin(x);
% xq = 0:pi/16:2*pi;
% figure
% vq1 = interp1(x,v,xq);
% plot(x,v,'o',xq,vq1,':.');
% xlim([0 2*pi]);
% title('(Default) Linear Interpolation');

% subplot (2,4,6);
% h6 = animatedline('Color','m','LineWidth',1);
% axis([min(Y.OutputData(:,3)),max(Y.OutputData(:,3)), ...
%       min(Y.OutputData(:,4)),max(Y.OutputData(:,4))]);
% title('Velocity in Vehicle Reference Frame');
% grid on;
%     addpoints(h6,Y.OutputData(k,3),Y.OutputData(k,4));


% 
% t = linspace(0,30,1000);             
% % udata = iddata([],[pi/180*1*ones(1000,1), ones(1000,2)] ,3/100);
% udata = iddata([],[[pi/180*2*ones(500,1); pi/180*2*ones(500,1)], 6.7*ones(1000,2)] ,3/100); % steering angle [rad]
% Y = sim(nlgr,udata);
% v1=sqrt(Y.OutputData(:,3).^2+Y.OutputData(:,4).^2);
% v2=sqrt(Y.OutputData(:,7).^2+Y.OutputData(:,8).^2);
% % display(Y.OutputData);
% 
% % h = animatedline('Color','b','LineWidth',1);
% % axis([min(t),max(t),min(v1),max(v1)]);
% % % axis([min(t),max(t),min(Y.OutputData(:,2)),max(Y.OutputData(:,2))])
% % for k = 1:length(t)
% %     addpoints(h,t(k),v1(k));
% % %     addpoints(h,t(k),Y.OutputData(k,2));
% %     drawnow
% % end
% 
% subplot (1, 3, 1);
% h1 = animatedline('Color','b','LineWidth',1);
% axis([min(Y.OutputData(:,5)),max(Y.OutputData(:,5)),min(Y.OutputData(:,6)),max(Y.OutputData(:,6))])
% title('Circle drive path');
% grid on;
% 
% subplot (1, 3, 2);
% h2 = animatedline('Color','r','LineWidth',1);
% axis([min(t),max(t),min(v1),max(v1)]);
% title('Circle drive velocity');
% grid on;
% 
% for k = 1:length(t)
%     addpoints(h1,Y.OutputData(k,5),Y.OutputData(k,6));
%     addpoints(h2,t(k),v1(k));
%     drawnow
% end
% 
% subplot (1, 3, 3);
% % figure(1); 
% hold on;  %axis equal;   
% plot(Y.OutputData(:,5),Y.OutputData(:,6),'b','linewidth',1);
% plot(Y.OutputData(:,7),Y.OutputData(:,8),'r','linewidth',1);
% legend('Ort' , 'Geschw.vektor');
% xlabel('Beschriftung der x- Achse');
% ylabel('Beschriftung der y- Achse');
% % plot(t,v1,'g','linewidth',1);
% % plot(t,v2 + 1,'k','linewidth',1);
% grid on; hold off;
% 
% % t = linspace(0,300,10000);             % time
% % % udata = iddata([],[pi/180*1*ones(1000,1), ones(1000,2)] ,3/100);
% % udata = iddata([],[[pi/180*1*ones(5000,1); pi/180*1*ones(5000,1)], 1*ones(10000,2)] ,3/100);
% % Y = sim(nlgr,udata);
% % % display(Y.OutputData);
% % 
% % figure(1); clf; hold on;  %axis equal;   
% % plot(Y.OutputData(:,5),Y.OutputData(:,6),'b','linewidth',1);
% % % plot(t,Y.OutputData(:,3),'r','linewidth',1);
% % grid on; hold off;
% 
