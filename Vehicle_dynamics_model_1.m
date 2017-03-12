function Vehicle_dynamics_model_1
    global m delta a b Fxf Fxr Fyf Fyr J theta_d2 
    m=1; delta = pi/180*1; a=1; b=1; Fxf=1; Fxr=1; 
    J = 0.25*(a+b)^2*m;
    Fyf = 0.5; 
    Fyr = 1; 
    theta_d2=1/J*(a*(Fxf*sin(delta)+Fyf*cos(delta))-b*Fyr);

    t = linspace(0, 10,1000);             % time
    X0 = [0, 0, 0, 0, 0, 0, 0, 0];        % [theta, theta_d1, vx, vy, x, y, vx_rel, vy_rel]
    [t,XY] = ode45(@vehicleDGL,t,X0);     % solving ODE
    DGL3 = 1/m*(Fxf*cos(delta)-Fyf*sin(delta)+Fxr)+XY(:,4).*XY(:,2);
    DGL4 = 1/m*(Fxf*sin(delta)+Fyf*cos(delta)+Fyr)-XY(:,3).*XY(:,2);
    DGL5 =  XY(:,3).*cos(XY(:,1))+XY(:,4).*sin(XY(:,1));
    DGL6 = -XY(:,3).*sin(XY(:,1))+XY(:,4).*cos(XY(:,1));
    xy1=sqrt(XY(:,5).^2+XY(:,6).^2);
    xy2=sqrt(XY(:,7).^2+XY(:,8).^2);
    v1=sqrt(XY(:,3).^2+XY(:,4).^2);
    v2=sqrt(DGL5.^2+DGL6.^2);
    a1=sqrt(DGL3.^2+DGL4.^2);

    figure(1); clf; hold on;    
%     plot(t,xy1,'b','linewidth',1);
%     plot(t,xy2,'r','linewidth',1);
%     plot(t,v1,'g','linewidth',1);
%     plot(t,v2+1,'r','linewidth',1);
%     plot(t,a1,'r','linewidth',1);

%     plot(t,XY(:,1),'r','linewidth',1);
%     plot(t,XY(:,2),'b','linewidth',1);
      
%     plot(t,XY(:,3),'r','linewidth',1);
%     plot(t,DGL3,'b','linewidth',1);

%     plot(t,XY(:,4),'r','linewidth',1);
%     plot(t,DGL4,'b','linewidth',1);

%     plot(t,XY(:,5),'g','linewidth',1);
%     plot(t,DGL5,'b','linewidth',1);

%     plot(t,XY(:,6),'r','linewidth',1);
%     plot(t,DGL6,'b','linewidth',1);

%     plot(XY(:,3),XY(:,4),'r','linewidth',1);
    plot(XY(:,5),XY(:,6),'b','linewidth',1);
%     plot(XY(:,7),XY(:,8),'k','linewidth',1);
%     plot(DGL3,DGL4,'b','linewidth',1);
%     plot(DGL5,DGL6,'b','linewidth',1);
      grid on; hold off;

end

function dgl=vehicleDGL(t,x)
    global m delta a b Fxf Fxr Fyf Fyr J theta_d2
    dgl = zeros(8,1);
    dgl(1) = x(2);
    dgl(2) = theta_d2;
    dgl(3) = 1/m*(Fxf*cos(delta)-Fyf*sin(delta)+Fxr)+x(4).*x(2);
    dgl(4) = 1/m*(Fxf*sin(delta)+Fyf*cos(delta)+Fyr)-x(3).*x(2);
    dgl(5) =  x(3).*cos(x(1))+x(4).*sin(x(1)); 
    dgl(6) = -x(3).*sin(x(1))+x(4).*cos(x(1)); 
    dgl(7) = x(3);
    dgl(8) = x(4);
end


