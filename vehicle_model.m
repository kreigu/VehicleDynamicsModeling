%VEHICLE_MODEL IDNLGREY model file
function [dx,y] = vehicle_model(t, x, u, m, a, b, Cy, varargin)
    % function parameters:
    % t: the current time
    % x: the state vector at time t 
    % u: the input vector at time t 
    % m: mass of the vehicle [kg]
    % a: distance from front axle to COG [m]
    % b: distance from rear axle to COG [m]
    % Cy: lateral tire stiffness (Achsenschräglaufsteifigkeit) [N/rad] 
    % varargin: optional inputs to the model file

    J = m*0.25*(a+b)^2;                             % moment of inertia (Trägheitsmoment) [kg*m^2]
    Fyf =  Cy*(u(1)-atan((x(4)+a*x(2))./x(3)));     % lateral tire force on front tires [N]
    Fyr = -Cy*atan((x(4)-b*x(2))./x(3));            % lateral tire force on rear tires [N]
    vx_inertial = x(3).*cos(x(1))-x(4).*sin(x(1));  % longitudinal velocity in inertial reference frame [m/s]
    vy_inertial = x(3).*sin(x(1))+x(4).*cos(x(1));  % lateral velocity in inertial reference frame [m/s]

    % state equations.
    dx = [x(2);                                               ... % yaw angle velocity [rad/s]
          1/J*(a*(u(2)*sin(u(1))+Fyf*cos(u(1)))-b*Fyr);       ... % yaw angle accel. [rad/s^2]
          1/m*(u(2)*cos(u(1))-Fyf*sin(u(1))+u(3))+x(4).*x(2); ... % longitudinal accel. in body reference frame [m/s^2]
          1/m*(u(2)*sin(u(1))+Fyf*cos(u(1))+Fyr)-x(3).*x(2);  ... % lateral accel. in body reference frame [m/s^2]
          vx_inertial; ... % longitudinal velocity in inertial reference frame [m/s]
          vy_inertial  ... % lateral velocity in inertial reference frame [m/s]
         ];

    % output equations.
    y = [x(1); ... % yaw angle [rad]
         x(2); ... % yaw angle velocity [rad/s]
         x(3); ... % longitudinal velocity in body reference frame [m/s]
         x(4); ... % lateral velocity in body reference frame [m/s]     
         x(5); ... % longitudinal position in inertial reference frame [m]
         x(6); ... % lateral position in inertial reference frame [m]
         vx_inertial; ... % longitudinal velocity in inertial reference frame [m/s]
         vy_inertial; ... % lateral velocity in inertial reference frame [m/s]
         Fyf; ...         % lateral tire force on front tires [N]
         Fyr ...         % lateral tire force on rear tires [N]
        ];
end

