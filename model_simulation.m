%model_simulation simulate vehicle model and show plots
function model_simulation(delta, Fxf, Fxr, vx_start, m, a, b, Cy, t_min, t_max, sample_count)
    % function parameters:
    % delta: steering angle vector [rad]
    % Fxf: vector for longitudinal forces on front tires [N]
    % Fxr: vector for longitudinal force on rear tires [N]
    % vx_start: initial longitudinal velocity in body reference frame [m/s]
    % m: mass of the vehicle [kg]
    % a: distance from front axle to COG [m]
    % b: distance from rear axle to COG [m]
    % Cy: lateral tire stiffness (Achsenschräglaufsteifigkeit) [N/rad]
    % t_min: start time [s]
    % t_max: stop time [s]
    % sample_count: number of samples 
    
    file_name     = 'vehicle_model'; % file describing the model structure
    order         = [10 3 6];        % model orders [Ny Nu Nx]
    parameters    = [            ... % Initial parameters (rc-car):
                     m;          ... % mass of the vehicle [kg]
                     a;          ... % distance from front axle to COG [m]
                     b;          ... % distance from rear axle to COG [m]
                     Cy];            % lateral tire stiffness (Achsenschräglaufsteifigkeit) [N/rad]
    initial_states = [           ... % initial states:
                     0;          ... % yaw angle [rad]
                     0;          ... % yaw angle velocity [rad/s]
                     vx_start;   ... % longitudinal velocity in body reference frame [m/s]
                     0;          ... % lateral velocity in body reference frame [m/s]
                     0;          ... % longitudinal position in inertial reference frame [m]
                     0];             % lateral position in inertial reference frame [m]
    ts            = 0;               % time-continuous system

    nlgr = idnlgrey(file_name, order, parameters, initial_states, ts,     ...
                    'Name', 'Vehicle dynamics model', 'TimeUnit', 's');

    nlgr.InputName =  {'steering angle';                        ...   % u(1)
                       'longitudinal force on front tires';     ...   % u(2)
                       'longitudinal force on rear tires' };    ...   % u(3)                                           
    nlgr.InputUnit =  {'rad'; 'N'; 'N'};

    nlgr.OutputName = {'yaw angle';                                         ...   % y(1)
                       'yaw angle velocity';                                ...   % y(2)
                       'longitudinal velocity in body reference frame';     ...   % y(3)
                       'lateral velocity in body reference frame';          ...   % y(4)                   
                       'longitudinal position in inertial reference frame'; ...   % y(5)
                       'lateral position in inertial reference frame';      ...   % y(6)
                       'longitudinal velocity in inertial reference frame'; ...   % y(7)
                       'lateral velocity in inertial reference frame';      ...   % y(8)
                       'lateral force on front tires [N]';                  ...   % y(9)
                       'lateral force on rear tires [N]'                    ...   % y(10)                    
                       };
    nlgr.OutputUnit = {'rad'; 'rad/s'; 'm/s'; 'm/s'; 'm'; 'm'; 'm/s'; 'm/s'; 'N'; 'N'};

    nlgr = setinit(nlgr, 'Name', {
                           'yaw angle';                                         ... % x(1)
                           'yaw angle velocity';                                ... % x(2)
                           'longitudinal velocity in body reference frame';     ... % x(3)
                           'lateral velocity in body reference frame';          ... % x(4)
                           'longitudinal position in inertial reference frame'; ... % x(5)
                           'lateral position in inertial reference frame'});    ... % x(6)
    nlgr = setinit(nlgr, 'Unit', {'rad'; 'rad/s'; 'm/s'; 'm/s'; 'm'; 'm'});
    nlgr.InitialStates(3).Minimum = eps(0);   % longitudinal velocity > 0 for the model to be valid.

    nlgr = setpar(nlgr, 'Name', {
                          'vehicle mass';                         ... % m
                          'distance from front axle to COG';      ... % a
                          'distance from rear axle to COG';       ... % b
                          'lateral tire stiffness';               ... % Cy     
                          });        
    nlgr = setpar(nlgr, 'Unit', {'kg'; 'm'; 'm'; 'N/rad'});
    nlgr = setpar(nlgr, 'Minimum', num2cell(eps(0)*ones(4, 1)));   % all parameters > 0!
    nlgr.Parameters(1).Fixed = true;
    nlgr.Parameters(2).Fixed = true;
    nlgr.Parameters(3).Fixed = true;
    nlgr.Parameters(4).Fixed = true;
    present(nlgr);

    sample_time = (t_max-t_min)/(sample_count-1);        % sample time [s]
    COG_distance = (parameters(2)+parameters(3))/2;      % mean distance from COG to axles [m]
    wheel_distance = COG_distance/2;                     % mean distance from COG to wheels [m]
    wheel_radius = COG_distance/2;                       % wheel radius (COG_distance=wheel_distance+wheel_radius) [m]
    tire_width = 2/5*wheel_radius;                       % tire width [m]
    scaled_min = -COG_distance-0.75*COG_distance;        % scaled minimum in vehicle reference frame [m]
    scaled_max = COG_distance+0.75*COG_distance;         % scaled maximum in vehicle reference frame [m]
    t = linspace(t_min,t_max,sample_count);              % time vector
    udata = iddata([],[delta', Fxf', Fxr'],sample_time); % input data for simulation
    y = sim(nlgr,udata);                                 % simulate model with input data 
    v = sqrt(y.OutputData(:,7).^2+y.OutputData(:,8).^2); % vehicle velocity in inertial reference frame [m/s]
    v_max = max(v);                                      % maximum velocity
    [ref_x_min,ref_x_max] = calc_min_max(y.OutputData(:,5));     % min. and max. in inertial reference frame [m]
    scaling_factor = abs(ref_x_max-ref_x_min)/(10*COG_distance); % scaling factor for mean distance from COG to axles
    Fxf_max = max(abs(max(Fxf)),abs(min(Fxf)));                  % max. longitudinal force on front tires [N]
    Fxr_max = max(abs(max(Fxr)),abs(min(Fxr)));                  % max. longitudinal force on rear tires [N]
    Fyf_max = max(abs(max(y.OutputData(:,9))),abs(min(y.OutputData(:,9))));   % max. lateral force on front tires [N]
    Fyr_max = max(abs(max(y.OutputData(:,10))),abs(min(y.OutputData(:,10)))); % max. lateral force on rear tires [N]
  
    % output plots ...
        
    figure('Name','Simulation Plot Window 2','NumberTitle','off','units','normalized','outerposition',[0 0 1 1])

    subplot (2,3,1);
    plot(t,y.OutputData(:,1),'b','linewidth',1);
    axis tight;
    title('Yaw Angle');
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    grid on;
    
    subplot (2,3,2);
    plot(t,y.OutputData(:,9),'c','linewidth',1);
    axis tight;
    title('Lateral Tire Force on Front Tires');
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid on;
    
    subplot (2,3,3);
    plot(t,Fxr,'m','linewidth',1);
    axis tight;
    title('Longitudinal Tire Force on Rear Tires');
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid on;
    
    subplot (2,3,4);
    plot(t,y.OutputData(:,2),'r','linewidth',1);
    axis tight;
    title('Yaw Angle Velocity');
    xlabel('Time [s]');
    ylabel('Angle Velocity [rad/s]');
    grid on;
    
    subplot (2,3,5);
    plot(t,y.OutputData(:,10),'c','linewidth',1);
    axis tight;
    title('Lateral Tire Force on Rear Tires');
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid on;
    
    figure('Name','Simulation Plot Window 1','NumberTitle','off','units','normalized','outerposition',[0 0 1 1])
    
    subplot (2,3,1);
    h1_1 = animatedline('Color','b','LineWidth',1);
    h1_2 = animatedline('Color','k','LineWidth',1);
    h1_3 = animatedline('Color','k','LineWidth',1);
    h1_4 = animatedline('Color','k','LineWidth',1);
    h1_5 = animatedline('Color','k','LineWidth',1,'Marker','o');
    x1 = ref_x_min;
    x2 = ref_x_max;
    [y1,y2] = calc_min_max(y.OutputData(:,6));
    axis([x1,x2,y1,y2])
    ax1 = gca;
    hold on;
    q1 = quiver(ax1,0,0,0,0,0,'Color','r');
    hold off;
    title('Vehicle Path');
    xlabel('Position x [m]');
    ylabel('Position y [m]');
    grid on;

    subplot (2,3,2);
    h2_1 = animatedline('Color','k','LineWidth',1);
    h2_2 = animatedline('Color','k','LineWidth',1);
    h2_3 = animatedline('Color','k','LineWidth',1);
    h2_4 = animatedline('Color','k','LineWidth',1,'Marker','o');
    h2_5 = animatedline('Color','b','LineWidth',1);
    [x1,x2] = calc_min_max(t);
    [y1,y2] = calc_min_max(t);
    axis([x1,x2,y1,y2]);
    ax2 = gca;
    hold on;
    q2 = quiver(ax2,0,0,0,0,0,'Color','r');
    hold off;
    title('Inertial Reference Frame');
    xlabel('Position x [m]');
    ylabel('Position y [m]');
%     grid on;

    subplot (2,3,3);
    h4 = animatedline('Color','g','LineWidth',1);
    [x1,x2] = calc_min_max(t);
    [y1,y2] = calc_min_max(delta);
    axis([x1,x2,y1,y2]);
    title('Steering Angle');
    xlabel('Time [s]');
    ylabel('Angle [rad]');
    grid on;

    subplot (2,3,4);
    h5 = animatedline('Color','r','LineWidth',1);
    [x1,x2] = calc_min_max(t);
    [y1,y2] = calc_min_max(v);
    axis([x1,x2,y1,y2]);
    title('Vehicle Velocity');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    grid on;

    subplot (2,3,5);   
    h6_1 = animatedline('Color','k','LineWidth',1);  
    h6_2 = animatedline('Color','k','LineWidth',1);  
    h6_3 = animatedline('Color','k','LineWidth',1,'Marker','o');
    x1 = scaled_min;
    x2 = scaled_max;
    y1 = scaled_min;
    y2 = scaled_max;
    axis([x1,x2,y1,y2]);
    ax6 = gca;
    hold on;
    q6_1 = quiver(ax6,0,0,0,0,0,'Color','r');
    q6_2 = quiver(ax6,0,0,0,0,0,'Color','m');
    q6_3 = quiver(ax6,0,0,0,0,0,'Color','m');
    q6_4 = quiver(ax6,0,0,0,0,0,'Color','c');
    q6_5 = quiver(ax6,0,0,0,0,0,'Color','c');
    rectangle('Position',[-wheel_distance-2*wheel_radius,-tire_width/2,2*wheel_radius,tire_width]);
    hold off;
    title('Vehicle Reference Frame');
    xlabel('Position x [m]');
    ylabel('Position y [m]');
%     grid on;

    subplot (2,3,6);
    h8 = animatedline('Color','m','LineWidth',1);
    [x1,x2] = calc_min_max(t);
    [y1,y2] = calc_min_max(Fxf);
    axis([x1,x2,y1,y2]);
    title('Longitudinal Tire Force on Front Tires');
    xlabel('Time [s]');
    ylabel('Force [N]');
    grid on;

    % constant vector and matrices for drawing the vehicle model on plots
    longitudinal_axis = [-COG_distance,COG_distance;0,0];
    front_wheel = [-wheel_radius,wheel_radius,wheel_radius,-wheel_radius,-wheel_radius; ...
            -tire_width/2,-tire_width/2,tire_width/2,tire_width/2,-tire_width/2];
    rear_wheel = [-wheel_distance-2*wheel_radius,-wheel_distance,-wheel_distance, ...
            -wheel_distance-2*wheel_radius,-wheel_distance-2*wheel_radius;
            -tire_width/2,-tire_width/2,tire_width/2,tire_width/2,-tire_width/2];
    % number of added plot points in one animation cycle
    animation_step = 5;
        
    for k = animation_step:animation_step:sample_count
        % steering angle (delta) rotation matrix
        delta_rotation_matrix = [cos(delta(k)),-sin(delta(k));sin(delta(k)),cos(delta(k))];
        % yaw angle rotation matrix
        yaw_rotation_matrix = [cos(y.OutputData(k,1)),-sin(y.OutputData(k,1)); ...
            sin(y.OutputData(k,1)),cos(y.OutputData(k,1))];
        % current position vector of vehicle
        position_vector = [y.OutputData(k,5);y.OutputData(k,6)];
        % current velocity vector in inertial reference frame
        velocity_vector = [y.OutputData(k,7);y.OutputData(k,8)];
        % scaled vector for longitudinal force on front tires
        long_front_force = 1.5*wheel_radius/Fxf_max*[Fxf(k);0];
        % scaled vector for longitudinal force on rear tires
        long_rear_force = 1.5*wheel_radius/Fxr_max*[Fxr(k);0];
        % scaled vector for lateral force on front tires
        lat_front_force = 2*wheel_radius/Fyf_max*[0;y.OutputData(k,9)];
        % scaled vector for lateral force on rear tires
        lat_rear_force = 2*wheel_radius/Fyr_max*[0;y.OutputData(k,10)];
                       
        % rotate and shift the plot points for plot 'Vehicle Path'
        longitudinal_axis_rot = scaling_factor*yaw_rotation_matrix*longitudinal_axis+position_vector;
        rear_wheel_rot = scaling_factor*yaw_rotation_matrix*rear_wheel+position_vector;
        front_wheel_rot = scaling_factor*(delta_rotation_matrix*front_wheel+[COG_distance;0]);
        front_wheel_rot = yaw_rotation_matrix*front_wheel_rot+position_vector;
        vk = 1.5*scaling_factor/v_max*velocity_vector;
        
        % draw points for plot 'Vehicle Path'
        clearpoints(h1_2);
        clearpoints(h1_3);
        clearpoints(h1_4);
        clearpoints(h1_5);
        addpoints(h1_1,y.OutputData(k-animation_step+1:k,5)',y.OutputData(k-animation_step+1:k,6)');
        addpoints(h1_2,longitudinal_axis_rot(1,:),longitudinal_axis_rot(2,:));        
        addpoints(h1_3,rear_wheel_rot(1,:),rear_wheel_rot(2,:));
        addpoints(h1_4,front_wheel_rot(1,:),front_wheel_rot(2,:));
        addpoints(h1_5,y.OutputData(k,5),y.OutputData(k,6));
        q1.XData = y.OutputData(k,5);
        q1.YData = y.OutputData(k,6);
        q1.UData = vk(1);
        q1.VData = vk(2);
        
        % scaled axis for plot 'Inertial Reference Frame'
        axis(ax2,[scaled_min+y.OutputData(k,5),scaled_max+y.OutputData(k,5), ...
            scaled_min+y.OutputData(k,6),scaled_max+y.OutputData(k,6)]);
        % rotate and shift the plot points for plot 'Inertial Reference Frame'
        longitudinal_axis_rot = yaw_rotation_matrix*longitudinal_axis+position_vector;
        rear_wheel_rot = yaw_rotation_matrix*rear_wheel+position_vector;
        front_wheel_rot = delta_rotation_matrix*front_wheel+[COG_distance;0];     
        front_wheel_rot = yaw_rotation_matrix*front_wheel_rot+position_vector;
        vk = (COG_distance+wheel_radius)/v_max*velocity_vector;

        % draw points for plot 'Inertial Reference Frame'
        clearpoints(h2_1);
        clearpoints(h2_2);
        clearpoints(h2_3);       
        clearpoints(h2_4);      
        addpoints(h2_1,longitudinal_axis_rot(1,:),longitudinal_axis_rot(2,:));
        addpoints(h2_2,rear_wheel_rot(1,:),rear_wheel_rot(2,:));
        addpoints(h2_3,front_wheel_rot(1,:),front_wheel_rot(2,:));
        addpoints(h2_4,y.OutputData(k,5),y.OutputData(k,6));
        addpoints(h2_5,y.OutputData(k-animation_step+1:k,5)',y.OutputData(k-animation_step+1:k,6)');
        q2.XData = y.OutputData(k,5);
        q2.YData = y.OutputData(k,6);
        q2.UData = vk(1);
        q2.VData = vk(2);

        % draw points for plots 'Steering Angle' and 'Vehicle Velocity'
        addpoints(h4,t(k-animation_step+1:k),delta(k-animation_step+1:k));     
        addpoints(h5,t(k-animation_step+1:k),v(k-animation_step+1:k));

        % rotate and shift the plot points for plot 'Vehicle Reference Frame'
        front_wheel_rot = delta_rotation_matrix*front_wheel+[COG_distance;0];
        vk = (COG_distance+wheel_radius)/v_max*[y.OutputData(k,3);y.OutputData(k,4)];
        rot_long_front_force = delta_rotation_matrix*[0,long_front_force(1);
            tire_width,tire_width+long_front_force(2)]+[COG_distance;0];
        rot_lat_front_force = delta_rotation_matrix*[0,lat_front_force(1);0,lat_front_force(2)] ...
            +[COG_distance;0];
        
        % draw points for plot 'Vehicle Reference Frame'
        clearpoints(h6_1);
        clearpoints(h6_2);
        clearpoints(h6_3);
        addpoints(h6_1,longitudinal_axis(1,:),longitudinal_axis(2,:));        
        addpoints(h6_2,front_wheel_rot(1,:),front_wheel_rot(2,:));  
        addpoints(h6_3,0,0);  
        q6_1.UData = vk(1);
        q6_1.VData = vk(2);
        q6_2.XData = rot_long_front_force(1,1);
        q6_2.YData = rot_long_front_force(2,1);
        q6_2.UData = rot_long_front_force(1,2)-rot_long_front_force(1,1);
        q6_2.VData = rot_long_front_force(2,2)-rot_long_front_force(2,1);
        q6_3.XData = -COG_distance;
        q6_3.YData = tire_width;
        q6_3.UData = long_rear_force(1);
        q6_3.VData = long_rear_force(2);
        q6_4.XData = rot_lat_front_force(1,1);
        q6_4.YData = rot_lat_front_force(2,1);
        q6_4.UData = rot_lat_front_force(1,2)-rot_lat_front_force(1,1);
        q6_4.VData = rot_lat_front_force(2,2)-rot_lat_front_force(2,1);
        q6_5.XData = -COG_distance;
        q6_5.YData = 0;
        q6_5.UData = lat_rear_force(1);
        q6_5.VData = lat_rear_force(2);

        % draw points for plot 'Longitudinal Tire Force'
        addpoints(h8,t(k-animation_step+1:k),Fxf(k-animation_step+1:k));
        drawnow limitrate  
    end
    
    % model validation    
    disp('model validation:');
    x7_1 = y.OutputData(:,7);
    x7_2 = y.OutputData(:,3).*cos(y.OutputData(:,1))-y.OutputData(:,4).*sin(y.OutputData(:,1));
    x8_1 = y.OutputData(:,8);
    x8_2 = y.OutputData(:,3).*sin(y.OutputData(:,1))+y.OutputData(:,4).*cos(y.OutputData(:,1));
    x9_1 = y.OutputData(:,9);
    x9_2 = Cy*(delta'-atan((y.OutputData(:,4)+a*y.OutputData(:,2))./y.OutputData(:,3)));
    x10_1 = y.OutputData(:,10);
    x10_2 = -Cy*atan((y.OutputData(:,4)-b*y.OutputData(:,2))./y.OutputData(:,3));
    x7_error_sum = sum(abs(x7_1-x7_2))
    x8_error_sum = sum(abs(x8_1-x8_2))
    x9_error_sum = sum(abs(x9_1-x9_2))
    x10_error_sum = sum(abs(x10_1-x10_2))    
end

% calculate scaling of x and y axis
function [min_value,max_value] = calc_min_max(array)
    min_value = min(array);
    max_value = max(array);
    
    if min_value==max_value 
        if min_value>0
            min_value = 0;
            max_value = 2*max_value;
        elseif min_value<0
            min_value = 2*min_value;
            max_value = 0;            
        else
            min_value = -1;
            max_value = 1;            
        end
    end    
end
    
