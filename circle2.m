% simulation example: circle with low tire stiffness

vx_start = 1;           % initial longitudinal velocity in body reference frame [m/s]    
m = 25;                 % mass of the vehicle [kg]
a = 0.5;                % distance from front axle to COG [m]
b = 0.5;                % distance from rear axle to COG [m]
Cy = 4  ;               % lateral tire stiffness (Achsenschräglaufsteifigkeit) [N/rad]
t_min = 0;              % start time [s]
t_max = 80;             % stop time [s]
sample_count = 500;     % number of samples

delta = 5*pi/180*ones(1,sample_count);         % steering angle [rad]
Fxf = 5*sin(linspace(0,1/2*pi,sample_count));    % longitudinal force on front tires [N]
Fxr = Fxf;                                     % longitudinal force on rear tires [N]

% start simulation
model_simulation(delta, Fxf, Fxr, vx_start, m, a, b, Cy, t_min, t_max, sample_count);
