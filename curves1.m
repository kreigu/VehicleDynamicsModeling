% simulation example: periodic steering angle change

vx_start = 1;       % initial longitudinal velocity in body reference frame [m/s]    
m = 25;             % mass of the vehicle [kg]
a = 0.5;            % distance from front axle to COG [m]
b = 0.5;            % distance from rear axle to COG [m]
Cy = 200;           % lateral tire stiffness (Achsenschräglaufsteifigkeit) [N/rad]
t_min = 0;          % start time [s]
t_max = 60;         % stop time [s]
sample_count = 800; % number of samples

delta = 14*pi/180*sin(linspace(0,4*pi,sample_count));   % steering angle [rad]
Fxf = 10.6*sin(linspace(0,1/2*pi,sample_count));           % longitudinal force on front tires [N]
Fxr = Fxf;                                              % longitudinal force on rear tires [N]

% start simulation
model_simulation(delta, Fxf, Fxr, vx_start, m, a, b, Cy, t_min, t_max, sample_count);



