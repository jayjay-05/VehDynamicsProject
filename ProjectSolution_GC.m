
clear all, close all, clc;

% Given Parameters
m = 1400;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
C_alpha_front = 25000;          %Front tire cornering stiffness (N/rad)
C_alpha_rear = 21000;           %Rear Tire cornering stiffness (N/rad)
I_z = 2420;                     %Yaw Inertia (kg*m^2)
u =75;                          %Velocity in x direction (km/h)
delta = 0.1;                    %steering angle input
time_step = 0.01;                %time step value


%info = vehicle_system_solver (m,a,b,I_z,54.37405,delta,C_alpha_rear,C_alpha_front, time_step);



%Analytical Solution Given to Use
n=1;
for tcorrect=0:0.01:60
   
  
    vcorrect(1,n) = 6.629745*exp(-1.9745*tcorrect)-24.868787*exp(-0.9839*tcorrect)-11.3720*tcorrect +18.24042;
    vcorrect(2,n) = 0.12458*exp(-1.9745*tcorrect) +0.703323*exp(-0.9839*tcorrect)+0.9457*tcorrect-0.82791;

    xcorrect(1,n) = -13.0964*exp(-1.9745*tcorrect)+24.4684*exp(-0.9839*tcorrect)-11.3720;
    xcorrect(2,n) = -0.249*exp(-1.9745*tcorrect) -0.6962*exp(-0.9839*tcorrect)+0.9457;
    n= n+1;

end

info = vehicle_system_solver (m,a,b,I_z,75,delta,C_alpha_rear,C_alpha_front, 0.01);
v_value(1,:) = info(4,:);
v_value(2,:) = info(6,:);

x_value(1,:) = info(3,:);
x_value(2,:) =info(5,:);

v_diff = v_value - vcorrect;
x_diff = x_value - xcorrect;

ydot_res = norm(x_diff(1,:));
psidot_res = norm(x_diff(2,:));
y_res = norm(v_diff(1,:));
psi_res = norm(v_diff(2,:));

%Ground Truth Using Smaller Grid

info_fine_mesh =  vehicle_system_solver (m,a,b,I_z,54.37405,delta,C_alpha_rear,C_alpha_front, 0.0001);

grid_spacing =[0.5, 0.1, 0.05, 0.01, 0.005, 0.001,0.0005, 0.0001];
log_grid_spacing = log10(grid_spacing);
RK4_diff = zeros(length(grid_spacing),length(info_fine_mesh(1,:)));
FEM_diff = zeros(length(grid_spacing),length(info_fine_mesh(1,:)));


for(n=1:length(grid_spacing))
    time_step = grid_spacing(n);
    info_temp = vehicle_system_solver (m,a,b,I_z,54.37405,delta,C_alpha_rear,C_alpha_front, time_step);

   %find difference
   for(i=1:length(info_temp(1,:)))

       time = info_temp(1,i);
       k= find(info_fine_mesh(1,:)==time );

    RK4_diff(n,k)= info_temp(5,i) - info_fine_mesh(5,k);
    FEM_diff(n,k)= info_temp(6,i) - info_fine_mesh(6,k);

   end
    RK4_norm(n) = norm(RK4_diff(n,:));
    FEM_norm(n) = norm(FEM_diff(n,:));

end

RK4_log_residual = log10(RK4_norm);
FEM_log_residual = log10(FEM_norm);

figure(2)
plot(log_grid_spacing(:), RK4_log_residual(:),'g', LineWidth=1.5)
hold on
plot(log_grid_spacing(:), FEM_log_residual(:),'r', LineWidth=1.5)
hold on
 title ('Grid Covergence')
    xlabel('log(grid spacing)')
    ylabel('log(residual)')
    legend({'RK4 Algorithm','FEM Algorithm'})




