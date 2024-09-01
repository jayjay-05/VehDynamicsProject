
clear all, close all, clc;

% Given Parameters
m = 1950;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
C_alpha_front = 25000;          %Front tire cornering stiffness (N/rad)
C_alpha_rear = 21000;           %Rear Tire cornering stiffness (N/rad)
I_z = 2420;                     %Yaw Inertia (kg*m^2)
%u =75;                          %Velocity in x direction (km/h)
delta = 0.1;                    %steering angle input
time_step = 0.01;                %time step value


info = vehicle_system_solver (m,a,b,I_z,75,delta,C_alpha_rear,C_alpha_front, time_step);


 % Stable Speed Algorithm
speed_a=0;
speed_b= 300;
tol = 0.01;
accel_tol = 1;
max_speed =0;
diff = (speed_b-speed_a)/2;
max_iterations = 100;
while i<=max_iterations
speed_p = speed_a + (speed_b-speed_a)/2; 
temp_p = vehicle_system_solver (m,a,b,I_z,speed_p,delta,C_alpha_rear,C_alpha_front,time_step);
lat_accel_last = temp_p(2,length(temp_p(2,:)));
lat_accel_second_last = temp_p(2,length(temp_p(2,:))-1);
lat_accel_diff = abs(lat_accel_last-lat_accel_second_last);

%{
temp_a = vehicle_system_solver (m,a,b,I_z,speed_a,delta,C_alpha_rear,C_alpha_front);
y_accel_a = temp_p(2,length(temp_a(2,:)));

temp_b = vehicle_system_solver (m,a,b,I_z,speed_b,delta,C_alpha_rear,C_alpha_front);
y_accel_b = temp_p(2,length(temp_b(2,:)));
%}
 diff = (speed_b-speed_a)/2;

if( diff<= tol)
    max_speed = speed_p;
end

i= i+1;
if(lat_accel_diff<= accel_tol)
    speed_a =speed_p;
end

if(lat_accel_diff>= accel_tol)
speed_b =speed_p;

end

end

info_stable_speed = vehicle_system_solver (m,a,b,I_z,max_speed,delta,C_alpha_rear,C_alpha_front,time_step);

figure(4)
plot(info_stable_speed(9,:), info_stable_speed(10,:))
hold on
title ('RAV4 Trajectory at Stable Speed')
xlabel('X (m)')
ylabel('Y (m)')
axis([-100 250 -100 250]);




