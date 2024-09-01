
clear all, close all, clc;

% Given Parameters
m = 1450;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
C_alpha_front = 25000;          %Front tire cornering stiffness (N/rad)
C_alpha_rear = 21000;           %Rear Tire cornering stiffness (N/rad)
I_z = 2420;                     %Yaw Inertia (kg*m^2)
u = 275;                        %Velocity in x direction (km/h)
delta = 0.1;                    %steering angle input
time_step = 0.01;               %time step value

%info = vehicle_system_solver (m,a,b,I_z,u,delta,C_alpha_rear,C_alpha_front,time_step);


%Part C-1

%Oringinal Parameters
m = 1400;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
u = 217.8088;                   %Stable Velocity in x direction from Part B(km/h)

info_oringinal = vehicle_system_solver (m,a,b,I_z,u,delta,C_alpha_rear,C_alpha_front,time_step);


%50kg mass on rear
m = 1450;                       %mass (kg)
a = 1.1858;                     %Front Axle to CM distance (m)
b = 1.284;                      %Rear Axle to CM distance (m)

info_rear = vehicle_system_solver (m,a,b,I_z,u,delta,C_alpha_rear,C_alpha_front,time_step);


%50kg mass on front
m = 1450;                       %mass (kg)
a = 1.10069;                    %Front Axle to CM distance (m)
b = 1.3631;                     %Rear Axle to CM distance (m)

info_front = vehicle_system_solver (m,a,b,I_z,u,delta,C_alpha_rear,C_alpha_front,time_step);


%plots for Part C-1 
figure (1)          %latteral accelleration vs time

plot( info_oringinal (1,:), info_oringinal (2,:), "r" )
hold on

plot( info_rear (1,:), info_rear (2,:), "b" )
hold on

plot( info_front (1,:), info_front (2,:), "g")
title ('Lateral Acceleration vs Time')
xlabel('Time, t')
ylabel('Lateral acceleration')
legend({'original mass','50 kg mass on rear', '50 kg mass on front'})

hold off

figure (2)          %yaw rate vs time

plot( info_oringinal (1,:), info_oringinal (5,:), "r" )
hold on           

plot( info_rear (1,:), info_rear (5,:), "b" )
hold on                 

plot( info_front (1,:), info_front(5,:), "g" )
title ('Yaw Rate vs Time')
xlabel('Time, t')
ylabel('Yaw rate')
legend({'original mass','50 kg mass on rear', '50 kg mass on front'})

hold off

figure(3)
plot (info_oringinal(9,:), info_oringinal(10,:), "r")
hold on

plot( info_rear (9,:), info_rear (10,:), "b" )
hold on                 

plot( info_front (9,:), info_front(10,:), "g" )

title ('Trajectory')
xlabel('X')
ylabel('Y')
legend({'original mass','50 kg mass on rear', '50 kg mass on front'})

hold off

%Part C-2 

m = 1400;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
u = 217.8088;                   %Stable Velocity in x direction from Part B(km/h)

delta = [0, 0.06, 0.1, 0.15, 0.2,0.25];                
for k = 1: length(delta)
    if delta(k) <= 0.06
        C_alpha_front = 20000;
        C_alpha_rear = 20000;
    end
        
    if 0.06 < delta(k) && delta(k) <= 0.2
        C_alpha_front = 100;
        C_alpha_rear = 100;
    end
    
    if delta(k) > 0.2 
        C_alpha_front = 0;
        C_alpha_rear = 0;
    end

    info_oringinal = vehicle_system_solver (m,a,b,I_z,u,delta(k),C_alpha_rear,C_alpha_front,time_step);

    figure (5)
    plot(info_oringinal(1,:) , info_oringinal(2,:))
    hold on
    title ('Lateral Acceleration vs Time')
    xlabel('Time, t')
    ylabel('Lateral acceleration')
    legend({'delta = 0','delta = 0.06', 'delta = 0.1', 'delta = 0.15', 'delta = 0.2', 'delta = 0.25'})
    
    figure (6)
    plot (info_oringinal(9,:), info_oringinal(10,:))
    hold on
    title ('Trajectory')
    xlabel('X')
    ylabel('Y')
    legend({'delta = 0','delta = 0.06', 'delta = 0.1', 'delta = 0.15', 'delta = 0.2', 'delta = 0.25'})

end 


%Part C-3

m = 1400;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
u = 217.8088;                   %Stable Velocity in x direction from Part B(km/h)

delta = [0, 0.06, 0.1, 0.15, 0.2,0.25, 0.3, 0.35];                
for k = 1: length(delta)
    if delta(k) <= 0.06
        C_alpha_front = 20000;
        C_alpha_rear = 20000;
    end
        
    if 0.06 < delta(k) && delta(k) <= 0.3
        C_alpha_front = 5000;
        C_alpha_rear = 5000;
    end
    
    if delta(k) > 0.3 
        C_alpha_front = 0;
        C_alpha_rear = 0;
    end

    info_oringinal = vehicle_system_solver (m,a,b,I_z,u,delta(k),C_alpha_rear,C_alpha_front,time_step);

    figure (7)
    plot(info_oringinal(1,:) , info_oringinal(2,:))
    hold on
    title ('Lateral Acceleration vs Time')
    xlabel('Time, t')
    ylabel('Lateral acceleration')
    legend({'delta = 0','delta = 0.06', 'delta = 0.1', 'delta = 0.15', 'delta = 0.2', 'delta = 0.25', 'delta = 0.3', 'delta = 0.35'})


    figure (8)
    plot (info_oringinal(9,:), info_oringinal(10,:))
    hold on
    title ('Trajectory')
    xlabel('X')
    ylabel('Y')
    legend({'delta = 0','delta = 0.06', 'delta = 0.1', 'delta = 0.15', 'delta = 0.2', 'delta = 0.25', 'delta = 0.3', 'delta = 0.35'})

end 


