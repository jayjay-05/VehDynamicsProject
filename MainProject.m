clear all, close all, clc;
% Given Parameters
m = 1400;                       %mass (kg)
a = 1.14;                       %Front Axle to CM distance (m)
b = 1.33;                       %Rear Axle to CM distance (m)
C_alpha_front = 25000;          %Front tire cornering stiffness (N/rad)
C_alpha_rear = 21000;           %Rear Tire cornering stiffness (N/rad)
I_z = 2420;                     %Yaw Inertia (kg*m^2)
u =275*1000/3600;               %Velocity in x direction (km/h)
delta = 0.1;                    %steering angle input


%Initial Conditions
x0= [0;0];

% Compute trajectory 
dt =0.1;                      % anything smaller than 0.001 takes 30+ seconds to solve
tspan=[0:dt:30];                 %choose domain range as needed

X(:,1)=x0;
xin = x0;
for i=1:length(tspan)-1
    time = i*dt;
    %xnext = rk4_algo(@(x)vehicle_model(C_alpha_front,C_alpha_rear,a,b,m,u,I_z,delta,xin),dt, time,xin);

    xnext = rk4_algo(@(t, x)vehicle_model(x, C_alpha_front,C_alpha_rear,a,b,m,u,I_z,delta) , dt, time, xin);

    X = [X xnext];
    xin = xnext;
end

% find psi and y
fun= @(t,x) x;    % v' = x
v0 =[0;0];
vin = v0;
V(:,1)=v0;

for n=2:length(tspan)
   vi = V(:,n-1);
   v_dot_i = dt* X(:,n-1);
   V(:,n)= vi +  v_dot_i;

end

%{
figure(1); 
plot(X(1,:),X(2,:),'b'); hold on

figure(2);
plot(V(1,:),V(2,:),'b'); hold on
%}

%Absolute coordinate System

for i =1:length(tspan)
Xdot_coordinate(i) =u*cos(V(2,i)) - (X(1,i)+ a*X(2,i))*sin(V(2,i));
Ydot_coordinate(i) = u*sin(V(2,i)) + (X(1,i)+ a*X(2,i))*cos(V(2,i));

end


X_coordinate(1)=0;
Y_coordinate(1)=0;

for n =2:length(tspan)
X_coordinate(n)= X_coordinate(n-1) + dt*Xdot_coordinate(n-1);
Y_coordinate(n)= Y_coordinate(n-1) + dt*Ydot_coordinate(n-1);

end




figure(3)
%plot(X_coordinate, Y_coordinate, 'b'); hold on

% lateral acceleration

for(i=1:length(tspan))
temp = vehicle_model(X(1,i),C_alpha_front,C_alpha_rear,a,b,m,u,I_z,delta);
lateral_accel(i)= temp(1,1);
end

%figure(4)
%plot(X(2,:),lateral_accel,'b')


data (1,:)= tspan;
data (2,:)= lateral_accel;
data (3,:)= X(1,:);
data (4,:)= V(1,:);
data (5,:)= X(2,:);
data (6,:)= V(2,:);
data (7,:)= Xdot_coordinate;
data (8,:)=Ydot_coordinate;
data (9,:)= X_coordinate;
data (10,:)= Y_coordinate;










%{
n=1;
% Coorrect graph
for tcorrect=0:dt:4
    %Xcorect = [-13.0964*exp(-1.9745*tcorrect)+24.4684*exp(-0.9839*tcorrect)-11.3720; -0.249*exp(-1.9745*tcorrect) -0.6962*exp(-0.9839*tcorrect)+0.9457];
    %plot(Xcorect(1), Xcorect(2))
    xcorrect(1,n) = -13.0964*exp(-1.9745*tcorrect)+24.4684*exp(-0.9839*tcorrect)-11.3720;
    xcorrect(2,n) = -0.249*exp(-1.9745*tcorrect) -0.6962*exp(-0.9839*tcorrect)+0.9457;

    vcorrect(1,n) = 7.29807*exp(-1.9745*tcorrect)-24.868787*exp(-0.9839*tcorrect)-11.3720*tcorrect -17.570717;
    vcorrect(2,n) = 0.126107*exp(-1.9745*tcorrect) +0.70759*exp(-0.9839*tcorrect)+0.9457*tcorrect+0.833697;
    n= n+1;

    figure (1) 
    %plot(xcorrect(1,:), xcorrect(2,:),"Color",'r');

    figure (2)
    plot(vcorrect(1,:), vcorrect(2,:),"Color",'r');

  
end


%finding discrepancy
d = V- vcorrect;
%}


