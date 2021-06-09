clear; clc;

%% inputs
name= input('Enter Name of State Equations m-file : ','s');
icfile= input('Enter Name of i.c. File : ','s');
tmp= dlmread(icfile);
n=tmp(1); m=tmp(2);
x=tmp(3:n+2); u=tmp(n+3:n+m+2);
stat=fclose('all');
runtime= input('Enter Run-Time : ');
dt = input('Enter Integration Time-step : ');
N=runtime/dt; k=0; NP= fix ( max(1,N/500) ); time=0.;

save=u(2); % For Example 3.6-3 only
u_array = zeros(1,N);
time_array = zeros(1,N);
%% loop
for i=1:N
time=i*dt;
time_array(1,i+1) = time_array(1,i) + dt;
k=k+1;
y(k,1)= x(1); % VT
y(k,2)= x(2); % alpha
y(k,3)= x(3); % theta
y(k,4)= x(4); % pitch rate, q
y(k,5) = x(5); % altitude
if time>=2 % For Example 3.6-3
    u(2)=save;
elseif time >= 7
    u(2) = save + .1;
elseif time>=4
    u(2)=save-.2;
elseif time>=1.0
    u(2)=save+.1;
else
    u(2)=save;
end
u_array(1,i) = u(2);
[x]= RK4(name,time,dt,x,u);
end

%% plotting
plot(time_array(1:N),y(:,1))
xlabel("Time (s)")
ylabel("Speed")
title("Speed VS Time")

