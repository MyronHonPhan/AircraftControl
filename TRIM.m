clear; clc;
close all;

global x u gamma

x(1) = input('Enter Vt: ');
x(5) = input('Enter h: ');
gamma = input('Enter Gamma (deg): ')*(pi/180);
name = input('Name of Cost function file? :','s');
cg = .25;
land = 0; % 0 = clean 1 = gears and flaps
u = [0.1 -10 cg land];
x(2) = .1; %alpha, init guess
x(3) = x(2) + gamma; %theta
x(4) = 0; % pitch rate
x(6) = 0;

s0 = [u(1) u(2) x(2)];

disp(['Initial cost = ',num2str(feval(name,s0))])
[s,fval] = fminsearch(name, s0);
x(2) = s(3);
x(3) = s(3) + gamma;
u(1) = s(1);
u(2) = s(2);
disp(['minimum cost = ', num2str(fval)]);
disp(['minimizing vector = ', num2str(s)]);
temp= [length(x), length(u), x, u];
name = input('Name of output file?: ','s');
dlmwrite(name,temp);