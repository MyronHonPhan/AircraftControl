% File VDPOL.m
function [xd]= vdpol(time,x,u)
xd= [x(2) -u(1)*(x(1) ^2-1)*x(2)-x(1)];