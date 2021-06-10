clear; clc;

% straight and level condition (taylor series expansion point)

xdot0 = [0;
    0;
    0;
    0;
    0;
    0;
    0;
    0;
    0];
x0 = [85;
    0;
    1.2713;
    0;
    0;
    0;
    0;
    0.015;
    0];

u0 = [0;
    -0.178;
    0;
    0.0821;
    0.0821];


% define pertubation matrices

dxdotMatrix = 10e-12*ones(9,9);
dxMatrix = 10e-12*ones(9,9);
duMatrix = 10e-12*ones(9,5);

[E, Ap, Bp] = LinearizationRoutine(@RCAM_model_implicit, xdot0, x0, u0, dxdotMatrix,dxMatrix,duMatrix);
A = -inv(E)*Ap;
B = -inv(E)*Bp;

