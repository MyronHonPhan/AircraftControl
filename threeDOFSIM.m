function [xd] = threeDOFSIM(time,x,u)

% the purpose of this function is to output the state derivative

%% initializing parameters
S = 2170; CBAR=17.5; MASS=5.0E3; IYY= 4.1E6;
TSTAT=6.0E4; DTDV =-38.0; ZE = 2.0; CDCLS= .042;
CLA = .085; CMA =-.022; CMDE =-.016; % per degree
CMQ =-16.0; CMADOT= -6.0; CLADOT= 0.0; % per radian
RTOD = 57.29578; GD=32.17;
THTL = u(1);        % throttle
ELEV =u(2);
XCG = u(3);
LAND = u(4);
VT = x(1); % TAS in fps
ALPHA= RTOD*x(2); % A.O.A.
THETA= x(3); % PITCH ATTITUDE
Q = x(4); % PITCH RATE
H = x(5); % ALTITUDE

%% calculations
% calculating necessary force and moment coefficients for the three states
% that we are calculating
[MACH,QBAR]= ADCM(VT,H);
QS = QBAR*S;
SALP= sin (x(2)); CALP= cos(x(2));
GAM = THETA - x(2); SGAM= sin (GAM); CGAM= cos(GAM);
if LAND == 0 % CLEAN
CLO= .20; CDO= .016;
CMO= .05; DCDG= 0.0; DCMG= 0.0;
elseif LAND == 1 % LANDING FLAPS & GEAR
CLO= 1.0; CDO= .08;
CMO= -.20; DCDG= .02; DCMG= -.05;
else
disp('Landing Gear & Flaps ?')
end
THR= (TSTAT+VT*DTDV) * max(THTL,0); % THRUST
CL=CLO+CLA*ALPHA; % NONDIM. LIFT
CM=DCMG+CMO+CMA*ALPHA+CMDE*ELEV+CL* (XCG-.25); % MOMENT
CD=DCDG+CDO+CDCLS*CL*CL; % DRAG POLAR

% STATE EQUATIONS NEXT
xd(1) = (THR*CALP-QS*CD)/MASS - GD*SGAM;
xd(2)=(-THR*SALP-QS*CL+MASS*(VT*Q+GD*CGAM))/(MASS*VT+QS*CLADOT);
xd(3) = Q;
D = .5*CBAR*(CMQ*Q+CMADOT*xd(2))/VT; % PITCH DAMPING
xd(4) = (QS*CBAR*(CM + D) + THR*ZE)/IYY; % Q-DOT
xd(5) = VT*SGAM; % VERTICAL SPEED
% xd(6) = VT*CGAM; % HORIZNTL. SPEED