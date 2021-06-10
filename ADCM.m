function [MACH, QBAR] = ADCM(VT, H)
rho = 2.377E-3;
TFAC = 1 - 0.703E-5*H;
T = 519*TFAC;
MACH = VT/sqrt(1.4*.4*1716.3*T);
QBAR = .5*rho*VT^2;
if H >= 35000
    T = 390;
    rho_new = rho*(TFAC^4.14);
    MACH = VT/sqrt(1.4*.4*1716.3*T);
    QBAR = .5*rho_new*VT^2; 
end
end
    