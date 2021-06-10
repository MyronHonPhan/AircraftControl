function [E,A_P, B_P] = ImplicitLinMod(fun, xdot0, x0, u0, DXDOT, DX, DU)


% obtain number of states and controls
n = length(xdot0);
m = length(u0);

% initialize Jacobian matrix
E = zeros(n,n);
A_P = zeros(n,n);
B_P = zeros(n,m); 

% calculating jacobian matrix (E)
% looping through i (which are the functions) and looping through j (the
% states/control)
for i = 1:n
    for j = 1:n
        
        % obtain the magnitude of pertubation to use
        dxdot = DXDOT(i,j);
        
        % define perubation vector
        xdot_plus = xdot0;
        xdot_minus = xdot0;
        
        xdot_plus(j) = xdot_plus(j) + dxdot;
        xdot_minus(j) = xdot_minus(j) - dxdot; 
        
        % calculate F, it's actually calculating all n rows but let's just
        % keep one at a time.
        F = feval(fun, xdot_plus, x0, u0);
        F_plus_keep = F(i);
        
        F = feval(fun, xdot_minus, x0, u0);
        F_minus_keep = F(i);
        
        % calculate E(row, col)
        E(i,j) = (F_plus_keep - F_minus_keep) / (2*dxdot);
    end
end

% calculating jacobian matrix (A prime)
for i = 1:n
    for j = 1:n
        % obtain the magnitude of pertubation to use
        dx = DX(i,j);
        
        % define perubation vector
        x_plus = x0;
        x_minus = x0;
        
        x_plus(j) = x_plus(j) + dx;
        x_minus(j) = x_minus(j) - dx; 
        
        % calculate F, it's actually calculating all n rows but let's just
        % keep one at a time.
        F = feval(fun, xdot0, x_plus, u0);
        F_plus_keep = F(i);
        
        F = feval(fun, xdot0, x_minus, u0);
        F_minus_keep = F(i);
        
        % calculate A'(row, col)
        A_P(i,j) = (F_plus_keep - F_minus_keep) / (2*dx);
    end
end

% calculating jacobian matrix (A prime)
for i = 1:n
    for j = 1:m
        % obtain the magnitude of pertubation to use
        du = DU(i,j);
        
        % define perubation vector
        u_plus = u0;
        u_minus = u0;
        
        u_plus(j) = u_plus(j) + du;
        u_minus(j) = u_minus(j) - du; 
        
        % calculate F, it's actually calculating all n rows but let's just
        % keep one at a time.
        F = feval(fun,xdot0, x0, u_plus);
        F_plus_keep = F(i);
        
        F = feval(fun, xdot0, x0, u_minus);
        F_minus_keep = F(i);
        
        % calculate B'(row, col)
        B_P(i,j) = (F_plus_keep - F_minus_keep) / (2*du);
    end
end
end