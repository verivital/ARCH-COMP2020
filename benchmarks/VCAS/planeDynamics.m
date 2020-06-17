function dx = planeDynamics(t,x,u,T)
%PLANEDYNAMICS 
%   These are the equations used to describe the VCAS airplane dynamics
% 
% Dynamics: (T = 1)
%
% h(t+1) = h - h0_dot * T - 0.5* h0_dot_dot*T^2;
% h0_dot(t+1) = h0_dot + h0_dot_dot*T;
% tao(t+1) = tao - T;
% adv(t+1) = adv;

% adv = u(1); New advisory
% h0_dot_dot = u(2); % Acceleration

dx(1,1) = - x(2)*T - 0.5*u(2)*T^2;
dx(2,1) = u(2)*T;
dx(3,1) = -T;
dx(4,1) = u(1);

end

