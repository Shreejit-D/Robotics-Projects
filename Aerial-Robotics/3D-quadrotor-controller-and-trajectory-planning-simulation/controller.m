function [F, M] = controller(t, state, des_state, params)
%CONTROLLER  Controller for the quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [x; y; z], state.vel = [x_dot; y_dot; z_dot],
%   state.rot = [phi; theta; psi], state.omega = [p; q; r]
%
%   des_state: The desired states are:
%   des_state.pos = [x; y; z], des_state.vel = [x_dot; y_dot; z_dot],
%   des_state.acc = [x_ddot; y_ddot; z_ddot], des_state.yaw,
%   des_state.yawdot
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls


% Defining Variables
[x, y, z, x_dot, y_dot, z_dot, phi, theta, psi, p, q, r] = deal(state.pos(1),...
    state.pos(2),state.pos(3),state.vel(1),state.vel(2),state.vel(3), ...
    state.rot(1), state.rot(2), state.rot(3), state.omega(1), state.omega(2), state.omega(3));

[x_des, y_des, z_des, x_dot_des, y_dot_des, z_dot_des, x_ddot_des, y_ddot_des,...
    z_ddot_des, psi_des, psi_dot_des] = deal(des_state.pos(1),...
    des_state.pos(2),des_state.pos(3),des_state.vel(1),des_state.vel(2),des_state.vel(3), ...
    des_state.acc(1), des_state.acc(2), des_state.acc(3), des_state.yaw, des_state.yawdot);
% System parameters
[m, g, I, F_max, F_min] = deal(params.mass, params.gravity, params.I, params.maxF, params.minF);

% Gains
[kd3, kp3] = deal(100,100);
[kdx, kpx] = deal(100,100);
[kdy, kpy] = deal(100,100);
[kpphi, kdphi] = deal(100,100);
[kqth, kdth] = deal(100,100);
[krpsi, kdpsi] = deal(100,100);

% Controller
F = m*(g + z_ddot_des + kd3*(z_dot_des - z_dot) + kp3*(z_des - z));

% Check for limits
if F > F_max
    F = F_max;
elseif F < F_min
    F = F_min;
end

% Inner Control Loop for M
com_x_ddot = x_ddot_des + kdx*(x_dot_des - x_dot) + kpx*(x_des - x);
com_y_ddot = y_ddot_des + kdy*(y_dot_des - y_dot) + kpy*(y_des - y);

phi_des = 1/g*(com_x_ddot*sin(psi_des) - com_y_ddot*cos(psi_des));
th_des = 1/g*(com_x_ddot*cos(psi_des) + com_y_ddot*sin(psi_des));

M = [kpphi*(phi_des - phi) + kdphi*(-p); kqth*(th_des - theta) + kdth*(-q); krpsi*(psi_des - psi) + kdpsi*(psi_dot_des - r)];

end
