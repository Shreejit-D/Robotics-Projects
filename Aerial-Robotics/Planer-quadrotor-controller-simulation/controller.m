function [ u1, u2 ] = controller(~, state, des_state, params)
%CONTROLLER  Controller for the planar quadrotor
%
%   state: The current state of the robot with the following fields:
%   state.pos = [y; z], state.vel = [y_dot; z_dot], state.rot = [phi],
%   state.omega = [phi_dot]
%
%   des_state: The desired states are:
%   des_state.pos = [y; z], des_state.vel = [y_dot; z_dot], des_state.acc =
%   [y_ddot; z_ddot]
%
%   params: robot parameters

%   Using these current and desired states, you have to compute the desired
%   controls

% Getting state variables
[y, z, y_dot, z_dot, phi, phi_dot] = deal(state.pos(1), state.pos(2), ...
    state.vel(1), state.vel(2), state.rot, state.omega);
[y_des, z_des, y_dot_des, z_dot_des, y_ddot_des, z_ddot_des] = ...
    deal(des_state.pos(1), des_state.pos(2), des_state.vel(1), ...
    des_state.vel(2), des_state.acc(1), des_state.acc(2));

% Params
m = params.mass;
g = params.gravity;
I = params.Ixx;

% Gains
[kpz, kdz] = deal(200,100);
[kpy, kdy] = deal(10,10);
[kpphi, kdphi] = deal(1000,100);

% Controller
% Attitude Controller
phi_des = -1/g * (y_ddot_des + kdy*(y_dot_des - y_dot) + kpy*(y_des - y));

% Control Parameters
u1 = m*(g + z_ddot_des + kdz*(z_dot_des - z_dot) + kpz*(z_des - z));
u2 = I*(kdphi*(-phi_dot) + kpphi*(phi_des - phi));


end

