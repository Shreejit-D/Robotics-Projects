function [ u ] = pd_controller(~, s, s_des, params)
%PD_CONTROLLER  PD controller for the height
%
%   s: 2x1 vector containing the current state [z; v_z]
%   s_des: 2x1 vector containing desired state [z; v_z]
%   params: robot parameters

% Getting state values and parameters
[z, v_z, z_des, vz_des] = deal(s(1),s(2),s_des(1),s_des(2));
[g, m] = deal(params.mass, params.gravity);

% Gains adjusted for rise time 1s; max overshoot <= 5%
kp_z = 300;
kd_z = 50;

% Control equation
u = m*(kp_z*(z_des - z) + kd_z*(vz_des - v_z) + g);

end

