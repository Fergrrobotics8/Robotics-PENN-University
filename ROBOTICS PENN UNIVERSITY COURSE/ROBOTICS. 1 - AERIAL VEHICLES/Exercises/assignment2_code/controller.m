function [ u1, u2 ] = controller(t, state, des_state, params)
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

Kvz=10;
Kpz=800;
Kvphi=25;
Kpphi=1600;
Kvy=10;
Kpy=40;

Kvz=15;
Kpz=700;
Kvphi=50;
Kpphi=1000;
Kvy=9;
Kpy=50;

phic=(-1/params.gravity)*(des_state.acc(1)+Kvy*(des_state.vel(1)-state.vel(1))+Kpy*(des_state.pos(1)-state.pos(1)));
u1 = params.mass*(params.gravity+des_state.acc(2)+Kvz*(des_state.vel(2)-state.vel(2))+Kpz*(des_state.pos(2)-state.pos(2)));
u2 = params.Ixx*(Kvphi*(-1*state.omega)+Kpphi*(phic-state.rot));


end

