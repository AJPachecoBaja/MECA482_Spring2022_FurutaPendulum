%% Furuta Invert5ed Pendulum
%
%clear
%clc
%close all

%
%% System Parameters
% Set Open-loop state-space model of rotary double-inverted pendulum

% Initial condition used in state-space model for simulation.
X0 = [-5, 1, -0.5, 0, 0, 0] * pi / 180;
%
%% Filter Parameters
% Cutoff frequency (rad/s)
%      -> Might use this Omega_f_1 from SIMULINK
wcf_1 = 2 * pi * 50.0;
% Pendulum High-pass filter in PD control used to compute velocity
% Cutoff frequency (rad/s)
wcf_2 = 2 * pi * 15.0;
% Damping ratio
zetaf_2 = 0.7;
%

ROTPEN_ABCD_eqns
