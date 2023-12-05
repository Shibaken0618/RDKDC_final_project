%% Lab 2
clear; clc;

% initialize ur5
ur5 = ur5_interface();


% first iteration transformation matrices (comment out if using first
% iteration)
g0a = [-0.81,   -0.58,    0.01,    0.17;
       -0.07,    0.12,    0.99,    0.45;
       -0.58,    0.80,   -0.14,    0.34;
        0,       0,       0,       1.00];
gab = [-0.01,   0.02,    0.99,   -0.32;
        0.99,  -0.02,    0.01,    0.23;
        0.02,   0.99,   -0.02,    0.26;
        0,      0,       0,       1.00];
gbc = [0.77,    0.08,   -0.63,    0.18;
       0.64,   -0.06,    0.76,    0.07;
       0.03,   -0.99,   -0.09,    0.41;
       0,       0,       0,       1.00];

% second iteration transformation matrices (comment out if using second
% iteration)
% g0a = [1, 0,          0,         0.5;
%        0, cos(pi/4), -sin(pi/4), 0;
%        0, sin(pi/4), cos(pi/4), 0;
%        0, 0,         0,         1];
% gab = [cos(pi/6),  0, sin(pi/6), 0;
%        0,          1, 0,         0.5;
%        -sin(pi/6), 0, cos(pi/6), 0;
%        0,          0, 0,         1];
% gbc = [cos(pi/3), -sin(pi/3), 0, 0;
%        sin(pi/3), cos(pi/3),  0, 0;
%        0,         0,          1, 0.5;
%        0,         0,          0, 1];

I4 = eye(4,4);

% create frames
pause(1);
Frame_A = tf_frame('base_link', 'F_A', g0a);
pause(0.5);
Frame_B = tf_frame('F_A', 'F_B', gab);
pause(0.5);
Frame_C = tf_frame('base_link', 'F_C', I4);
pause(0.5);
Frame_C.move_frame('F_B',gbc)
pause(1);

% compare real and calculated gac
gac = gab * gbc;
err = gac - Frame_C.read_frame('F_A');
disp(err);






