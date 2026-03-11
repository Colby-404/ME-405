% Electromechanical properties
% NEED CONTROL SYS TOOLBOX
% K = 250*2*pi/60/4.5 % Motor Gain [rad/(V*s)]
K = 2.835;
tau = 0.1 ; % Motor Time Constant [s]
r = 35 ; % Wheel Radius [mm]
w = 141 ; % Width between wheels [mm]
speed_multiplier = 10;
T_s = tau/10 ;% Time step in seconds
A = [-1/tau 0 0 0,
 0 -1/tau 0 0,
 r/2 r/2 0 0,
 -r/w r/w 0 0];
B = [K/tau 0,
 0 K/tau,
 0 0,
 0 0];
C = [0 0 1 -w/2,
 0 0 1 w/2,
 0 0 0 1,
 -r/w r/w 0 0];
D = zeros([4,2]);
% Create the state-space model object
sys_ss = ss(A, B, C, D);
% Get the poles of the system model
poles = pole(sys_ss);
sped_up_poles = poles.*speed_multiplier;
complex_poles = [-1000 + 200j, -1000 - 200j];
desired_poles = [sped_up_poles(3:4)', complex_poles ];
L = place(A', C', sped_up_poles)';
A_zero = A - L*C;
B_zero = [B- L*D, L];
new_sys = ss(A_zero, B_zero, C, 0);
disc = c2d(new_sys, T_s, 'zoh');
A_d = disc.A
B_d = disc.B