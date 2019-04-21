clear
close all
clc

%% One Segment

% Variables
syms q dq ddq real

% Parameters
m = 0.05; % This is made up for now
g = [9.81; 0; 0]; % The x axis points down
L = 0.084; % 8.4 cm

% Initial conditions
T0_end = [eye(3), [L; 0; 0]; [0, 0, 0, 1]];
T0_com = [eye(3), [L/2; 0; 0]; [0, 0, 0, 1]];

q_adj = q + 0.0001; % move the singularity to -0.0001 instead of zero, but leave everything else the same

% Transformation matrix
[T1, T2, T3, T4] = finger_transforms(q_adj, L);

%% Calculations
% Rather than using the rigid dynamics as a crutch, we can directly find
% D(q) ddq + C(q, dq)dq + G(q) = B*tau
% We're ignoring external force

% Positions
G_com = T1*T2*T0_com;
p_com = G_com(1:3, 4);

G_end = T1*T2*T3*T4*T0_end;
p_end = G_end(1:3, 4);

% Velocities
dp_com = simplify(jacobian(p_com, q)*dq);
dp_end = simplify(jacobian(p_end, q)*dq);

% Energy
KE = 0.5*m*(dp_com'*dp_com);
PE = m*g'*p_com;

%% Dynamics

[M, C, G, B] = LagrangianDynamics(KE, PE, q, dq, q);

%% Augmented Dynamics

syms K D Tau real

% B*Tau = M*ddq + (C + D)*dq + G + K*q;
% Represent the dynamics as a control-affine system

fs = [dq; -M\(C + D)*dq - M\(G + K*q)];
gs = [zeros(size(dq)); M\B];
dynamics = fs + gs*Tau;


%% Alternate Stiffness model

syms C1 C2 real

lambda = q/sin(q);
sigma = ((lambda^4 - 1)/ lambda^3)*(2*C1 + 4*C2*(lambda - 1/lambda)^2);
K2 = sigma/lambda;

fs2 = [dq; -M\(C + D)*dq - M\(G + K2*q)];
dynamics2 = fs2 + gs*Tau;


%% Generate functions

if ~exist('./gen')
    mkdir('./gen')
end
addpath('./gen')

matlabFunction(M, 'File', 'gen/M_gen', 'Vars', {q});
matlabFunction(C, 'File', 'gen/C_gen', 'Vars', {q, dq});
matlabFunction(G, 'File', 'gen/G_gen', 'Vars', {q});
matlabFunction(B, 'File', 'gen/B_gen', 'Vars', {q});
matlabFunction(dynamics, 'File', 'gen/dynamics_gen', 'Vars', {q, dq, K, D, Tau});
matlabFunction(dynamics2, 'File', 'gen/dynamics2_gen', 'Vars', {q, dq, C1, C2, D, Tau});
matlabFunction(p_end, 'File', 'gen/p_end_gen', 'Vars', {q});

