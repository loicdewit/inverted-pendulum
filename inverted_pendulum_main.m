clear all;
close all;
clc;

%% Specifications

Rm = 2.6;            % (Ohms) Motor armature resistance
Km = 0.00767;        % (Nm/A) Motor torque constant
Kb = 0.00767;        % (V/(rad/s)) Motor back EMF constant
Kg = 3.7;            % 3.7:1 (ratio) Motor gear ratio
M = 0.455;           % (kg) Cart mass with motor and parts
l = 0.305;           % (m) rod length
m = 0.210;           % (kg) rod mass
r = 0.635 * 1e-2;    % (m) radius of motor output gear 
g = 9.81;            % (m/s^2) accelaration due to gravity

%% Linear State Space Model

A = [ 0         0               1                   0;
      0         0               0                   1; 
      0     -m*g/M      -(Kg^2*Km*Kb)/(M*Rm*r^2)    0; 
      0     (M+m)*g/(M*l) (Kg^2*Km*Kb)/(M*Rm*r^2*l)   0;
     ];

B = [       0; 
            0;
     (Km*Kg)/(M*Rm*r);
     -(Km*Kg)/(r*Rm*M*l);
     ];

C = [1 0 0 0;
     0 1 0 0];

D = zeros(2, 1);

eigs(A)

sys = ss(A, B, C, D);

%% Open Loop Analysis

% Poles
pole(sys)

% Transmission zeros 
tzero(sys)

% Stability, controllability, observability, stabilizability, detectability
% minimality ?

%% LQR Controller 

% We are assuming the full state vector is available, so
% for this part the C matrix is set to the identity matrix
Cc = eye(4);

Dc = zeros(4, 1);

sysc = ss(A, B, Cc, Dc);

%% TUNING Q ---- 

Qs{1} = [0.25 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 0];
Qs{2} = [0.25 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 1];
Qs{3} = [0.25 0 0 0;
     0 4 0 0;
     0 0 1 0;
     0 0 0 0];
Qs{4} = [0.25 0 0 0;
     0 4 0 0;
     0 0 1 0;
     0 0 0 1];
Qs{5} = [0.25 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 2];
Qs{6} = [0.25 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 0.5];
Qs{7} = [0.25 0 0 0;
     0 3 0 0;
     0 0 0 0;
     0 0 0 0.5];
Qs{8} = [0.25 0 0 0;
     0 5 0 0;
     0 0 0 0;
     0 0 0 0.5];
Qs{9} = [0.15 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 0.5];
Qs{10} = [0.35 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 0.5];
Qs{11} = [0.2 0 0 0;
     0 5 0 0;
     0 0 0 0;
     0 0 0 0.5];

% Qt = [0.2 0 0 0;
%      0 5 0 0;
%      0 0 0 0;
%      0 0 0 0.5];

Qt = [0.15 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 0.5];

Rs = [0.003 0.003 0.003 0.003 0.003 0.003 0.003 ... 
    0.003 0.003 0.003 0.003 0.003 0.003];

% Cell arrays to store the simulation results

x_ref = {};
x = {};
alpha = {};
u = {};
time = {};
leg = {};

for i=1:length(Qs)
    Q = Qs{i};
    R = Rs(i);
    % Computing the state feedback gain
    K = lqr(sysc, Q, R);
    % Simulating the closed-loop in Simulink
    out = sim('inverted_pendulum.slx');
    p = get(out, 'ScopeData2');
    time{i} = p.time;
    x_ref{i} = p.signals(1).values(:, 1);
    x{i} = p.signals(1).values(:, 2);
    alpha{i} = p.signals(2).values();
    u{i} = p.signals(3).values();
    leg{i}=sprintf('Q_{%i}', i);
end

% Plotting the results
figure;
lw = 1.2; %linewidth

for i=1:length(Qs)
    % Plotting the step response of the horizontal position
    subplot(1, 3, 1); hold on;
    plot(time{i}, x{i}, '-', 'LineWidth', lw);
    xlabel('t[s]'); ylabel('x(t) [m]'); box on;

    % Plotting the step response of the angle alpha
    subplot(1, 3, 2); hold on;
    plot(time{i}, alpha{i}, '-', 'LineWidth', lw);
    xlabel('t[s]'); ylabel('alpha(t) [rad]'); box on;

    % Plotting the control action
    subplot(1, 3, 3); hold on;
    plot(time{i}, u{i}, '-', 'LineWidth', lw);
    xlabel('t[s]'); ylabel('u(t) [V]'); box on;
end

% Plotting the reference
subplot(1, 3, 1);
stairs(time{i}, x_ref{i}, 'k--', 'LineWidth', lw);
leg_ext = leg;
leg_ext{end + 1} = 'reference';
legend(leg_ext, 'location', 'best');
v = axis;
axis([v(1) v(2) -0.2 1.2])
grid

subplot(1, 3, 2)
legend(leg);
grid
subplot(1, 3, 3);
legend(leg);
grid

%% TUNING R -----

Rs = [0.003 0.006 0.008 0.01];%0.001];

x_ref = {};
x = {};
alpha = {};
u = {};
time = {};
leg = {};

for i=1:length(Rs)
    Q = Qt;
    R = Rs(i);
    % Computing the state feedback gain
    K = lqr(sysc, Q, R);
    % Simulating the closed-loop in Simulink
    out = sim('inverted_pendulum.slx');
    p = get(out, 'ScopeData2');
    time{i} = p.time;
    x_ref{i} = p.signals(1).values(:, 1);
    x{i} = p.signals(1).values(:, 2);
    alpha{i} = p.signals(2).values();
    u{i} = p.signals(3).values();
    leg{i}=sprintf('R=%g', Rs(i));
end

% Plotting the results
figure;
lw = 1.2; %linewidth

for i=1:length(Rs)
    % Plotting the step response of the horizontal position
    subplot(1, 3, 1); hold on;
    plot(time{i}, x{i}, '-', 'LineWidth', lw);
    xlabel('t[s]'); ylabel('x(t) [m]'); box on;

    % Plotting the step response of the angle alpha
    subplot(1, 3, 2); hold on;
    plot(time{i}, alpha{i}, '-', 'LineWidth', lw);
    xlabel('t[s]'); ylabel('alpha(t) [rad]'); box on;

    % Plotting the control action
    subplot(1, 3, 3); hold on;
    plot(time{i}, u{i}, '-', 'LineWidth', lw);
    xlabel('t[s]'); ylabel('u(t) [V]'); box on;
end

% Plotting the reference
subplot(1, 3, 1);
stairs(time{i}, x_ref{i}, 'k--', 'LineWidth', lw);
leg_ext = leg;
leg_ext{end + 1} = 'reference';
legend(leg_ext, 'location', 'best');
v = axis;
axis([v(1) v(2) -0.2 1.2])
grid

subplot(1, 3, 2)
legend(leg);
grid
subplot(1, 3, 3);
legend(leg);
grid

%% Tuned model: 

% Need to repeat the experimenting with different tuned models

Qf = [0.35 0 0 0;
     0 4 0 0;
     0 0 0 0;
     0 0 0 0.5];
% 
% Qf = [0.25 0 0 0;
%      0 4 0 0;
%      0 0 1 0;
%      0 0 0 1];

% Qf = [0.2 0 0 0;
%      0 5 0 0;
%      0 0 0 0;
%      0 0 0 0.5];

% Qf = [0.8 0 0 0;
%      0 8 0 0;
%      0 0 0.5 0;
%      0 0 0 4];

Rf = 0.008;

K = lqr(sys, Qf, Rf)

% K = [-7.9057 -52.3826 -14.3830 -12.2393]

% Checking the properties of the closed-loop control system

H = feedback(sysc, K);

% Poles
pole(H)

% Transmission zeroes
tzero(H)


%% Discretizing the model
Ts = 0.005;
sysd = c2d(sysc, Ts, 'tustin');
Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

Kd = lqr(sysd, Qf, Rf);


%% Designing the filter 
wc = pi;
numerator_filter = wc*Ts;
filter_pole = 1/(1+wc*Ts);






