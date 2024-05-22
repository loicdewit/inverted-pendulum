clear all;
close all;
clc;

%% DC motor
% Parameters
Tm = 1; %Time constant
Km = 10;  %Gain

A = [0   1
    0 -1/Tm];

B =[0
    Km/Tm];

C = [1 0]; % Only the angular position is measured

D = 0;

dc_motor =ss(A,B,C,D)

%% Checking controllability
Co = ctrb(A,B);
disp('Rank of the controllability matrix:')
rank(Co)

%% LQR

%Weigting matrices:
Qs{1} = diag([1 1]);
Qs{2} = diag([20 1]);
Qs{3} = diag([1 5]);
Qs{4} = diag([1 1]);
Qs{5} = diag([1 1]);
Qs{6} = diag([100 1]);
Rs= [1 1 1  10 0.1 0.01];

 % Cell arrays to store the simulation results
 theta_ref={};
 theta ={};
 omega ={};
 u ={};
 time ={};
 leg={};
 
for i=1:length(Qs)
    Q = Qs{i};
    R = Rs(i);
    %Computing the state feedback gain
    K =lqr(A,B,Q,R);
    % Simulating the closed-loop in Sumulink
    out = sim('DC_motor_simulink');
    p =get(out,'simresults');
    time{i} = p.time;
    theta_ref{i} =  p.signals(1).values(:,1);
    theta{i} =  p.signals(1).values(:,2);
    omega{i} =  p.signals(2).values;
    u{i} = p.signals(3).values;
    leg{i}=sprintf('Q = diag([%g %g]), R=%g',Qs{i}(1,1),Qs{i}(2,2),Rs(i)); %legend
end
    
%% Plotting the results

figure;
lw = 1.2; %linewitdth

for i=1:length(Qs)
    subplot(1,3,1); hold on
    plot(time{i},theta{i},'-','LineWidth',lw);
    xlabel('t[s]');ylabel('theta(t)');box on
    
    subplot(1,3,2); hold on
    plot(time{i},omega{i},'-','LineWidth',lw);
    xlabel('t[s]');ylabel('omega(t)');box on
       
    subplot(1,3,3); hold on
    plot(time{i}, u{i},'-','LineWidth',lw);
    xlabel('t[s]');ylabel('u(t)');box on
end

%plotting the refernece
subplot(1,3,1);
stairs(time{i},theta_ref{i},'k--','LineWidth',lw);
leg_ext = leg;
leg_ext{end+1}='ref';
legend(leg_ext);
v=axis;
axis([v(1) v(2) -0.2 1.2])
grid

subplot(1,3,2);
legend(leg);
grid
subplot(1,3,3);
legend(leg);
grid