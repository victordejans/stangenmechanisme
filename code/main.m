%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Stangenmechanisme Walchaerts Valve Gear

%Beweging en trillingen 2019-2020

%Thomas Brzeski
%Victor Dejans

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close
clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Data initialisation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% program data
fig_kin_12bar = 1;        % draw figures of kinematic analysis if 1
fig_dyn_12bar = 0;        % draw figures of dynamic analysis if 1

%link lengths in cm and fixed angles in degrees
r2a = 67; %between joint 2 and 3
r2b = 59; %between joint 1 and 2
r2c = 25; %between joint 1 and 3
r3 = 300;
r4 = 148;
r6 = 138;
r7a = 95;
r7b = 450;
r7 = r7a+r7b;
r8a = 174; %between joint 10 and 13
r8b = 150; %between joint 11 and 13
r8c = 50; %between joint 10 and 11
r10 = 156;
r12 = 564;
phiA = 97.4598;
phiB = 21.7141;
phiC = 60.8261;
phiJ = 53.8022;
phiK = 21.7141;
phiL = 97.5498;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Kinematic analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial angles of links to horizon in degrees
theta3_init = 13;
theta4_init = 93;
theta6_init = 133;
theta7_init = 354;
theta8_init = 66;
theta10_init = 347;
theta12_init = 5;

%time
t_begin = 0;                   % start time of simulation
t_end = 10;                    % end time of simulation
Ts = 0.05;                     % time step of simulation
t = [t_begin:Ts:t_end]';       % time vector

%initialisation of driver (we drijven het wiel aan)
omega = 0.5;
theta2 = omega*t - 47*pi/180;
dtheta2 = omega;
ddtheta2 = 0;


