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
phiA = deg2rad(97.4598);
phiB = deg2rad(21.7141);
phiC = deg2rad(60.8261);
phiJ = deg2rad(53.8022);
phiK = deg2rad(21.7141);
phiM = deg2rad(97.5498);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Kinematic analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial angles of links to horizon in degrees
phi3_init = deg2rad(13);
phi4_init = deg2rad(93);
phi6_init = deg2rad(133);
phi7_init = deg2rad(354);
phi8_init = deg2rad(66);
phi10_init = deg2rad(347);
phi12_init = deg2rad(5);

%time
t_begin = 0;                   % start time of simulation
t_end = 10;                    % end time of simulation
Ts = 0.05;                     % time step of simulation
t = [t_begin:Ts:t_end]';       % time vector

%initialisation of driver (we drijven het wiel aan)
omega = 0.5;
phi2 = omega*t - 47*pi/180; %phi2 is dus een vector met alle waarden van phi2 op de verschillende tijdstippen
dphi2 = omega;
ddphi2 = 0;

[phi3,phi4,phi6,phi7,phi8,phi10,phi12,dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,ddphi3,ddphi4,ddphi6,ddphi7,ddphi8,ddphi10,ddphi12] = ...
    kinematics_12bar(r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8c,r10,r12,phiA,phiB,phiC,phiJ,phiK,phiM,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,t,fig_kin_12bar);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Movie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% figure
% load 12bar_movie Movie
% movie(Movie)
% 

