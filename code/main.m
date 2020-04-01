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
fig_kin_12bar = 0;        % draw figures of kinematic analysis if 1
fig_dyn_12bar = 0;        % draw figures of dynamic analysis if 1

%link lengths in cm and fixed angles in degrees
r1a = 321; %between joint A and E
r1b = 385; %between joint A and F
r1c = 204; %between joint E and F
r2a = 65; %between joint B and C
r2b = 59; %between joint A and B
r2c = 27; %between joint A and C
r3 = 299;
r4 = 142;
r6 = 135;
r7a = 100;
r7b = 452;
r7 = r7a+r7b;
r8a = 30; %between joint J and K
r8b = 146; %between joint K and M
r8 = r8a+r8b;
r10 = 158;
r11 = 36;
r12 = 559;
y9 = 764*sin(deg2rad(8.37)); %vaste y-coordinaat van prisma 9 t.o.v. A
y11 = 599 * sin(deg2rad(0.29)); %vaste y-coordinaat van prisma 11 t.o.v. A
phiAE = deg2rad(69);
phiAF = phiAE - acos((r1a^2+r1b^2-r1c^2)/(2*r1a*r1b)); %cosinusregel
phiA = acos((r2c^2+r2b^2-r2a^2)/(2*r2c*r2b));
phiB = acos((r2a^2+r2b^2-r2c^2)/(2*r2a*r2b));
phiC = acos((r2a^2+r2c^2-r2b^2)/(2*r2a*r2c));
phi11 = 3*pi()/2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Kinematic analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%initial angles of links to horizon in degrees and initial positions of the sliders in cm
phi3_init = deg2rad(13);
phi4_init = deg2rad(91);
phi6_init = deg2rad(133);
phi7_init = deg2rad(354);
phi8_init = deg2rad(90);
phi10_init = deg2rad(348);
phi12_init = deg2rad(4.3);
x9_init = 764*cos(deg2rad(8.37));
x11_init = 599 * cos(deg2rad(0.29));
r4a_init = 104; %between joint D and H

%time
t_begin = 0;                   % start time of simulation
t_end = 10;                    % end time of simulation
Ts = 0.05;                     % time step of simulation
t = (t_begin:Ts:t_end)';       % time vector
number_of_time_samples = length(t);

%initialisation of driver (we drijven het wiel aan)
omega = pi();
phi2 = omega*t - 47*pi/180; %phi2 is dus een vector met alle waarden van phi2 op de verschillende tijdstippen. we trekken de initiele hoek phi2_init=47� af.
dphi2 = omega*ones(number_of_time_samples,1);
ddphi2 = zeros(number_of_time_samples,1);

[phi3,phi4,phi6,phi7,phi8,phi10,phi12,x9,x11,r4a,dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,dx9,dx11,dr4a,ddphi3,ddphi4,ddphi6,ddphi7,ddphi8,ddphi10,ddphi12,ddx9,ddx11,ddr4a] = ...
    kinematics_12bar(r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r11,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,phi11,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,x9_init,x11_init,r4a_init,t,fig_kin_12bar);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Movie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure
load 12bar_movie Movie
movie(Movie)


