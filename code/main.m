%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Stangenmechanisme Walchaerts Valve Gear

%Beweging en trillingen 2019-2020

%Thomas Brzeski
%Victor Dejans

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Data initialisation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% program data
fig_kin_12bar = 1;        % draw figures of kinematic analysis if 1
fig_dyn_12bar = 1;        % draw figures of dynamic analysis if 1
movie_12bar = 1;
contr_kin_12bar = 1;
contr_dyn_12bar = 1;

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
r6a = 95; %1 zijde van de L-vormige stang
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

%dynamic parameters

straal_zuiger9 = 19;
dikte_zuiger9 = 91;
inhoud_zuiger9 = dikte_zuiger9 * straal_zuiger9^2*pi();
lengte_stang9 = 178;
inhoud_blokje9 = 0;

straal_zuiger11 = 50;
dikte_zuiger11 = 10;
inhoud_zuiger11 = dikte_zuiger11 * straal_zuiger11^2*pi();
lengte_stang11 = 221;
x_blokje11 = 50;
y_blokje11 = 60;
z_blokje11 = 40;
inhoud_blokje11 = x_blokje11 * y_blokje11 * z_blokje11;

straalWiel = 176;
sgStaal = 7800/10^6; %kg per cm^3 (onze afmetingen staan ook in cm)
doorsnedeStang = 10^2 * pi(); %oppervlak van doorsnede van een stang in cm^2, veronderstel dat alle stangen even dik zijn

X2 = 0;
Y2 = 0;
m2 = sgStaal * straalWiel^2 * pi() * 5; %wiel is 5cm dik
J2 = 0.5 * m2 * straalWiel;

X3 = r3/2;
Y3 = 0;
m3 = sgStaal * doorsnedeStang * r3;
J3 = m3*r3^2/12;

X4 = r4/2;
Y4 = 0;
m4 = sgStaal * doorsnedeStang * r4;
J4 = m4*r4^2/12;

X5 = 0;
Y5 = 0;
m5 = 0;
J5 = 0;

X6 = r6/2;
Y6 = r6a/2*cos(pi()/4); %door de knik in deze stang
m6 = sgStaal * doorsnedeStang * 2*r6a;
J6 = 2*(m6/2)*r6a^2/3;

X7 = r7/2;
Y7 = 0;
m7 = sgStaal * doorsnedeStang * r7;
J7 = m7*r7^2/12;

X8 = r8/2;
Y8 = 0;
m8 = sgStaal * doorsnedeStang * r8;
J8 = m8*r8^2/12;

m_blokje9 = sgStaal * inhoud_blokje9;
m_stang9 = sgStaal * doorsnedeStang * lengte_stang9;
m_zuiger9 = sgStaal * inhoud_zuiger9;
m9 = m_blokje9 + m_stang9 + m_zuiger9;
X9 = (m_blokje9 * 0 + m_stang9 * lengte_stang9/2 + m_zuiger9 * lengte_stang9)/m9;
Y9 = (m_blokje9 * 0 + m_stang9 * 0 + m_zuiger9 * 0)/m9;
J9 = (m_blokje9 * 0) + ...
     (m_stang9 * lengte_stang9^2 / 3 + m_stang9 * X9^2) + ...
     (m_zuiger9 * (straal_zuiger9^2/4 + dikte_zuiger9^2/12) + m_zuiger9 * (X9-lengte_stang9)^2);

X10 = r10/2;
Y10 = 0;
m10 = sgStaal * doorsnedeStang * r10;
J10 = m10*r10^2/12;

m_blokje11 = sgStaal * inhoud_blokje11;
m_stang11 = sgStaal * doorsnedeStang * lengte_stang11;
m_zuiger11 = sgStaal * inhoud_zuiger11;
m11 = m_blokje11 + m_stang11 + m_zuiger11;
X11 = (m_blokje11 * 0 + m_stang11 * lengte_stang11/2 + m_zuiger11 * lengte_stang11)/m11;
Y11 = (m_blokje11 * -10 + m_stang11 * 0 + m_zuiger11 * 0)/m11;
J11 = (m_blokje11 * (x_blokje11^2 + y_blokje11^2)/12 + m_blokje11 * (X11^2 + (Y11+10)^2)) + ...
      (m_stang11 * lengte_stang11^2 / 3 + m_stang11 * X11^2) + ...
      (m_zuiger11 * (straal_zuiger11^2/4 + dikte_zuiger11^2/12) + m_zuiger11 * (X11-lengte_stang11)^2);

X12 = r12/2;
Y12 = 0;
m12 = sgStaal * doorsnedeStang * r12;
J12 = m12*r12^2/12;

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
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,x9_init,x11_init,r4a_init,t,fig_kin_12bar,contr_kin_12bar,movie_12bar);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Dynamic analysis
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%info over de joints:   H is het scharnier dat 7 rond 5 doet draaien
%                       I is de prismatische joint van 5 op 4
%                       K is het scharnier tussen 8 en 9
%                       L is de prismatische joint van 9 op de grond
%                       N is het scharnier van 12 op 11
%                       O is het scharnier van 10 op 11
%                       P is de prismatic van 11 op de grond

[FAx,FAy,MAz,FBx,FBy,FCx,FCy,FDx,FDy,FEx,FEy,FFx,FFy,FGx,FGy,FHx,FHy,FIn,MIz,FJx,FJy,FKx,FKy,FLy,MLz,FMx,FMy,FNx,FNy,FOx,FOy,FPy,MPz]...
    = dynamics_12bar(phi2,phi3,phi4,phi6,phi7,phi8,phi10,phi12,x9,x11,r4a,...
                     dphi2,dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,dx9,dx11,dr4a,...
                     ddphi2,ddphi3,ddphi4,ddphi6,ddphi7,ddphi8,ddphi10,ddphi12,ddx9,ddx11,ddr4a,...
                     r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r11,r12,y9,y11,...
                     phiA,phiB,phiC,phiAE,phiAF,phi11,...
                     m2,m3,m4,m5,m6,m7,m8,m9,m10,m11,m12,...
                     X2,X3,X4,X5,X6,X7,X8,X9,X10,X11,X12,...
                     Y2,Y3,Y4,Y5,Y6,Y7,Y8,Y9,Y10,Y11,Y12,...
                     J2,J3,J4,J5,J6,J7,J8,J9,J10,J11,J12,...
                     lengte_stang9,straal_zuiger9,lengte_stang11,straal_zuiger11,...
                     t,fig_dyn_12bar,contr_dyn_12bar);
                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Movie
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if movie_12bar

    figure
    load 12bar_movie Movie
    movie(Movie)

    % schets van het mechanisme in initi�le toestand
    schets_mechanisme(r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r10,r11,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,phi11,...
        phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,t,fig_kin_12bar)

end