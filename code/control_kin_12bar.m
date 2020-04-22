function control_kin_12bar(phi2,phi3,phi4,dphi2,dphi3,dphi4,ddphi2,ddphi3,ddphi4,...
                            r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r11,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,phi11,...
                            t)


% control of the  kinematics via position, speed and
% acceleration of point F

% position

F1 = r1b*exp(1i*phiAF); % position of F based on direct measurement from point A

F2 = r2c*exp(1i*(phi2 + phiA)) + r3*exp(1i*phi3) + r4*exp(1i*phi4); % Position of F by travelling from point A to C to D to F

figure
clf
hold on
plot(t,real(F1) - real(F2))
xlabel('[s]')
ylabel('[cm]')
title('error X position point F')

figure
clf
hold on
plot(t,imag(F1) - imag(F2))
xlabel('[s]')
ylabel('[cm]')
title('error Y position point F')

figure
clf
hold on
plot(t,1 - (real(F2)/real(F1)))
xlabel('[s]')
ylabel('[/]')
title(' Relative error X position point F')

figure
clf
hold on
plot(t,1 - (imag(F2)/imag(F1)))
xlabel('[s]')
ylabel('[/]')
title(' Relative error Y position point F')

% velocity

 omega2 = [zeros(size(dphi2)) zeros(size(dphi2)) dphi2];
 omega3 = [zeros(size(dphi3)) zeros(size(dphi3)) dphi3];
 omega4 = [zeros(size(dphi4)) zeros(size(dphi4)) dphi4];
 
 AC_vec = [r2c*cos(phi2 + phiA) r2c*sin(phi2 + phiA) zeros(size(phi2))];
 CD_vec = [r3*cos(phi3) r3*sin(phi3) zeros(size(phi3))];
 DF_vec = [r4*cos(phi4) r4*sin(phi4) zeros(size(phi4))];
 
 V_F = cross(omega2,AC_vec)+ cross(omega3,CD_vec) + cross(omega4,DF_vec);

 V_F_x = V_F(:,1);
 V_F_y = V_F(:,2); 
 
figure
clf
hold on
plot(t,V_F_x)
xlabel('[s]')
ylabel('[cm/s]')
title('error X velocity point F')

figure
clf
hold on
plot(t,V_F_y)
xlabel('[s]')
ylabel('[cm/s]')
title('error Y velocity point F')

% acceleration

 alpha2 = [zeros(size(ddphi2)) zeros(size(ddphi2)) ddphi2];
 alpha3 = [zeros(size(ddphi3)) zeros(size(ddphi3)) ddphi3];
 alpha4 = [zeros(size(ddphi4)) zeros(size(ddphi4)) ddphi4];
 
 A_F = cross(omega2,cross(omega2,AC_vec)) + cross(alpha2,AC_vec) + cross(omega3,cross(omega3,CD_vec)) + cross(alpha3,CD_vec) + cross(omega4,cross(omega4,DF_vec)) + cross(alpha4,DF_vec);
 
 A_F_x = A_F(:,1);
 A_F_y = A_F(:,2);

figure
clf
hold on
plot(t,A_F_x)
xlabel('[s]')
ylabel('[cm/s^2]')
title('error X acceleration point F')

figure
clf
hold on
plot(t,A_F_y)
xlabel('[s]')
ylabel('[cm/s^2]')
title('error Y acceleration point F')