function [phi3,phi4,phi6,phi7,phi8,phi10,phi12,x9,x11,r4a,dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,dx9,dx11,dr4a,ddphi3,ddphi4,ddphi6,ddphi7,ddphi8,ddphi10,ddphi12,ddx9,ddx11,ddr4a] = ...
    kinematics_12bar(r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r11,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,phi11,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,x9_init,x11_init,r4a_init,t,fig_kin_12bar)

% allocation of the result vectors (this results in better performance because we don't have to reallocate and
% copy the vector each time we add an element).
phi3 = zeros(size(t));
phi4 = zeros(size(t));
phi6 = zeros(size(t));
phi7 = zeros(size(t));
phi8 = zeros(size(t));
phi10 = zeros(size(t));
phi12 = zeros(size(t));
x9 = zeros(size(t));
x11 = zeros(size(t));
r4a = zeros(size(t));
dphi3 = zeros(size(t));
dphi4 = zeros(size(t));
dphi6 = zeros(size(t));
dphi7 = zeros(size(t));
dphi8 = zeros(size(t));
dphi10 = zeros(size(t));
dphi12 = zeros(size(t));
dx9 = zeros(size(t));
dx11 = zeros(size(t));
dr4a = zeros(size(t));
ddphi3 = zeros(size(t));
ddphi4 = zeros(size(t));
ddphi6 = zeros(size(t));
ddphi7 = zeros(size(t));
ddphi8 = zeros(size(t));
ddphi10 = zeros(size(t));
ddphi12 = zeros(size(t));
ddx9 = zeros(size(t));
ddx11 = zeros(size(t));
ddr4a = zeros(size(t));

% fsolve options (help fsolve, help optimset)
optim_options = optimset('Display','off');

% *** loop over positions ***
Ts = t(2) - t(1);      % timestep
t_size = size(t,1);    % number of simulation steps
for k=1:t_size
    
    % *** position analysis ***
    
        % fsolve solves the non-linear set of equations
        % loop closure equations: see loop_closure_eqs.m
        % argument loop_closure_eqs: file containing closure equations
        % argument [..]': initial values of unknown angles phi3 and phi4
        % argument optim options: parameters for fsolve
        % argument phi2(k): input angle phi2 for which we want to calculate the unknown angles and lengths
        % argument a1 ... phi1: constants
        % return value x: solution for the unknown angles phi3 and phi4
        % return exitflag: indicates convergence of algorithm

        [x, ~, exitflag]=fsolve('loop_closure_eqs',...
            [phi3_init phi4_init phi6_init phi7_init phi8_init phi10_init,phi12_init,x9_init,x11_init,r4a_init],...
            optim_options,...
            phi2(k),r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r11,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,phi11);

        if (exitflag ~= 1)
            disp 'The fsolve exit flag was not 1, probably no convergence!'
            exitflag
        end

        % save results of fsolve
        phi3(k)=x(1);
        phi4(k)=x(2);
        phi6(k)=x(3);
        phi7(k)=x(4);
        phi8(k)=x(5);
        phi10(k)=x(6);
        phi12(k)=x(7);
        x9(k)=x(8);
        x11(k)=x(9);
        r4a(k)=x(10);
    
    % *** velocity analysis ***

        dphi_init = zeros(1,10);
        
        [x, ~, exitflag]=fsolve('velocity_eqs',...
            dphi_init,optim_options,...
            phi3(k),phi4(k),phi6(k),phi7(k),phi8(k),phi10(k),phi12(k),x9(k),x11(k),r4a(k),...
            phi2(k),dphi2(k),r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF);

        if (exitflag ~= 1)
            disp 'The fsolve exit flag was not 1, probably no convergence!'
            exitflag
        end
        
        % save results
        dphi3(k) = x(1);
        dphi4(k) = x(2);
        dphi6(k) = x(3);
        dphi7(k) = x(4);
        dphi8(k) = x(5);
        dphi10(k) = x(6);
        dphi12(k) = x(7);
        dx9(k)=x(8);
        dx11(k)=x(9);
        dr4a(k)=x(10);
    
    % *** acceleration analysis ***
        
        ddphi_init = zeros(1,10);
        
        [x, ~, exitflag]=fsolve('acceleration_eqs',...
            ddphi_init,optim_options,...
            phi3(k),phi4(k),phi6(k),phi7(k),phi8(k),phi10(k),phi12(k),x9(k),x11(k),r4a(k),...
            dphi3(k),dphi4(k),dphi6(k),dphi7(k),dphi8(k),dphi10(k),dphi12(k),dx9(k),dx11(k),dr4a(k),...
            phi2(k),dphi2(k),ddphi2(k),r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF);

        if (exitflag ~= 1)
            disp 'The fsolve exit flag was not 1, probably no convergence!'
            exitflag
        end
        
        % save results
        ddphi3(k) = x(1);
        ddphi4(k) = x(2);
        ddphi6(k) = x(3);
        ddphi7(k) = x(4);
        ddphi8(k) = x(5);
        ddphi10(k) = x(6);
        ddphi12(k) = x(7);
        ddx9(k)=x(8);
        ddx11(k)=x(9);
        ddr4a(k)=x(10);    
        
    % *** calculate initial values for next iteration step ***
    
        phi3_init = phi3(k)+Ts*dphi3(k);
        phi4_init = phi4(k)+Ts*dphi4(k);
        phi6_init = phi6(k)+Ts*dphi6(k);
        phi7_init = phi7(k)+Ts*dphi7(k);
        phi8_init = phi8(k)+Ts*dphi8(k);
        phi10_init = phi10(k)+Ts*dphi10(k);
        phi12_init = phi12(k)+Ts*dphi12(k);
        x9_init = x9(k)+Ts*dx9(k);
        x11_init = x11(k)+Ts*dx11(k);
        r4a_init = r4a(k)+Ts*dr4a(k);
    
end % loop over positions


% *** create movie ***

if fig_kin_12bar == 1

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
title(' Absolute error X position point F')

figure
clf
hold on
plot(t,imag(F1) - imag(F2))
xlabel('[s]')
ylabel('[cm]')
title(' Absolute error Y position point F')

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
title(' Absolute error X velocity point F')

figure
clf
hold on
plot(t,V_F_y)
xlabel('[s]')
ylabel('[cm/s]')
title(' Absolute error Y velocity point F')

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
title(' Absolute error X acceleration point F')

figure
clf
hold on
plot(t,A_F_y)
xlabel('[s]')
ylabel('[cm/s^2]')
title(' Absolute error Y acceleration point F')

% Movie
    
% define which positions we want as frames in our movie
frames = 200;    % number of frames in movie
delta = floor(t_size/frames); % time between frames
index_vec = [1:delta:t_size]';

%fixed points
number_of_time_samples = length(phi2);
A = 0; %centrum van wiel, oorsprong van het systeem
E = r1a*exp(1i*phiAE); %1i is the imaginary constant
F = r1b*exp(1i*phiAF);

% Create a window large enough for the whole mechanisme in all positions, to prevent scrolling.
% This is done by plotting a diagonal from (x_left, y_bottom) to (x_right, y_top), setting the
% axes equal and saving the axes into "movie_axes", so that "movie_axes" can be used for further
% plots.
x_left = -r2b-50;
y_bottom = -max(r2b,r10-y11)-50;
x_right = r2b+r12+r10+50;
y_top = max(r1a*sin(phiAE),r1b*sin(phiAF))+50;

figure(10)
hold on
plot([x_left, x_right], [y_bottom, y_top]);
axis equal;
movie_axes = axis;   %save current axes into movie_axes

% draw and save movie frame
for m=1:length(index_vec)
    index = index_vec(m);
    %moving points
    B = r2b*exp(1i*phi2(index));
    C = r2c*exp(1i*(phi2(index)+phiA));
    D = C + r3*exp(1i*phi3(index));
    G = E - r6*exp(1i*phi6(index));
    H = G + r7a*exp(1i*phi7(index));
    J = H + r7b*exp(1i*phi7(index));
    N = B + r12*exp(1i*phi12(index));
    O = N + r11*exp(1i*phi11);
    M = N + r10*exp(1i*phi10(index));
    K = M + r8b*exp(1i*phi8(index));
    
    loop1 = [A B C A];
    loop2 = [C D H F];
    loop3 = [E G H J K M O N B];
    
    figure(10)
    clf
    hold on
    plot(real(loop1),imag(loop1),'-o')
    hold on
    plot(real(loop2),imag(loop2),'-o')
    hold on
    plot(real(loop3),imag(loop3),'-o')
    
    axis(movie_axes);     % set axes as in movie_axes
    Movie(m) = getframe;  % save frame to a variable Film
end

% save movie
save 12bar_movie Movie
close(10)

end

end
