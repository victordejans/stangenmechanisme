function [phi3,phi4,phi6,phi7,phi8,phi10,phi12,dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,...
    ddphi3,ddphi4,ddphi6,ddphi7,ddphi8,ddphi10,ddphi12] = ...
    kinematics_12bar(r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8c,r10,r12,phiA,phiB,phiC,phiJ,phiK,phiM,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,t,fig_kin_12bar)

% allocation of the result vectors (this results in better performance because we don't have to reallocate and
% copy the vector each time we add an element).
phi3 = zeros(size(t));
phi4 = zeros(size(t));
phi6 = zeros(size(t));
phi7 = zeros(size(t));
phi8 = zeros(size(t));
phi10 = zeros(size(t));
phi12 = zeros(size(t));
dphi3 = zeros(size(t));
dphi4 = zeros(size(t));
dphi6 = zeros(size(t));
dphi7 = zeros(size(t));
dphi8 = zeros(size(t));
dphi10 = zeros(size(t));
dphi12 = zeros(size(t));
ddphi3 = zeros(size(t));
ddphi4 = zeros(size(t));
ddphi6 = zeros(size(t));
ddphi7 = zeros(size(t));
ddphi8 = zeros(size(t));
ddphi10 = zeros(size(t));
ddphi12 = zeros(size(t));

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
        % argument phi2(k): input angle phi2 for which we want to calculate the unknown angles phi3 and phi4
        % argument a1 ... phi1: constants
        % return value x: solution for the unknown angles phi3 and phi4
        % return exitflag: indicates convergence of algorithm

        [x, fval, exitflag]=fsolve('loop_closure_eqs',[phi3_init phi4_init phi6_init phi7_init phi8_init phi10_init...
            phi12_init]',optim_options,phi2(k),r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8c,r10,r12,phiA,phiB,phiC,phiJ,phiK,phiM);
        if (exitflag ~= 1)
            display 'The fsolve exit flag was not 1, probably no convergence!'
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
    
    % *** velocity analysis ***
    
        A = [-r3*sin(phi3(k)),  r4*sin(phi4(k));
             r3*cos(phi3(k)), -r4*cos(phi4(k))];
        B = [ r2*sin(phi2(k))*dphi2(k);
             -r2*cos(phi2(k))*dphi2(k)];

        x = A\B;

        % save results
        dphi3(k) = x(1);
        dphi4(k) = x(2);
        dphi6(k) = x(3);
        dphi7(k) = x(4);
        dphi8(k) = x(5);
        dphi10(k) = x(6);
        dphi12(k) = x(7);
    
    % *** acceleration analysis ***
    
        A = [-r3*sin(phi3(k)),  r4*sin(phi4(k));
             r3*cos(phi3(k)), -r4*cos(phi4(k))];
        B = [r2*cos(phi2(k))*dphi2(k)^2+r2*sin(phi2(k))*ddphi2(k)+r3*cos(phi3(k))*dphi3(k)^2-r4*cos(phi4(k))*dphi4(k)^2;
             r2*sin(phi2(k))*dphi2(k)^2-r2*cos(phi2(k))*ddphi2(k)+r3*sin(phi3(k))*dphi3(k)^2-r4*sin(phi4(k))*dphi4(k)^2];

        x = A\B;

        % save results
        ddphi3(k) = x(1);
        ddphi4(k) = x(2);
        ddphi6(k) = x(3);
        ddphi7(k) = x(4);
        ddphi8(k) = x(5);
        ddphi10(k) = x(6);
        ddphi12(k) = x(7);
    
    % *** calculate initial values for next iteration step ***
    
        phi3_init = phi3(k)+Ts*dphi3(k);
        phi4_init = phi4(k)+Ts*dphi4(k);
        phi6_init = phi6(k)+Ts*dphi6(k);
        phi7_init = phi7(k)+Ts*dphi7(k);
        phi8_init = phi8(k)+Ts*dphi8(k);
        phi10_init = phi10(k)+Ts*dphi10(k);
        phi12_init = phi12(k)+Ts*dphi12(k);
    
end % loop over positions



% *** create movie ***

% point P = fixed
P = 0;
% point S = fixed
S = r1*exp(j*phi1);
% define which positions we want as frames in our movie
frames = 40;    % number of frames in movie
delta = floor(t_size/frames); % time between frames
index_vec = [1:delta:t_size]';

% Create a window large enough for the whole mechanisme in all positions, to prevent scrolling.
% This is done by plotting a diagonal from (x_left, y_bottom) to (x_right, y_top), setting the
% axes equal and saving the axes into "movie_axes", so that "movie_axes" can be used for further
% plots.
x_left = -1.5*r2;
y_bottom = -1.5*max(r2,r4);
x_right = r1+1.5*r4;
y_top = 1.5*max(r2,r4);

figure(10)
hold on
plot([x_left, x_right], [y_bottom, y_top]);
axis equal;
movie_axes = axis;   %save current axes into movie_axes

% draw and save movie frame
for m=1:length(index_vec)
    index = index_vec(m);
    Q = P + r2 * exp(j*phi2(index));
    R1 = Q + r3 * exp(j*phi3(index));
    R2 = S + r4 * exp(j*phi4(index));
    
    loop1 = [P Q R1 R2 S];
    
    figure(10)
    clf
    hold on
    plot(real(loop1),imag(loop1),'-o')
    
    axis(movie_axes);     % set axes as in movie_axes
    Movie(m) = getframe;  % save frame to a variable Film
end

% save movie
save fourbar_movie Movie
close(10)