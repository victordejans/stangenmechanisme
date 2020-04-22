%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Een file exclusief gebruikt om een schets te maken van ons mechanisme in
% initiële toestand ter referentie.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function schets_mechanisme(r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,t,fig_kin_12bar)

    A = 0;
    %warning("we gebruiken de eerste stap van phi2, dus -47°")
    B = r2b*exp(1i*phi2(1));
    C = r2c*exp(1i*(phi2(1)+phiA));
    D = C + r3*exp(1i*phi3_init);
    E = r1a*exp(1i*phiAE);
    F = r1b*exp(1i*phiAF);
    G = E - r6*exp(1i*phi6_init);
    H = G + r7a*exp(1i*phi7_init);
    J = H + r7b*exp(1i*phi7_init);
    N = B + r12*exp(1i*phi12_init);
    M = N + r10*exp(1i*phi10_init);
    K = M + r8b*exp(1i*phi8_init);
    
    % Create a window large enough for the whole mechanisme in all positions, to prevent scrolling.
    % This is done by plotting a diagonal from (x_left, y_bottom) to (x_right, y_top), setting the
    % axes equal and saving the axes into "movie_axes", so that "movie_axes" can be used for further
    % plots.
    x_left = -r2b-50;
    y_bottom = -max(r2b,r10-y11)-50;
    x_right = r2b+r12+r10+50;
    y_top = max(r1a*sin(phiAE),r1b*sin(phiAF))+50;

    figure
    hold on
    plot([x_left, x_right], [y_bottom, y_top]);
    axis equal;
    movie_axes = axis;   %save current axes into movie_axes
    clf
    
    assembly=[A, B, C, A, C, D, H, F, H, G, E, G, J, K, M, N, B]; %de volgorde lijkt mss wat raar, maar dan zijn de verbindingslijnen op de plot zoals de staven in het reëele mechanisme
    plot(real(assembly),imag(assembly),'ro-')
    xlabel('[cm]')
    ylabel('[cm]')
    title('Walschaerts Valve Gear')
    axis(movie_axes);