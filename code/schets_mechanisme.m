%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Een file exclusief gebruikt om een schets te maken van ons mechanisme in
% initiële toestand ter referentie.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function schets_mechanisme(r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,t,fig_kin_12bar)

    A = 0;
    warning("we gebruiken de eerste stap van phi2, dus -47°")
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
    
    
    figure
    assembly=[A, B, C, D, H, F, E, G, J, K, M, N]; %de volgorde lijkt mss wat raar, maar dan zijn de verbindingslijnen op de plot logischer
    plot(real(assembly),imag(assembly),'ro-')
    xlabel('[m]')
    ylabel('[m]')
    title('Walschaerts Valve Gear')
    axis equal