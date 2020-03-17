%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Een file exclusief gebruikt om een schets te maken van ons mechanisme in
% initiële toestand ter referentie.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [phi3,phi4,phi6,phi7,phi8,phi10,phi12,dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,...
    ddphi3,ddphi4,ddphi6,ddphi7,ddphi8,ddphi10,ddphi12] = ...
    schets_mechanisme(r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,...
    phi2,dphi2,ddphi2,phi3_init,phi4_init,phi6_init,phi7_init,phi8_init,phi10_init,phi12_init,t,fig_kin_12bar)

    A = 0;
    B = r2b*exp(j*phi2);
    C = r2c*exp(j*(phi2+phiA));
    D = C + r3*exp(j*phi3_init);
    E = r1a*exp(j*phiAE);
    F = r1b*exp(j*phiAF);
    G = E - r6*exp(j*phi6_init);
    H = G + r7a*exp(j*phi7_init);
    
    figure
    assembly=[A, B, C, D, E, F, G, H, J, K, M, N];
    plot(real(assembly),imag(assembly),'ro-')
    xlabel('[m]')
    ylabel('[m]')
    title('Walschaerts Valve Gear')
    axis equal