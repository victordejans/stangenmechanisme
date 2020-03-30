function F=velocity_eqs(dphi_init,...
        phi3,phi4,phi6,phi7,phi8,phi10,phi12,x9,x11,r4a,...
        phi2,dphi2,r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF)
    
    
%fixed parameters
r7=r7a+r7b;

%dphi_init
dphi3 = dphi_init(1);
dphi4 = dphi_init(2);
dphi6 = dphi_init(3);
dphi7 = dphi_init(4);
dphi8 = dphi_init(5);
dphi10 = dphi_init(6);
dphi12 = dphi_init(7);
dx9 = dphi_init(8);
dx11 = dphi_init(9);
dr4a = dphi_init(10);

%velocity equations
F(1) = -r2c*sin(phi2+phiA)*dphi2-r3*sin(phi3)*dphi3-r4*sin(phi4)*dphi4;
F(2) = r2c*cos(phi2+phiA)*dphi2+r3*cos(phi3)*dphi3+r4*cos(phi4)*dphi4;
F(3) = r6*sin(phi6)*dphi6-r7*sin(phi7)*dphi7+r8*sin(phi8)*dphi8+r10*sin(phi10)*dphi10+r12*sin(phi12)*dphi12+r2b*sin(phi2)*dphi2;
F(4) = -r6*cos(phi6)*dphi6+r7*cos(phi7)*dphi7-r8*cos(phi8)*dphi8-r10*cos(phi10)*dphi10-r12*cos(phi12)*dphi12-r2b*cos(phi2)*dphi2;
F(5) = -r2c*sin(phi2+phiA)*dphi2-r3*sin(phi3)*dphi3+dr4a*cos(phi4)-r4a*sin(phi4)*dphi4+r7a*sin(phi7)*dphi7-r6*sin(phi6)*dphi6;
F(6) = r2c*cos(phi2+phiA)*dphi2+r3*cos(phi3)*dphi3+dr4a*sin(phi4)+r4a*cos(phi4)*dphi4-r7a*cos(phi7)*dphi7+r6*cos(phi6)*dphi6;
%warning('Als je zoekt naar typfouten: die hieronder tot en met de laatste term van phi4a is gekopieerd van F(5) en F(6)')
F(7) = -r2c*sin(phi2+phiA)*dphi2-r3*sin(phi3)*dphi3+dr4a*cos(phi4)-r4a*sin(phi4)*dphi4-r7b*sin(phi7)*dphi7+r8a*sin(phi8)*dphi8-dx9;
F(8) = r2c*cos(phi2+phiA)*dphi2+r3*cos(phi3)*dphi3+dr4a*sin(phi4)+r4a*cos(phi4)*dphi4+r7b*cos(phi7)*dphi7-r8a*cos(phi8)*dphi8;
F(9) = -r2b*sin(phi2)*dphi2-r12*sin(phi12)*dphi12-dx11;
F(10) = r2b*cos(phi2)*dphi2+r12*cos(phi12)*dphi12;



