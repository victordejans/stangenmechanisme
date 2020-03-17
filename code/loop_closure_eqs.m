function F=loop_closure_eqs(phi_init,r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF,phi2)
%phi_init = column vector with the initial values of the varying angles
%phi2 = powered angle (on the wheel)

%fixed parameters (lengths and fixed angles)
r7=r7a+r7b;

%phi_init
phi3 = phi_init(1);
phi4 = phi_init(2);
phi6 = phi_init(3);
phi7 = phi_init(4);
phi8 = phi_init(5);
phi10 = phi_init(6);
phi12 = phi_init(7);
x9 = phi_init(8);
x11 = phi_init(9);
r4a = phi_init(10);

%loop closure equations

%van knoop A naar knoop F en terug
F(1) = r2c*cos(phi2+phiA)+r3*cos(phi3)+r4*cos(phi4)-r1b*cos(phiAF);
F(2) = r2c*sin(phi2+phiA)+r3*sin(phi3)+r4*sin(phi4)-r1b*sin(phiAF);
%van knoop E naar knoop A langs driehoek8 en terug
F(3) = -r6*cos(phi6)+r7*cos(phi7)-r8*cos(phi8)-r10*cos(phi10)-r12*cos(phi12)-r2b*cos(phi2)+r1a*cos(phiAE);
F(4) = -r6*sin(phi6)+r7*sin(phi7)-r8*sin(phi8)-r10*sin(phi10)-r12*sin(phi12)-r2b*sin(phi2)+r1a*sin(phiAE);
%van knoop A naar knoop E langs bar3 en terug
F(5) = r2c*cos(phi2+phiA)+r3*cos(phi3)+r4a*cos(phi4)-r7a*cos(phi7)+r6*cos(phi6)-r1a*cos(phiAE);
F(6) = r2c*sin(phi2+phiA)+r3*sin(phi3)+r4a*sin(phi4)-r7a*sin(phi7)+r6*sin(phi6)-r1a*sin(phiAE);
%van knoop A naar prisma 9 en terug
F(7) = r2c*cos(phi2+phiA)+r3*cos(phi3)+r4a*cos(phi4)+r7b*cos(phi7)-r8a*cos(phi8)-x9;
F(8) = r2c*sin(phi2+phiA)+r3*sin(phi3)+r4a*sin(phi4)+r7b*sin(phi7)-r8a*sin(phi8)-y9;
%van knoop A naar prisma 11 langs bar12 en terug
F(9) = r2b*cos(phi2)+r12*cos(phi12)-x11;
F(10) = r2b*sin(phi2)+r12*sin(phi12)-y11;
% %van prisma 11 naar prisma 9 via bar10 en terug
% F(11) = r10*cos(phi10)+r8b*cos(phi8)-x9+x11;
% F(12) = r10*sin(phi10)+r8b*sin(phi8)-y9+y11;


