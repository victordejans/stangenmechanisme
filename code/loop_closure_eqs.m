function F=loop_closure_eqs(phi_init,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8c,r10,r12,phiA,phiB,phiC,phiJ,phiK,phiM,phi2)
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

%loop closure equations

%driehoek 2
F(1) = r2c*cos(phi2+phiA)-r2a*cos(phi2+phiA+phiC)-r2b*cos(phi2);
F(2) = r2c*sin(phi2+phiA)-r2a*sin(phi2+phiA+phiC)-r2b*sin(phi2);
%driehoek8
F(3) = r8a*cos(phi8+phiM)-r8c*cos(phi8+phiM+phiJ)-r8b*cos(phi8);
F(4) = r8a*sin(phi8+phiM)-r8c*sin(phi8+phiM+phiJ)-r8b*sin(phi8);
%van knoop A naar knoop F
F(5) = r2c*cos(phi2+phiA)+r3*cos(phi3)+r4*cos(phi4);
F(6) = r2c*sin(phi2+phiA)+r3*sin(phi3)+r4*sin(phi4);
%van knoop E naar knoop A langs driehoek8
F(7) = -r6*cos(phi6)+r7*cos(phi7)-r8a*cos(phi8+phiM)-r10*cos(phi10)-r12*cos(phi12)-r2b*cos(phi2);
F(8) = -r6*sin(phi6)+r7*sin(phi7)-r8a*sin(phi8+phiM)-r10*sin(phi10)-r12*sin(phi12)-r2b*sin(phi2);