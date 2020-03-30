function F = acceleration_eqs(ddphi_init,...
        phi3,phi4,phi6,phi7,phi8,phi10,phi12,x9,x11,r4a,...
        dphi3,dphi4,dphi6,dphi7,dphi8,dphi10,dphi12,dx9,dx11,dr4a,...
        phi2,dphi2,ddphi2,r1a,r1b,r2a,r2b,r2c,r3,r4,r6,r7a,r7b,r8a,r8b,r8,r10,r12,y9,y11,phiA,phiB,phiC,phiAE,phiAF)



%fixed parameters
r7=r7a+r7b;

%dphi_init
ddphi3 = ddphi_init(1);
ddphi4 = ddphi_init(2);
ddphi6 = ddphi_init(3);
ddphi7 = ddphi_init(4);
ddphi8 = ddphi_init(5);
ddphi10 = ddphi_init(6);
ddphi12 = ddphi_init(7);
ddx9 = ddphi_init(8);
ddx11 = ddphi_init(9);
ddr4a = ddphi_init(10);

%acceleration equations
F(1) = -r2c*cos(phi2+phiA)*dphi2^2-r2c*sin(phi2+phiA)*ddphi2-r3*cos(phi3)*dphi3^2-r3*sin(phi3)*ddphi3-r4*cos(phi4)*dphi4^2-r4*sin(phi4)*ddphi4;
F(2) = -r2c*sin(phi2+phiA)*dphi2^2+r2c*cos(phi2+phiA)*ddphi2-r3*sin(phi3)*dphi3^2+r3*cos(phi3)*ddphi3-r4*sin(phi4)*dphi4^2+r4*cos(phi4)*ddphi4;
F(3) = r6*cos(phi6)*dphi6^2+r6*sin(phi6)*ddphi6-r7*cos(phi7)*dphi7^2-r7*sin(phi7)*ddphi7+r8*cos(phi8)*dphi8^2+r8*sin(phi8)*ddphi8+r10*cos(phi10)*dphi10^2+r10*sin(phi10)*ddphi10+r12*cos(phi12)*dphi12^2+r12*sin(phi12)*ddphi12+r2b*cos(phi2)*dphi2^2+r2b*sin(phi2)*ddphi2;
F(4) = r6*sin(phi6)*dphi6^2-r6*cos(phi6)*ddphi6-r7*sin(phi7)*dphi7^2+r7*cos(phi7)*ddphi7+r8*sin(phi8)*dphi8^2-r8*cos(phi8)*ddphi8+r10*sin(phi10)*dphi10^2-r10*cos(phi10)*ddphi10+r12*sin(phi12)*dphi12^2-r12*cos(phi12)*ddphi12+r2b*sin(phi2)*dphi2^2-r2b*cos(phi2)*ddphi2;
%warning('Als je zoekt naar typfouten: die hieronder tot en met de laatste term van phi3 is gekopieerd van F(1) en F(2)')
F(5) = -r2c*cos(phi2+phiA)*dphi2^2-r2c*sin(phi2+phiA)*ddphi2-r3*cos(phi3)*dphi3^2-r3*sin(phi3)*ddphi3+ddr4a*cos(phi4)-dr4a*sin(phi4)*dphi4-dr4a*sin(phi4)*dphi4-r4a*cos(phi4)*dphi4^2-r4a*sin(phi4)*ddphi4+r7a*cos(phi7)*dphi7^2+r7a*sin(phi7)*ddphi7-r6*cos(phi6)*dphi6^2-r6*sin(phi6)*ddphi6;
F(6) = -r2c*sin(phi2+phiA)*dphi2^2+r2c*cos(phi2+phiA)*ddphi2-r3*sin(phi3)*dphi3^2+r3*cos(phi3)*ddphi3+ddr4a*sin(phi4)+dr4a*cos(phi4)*dphi4+dr4a*cos(phi4)*dphi4-r4a*sin(phi4)*dphi4^2+r4a*cos(phi4)*ddphi4+r7a*sin(phi7)*dphi7^2-r7a*cos(phi7)*ddphi7-r6*sin(phi6)*dphi6^2+r6*cos(phi6)*ddphi6;
%warning('Als je zoekt naar typfouten: die hieronder tot en met de laatste term van phi4a is gekopieerd van F(5) en F(6)')
F(7) = -r2c*cos(phi2+phiA)*dphi2^2-r2c*sin(phi2+phiA)*ddphi2-r3*cos(phi3)*dphi3^2-r3*sin(phi3)*ddphi3+ddr4a*cos(phi4)-dr4a*sin(phi4)*dphi4-dr4a*sin(phi4)*dphi4-r4a*cos(phi4)*dphi4^2-r4a*sin(phi4)*ddphi4-r7b*cos(phi7)*dphi7^2-r7b*sin(phi7)*ddphi7+r8a*cos(phi8)*dphi8^2+r8a*sin(phi8)*ddphi8-ddx9;
F(8) = -r2c*sin(phi2+phiA)*dphi2^2+r2c*cos(phi2+phiA)*ddphi2-r3*sin(phi3)*dphi3^2+r3*cos(phi3)*ddphi3+ddr4a*sin(phi4)+dr4a*cos(phi4)*dphi4+dr4a*cos(phi4)*dphi4-r4a*sin(phi4)*dphi4^2+r4a*cos(phi4)*ddphi4-r7b*sin(phi7)*dphi7^2+r7b*cos(phi7)*ddphi7+r8a*sin(phi8)*dphi8^2-r8a*cos(phi8)*ddphi8;
F(9) = -r2b*cos(phi2)*dphi2^2-r2b*sin(phi2)*ddphi2-r12*cos(phi12)*dphi12^2-r12*sin(phi12)*ddphi12-ddx11;
F(10) = -r2b*sin(phi2)*dphi2^2+r2b*cos(phi2)*ddphi2-r12*sin(phi12)*dphi12^2+r12*cos(phi12)*ddphi12;