function F=loop_closure_eqs(phi_init,phi2)
%phi_init = column vector with the initial values of the varying angles
%phi2 = powered angle (on the wheel)

%fixed parameters (lengths and fixed angles)
r2a = 67; %between joint 2 and 3
r2b = 59; %between joint 1 and 2
r2c = 25; %between joint 1 and 3
r3 = 300;
r4 = 148;
r6 = 138;
r7a = 95;
r7b = 450;
r7 = r7a+r7b;
r8a = 174; %between joint 10 and 13
r8b = 150; %between joint 11 and 13
r8c = 50; %between joint 10 and 11
r10 = 156;
r12 = 564;
phiA = 97.4598;
phiB = 21.7141;
phiC = 60.8261;
phiJ = 53.8022;
phiK = 21.7141;
phiL = 97.5498;

%phi_init
phi3 = phi_init(1);
phi4 = phi_init(2);
phi6 = phi_init(3);
phi7 = phi_init(4);
phi8 = phi_init(5);
phi10 = phi_init(6);
phi12 = phi_init(7);

