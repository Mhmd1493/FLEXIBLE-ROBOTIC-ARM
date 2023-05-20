J1 = 10/9;
J2 = 10; 
k = 1; 
d= 0.1; 
ki=1;
w0 = sqrt(k*(J1 + J2)/(J1*J2));

alpha=J1/(J1+J2);
beta1=d/(J1*w0);
beta2=d/(J2*w0);
gamma=ki/(J1*w0);
delta=1/(J1*w0);

% Define the system's state space representation
A = [0 1 -1; alpha-1 -beta1 beta1; alpha beta2 -beta2]; % state matrix
Anew=w0*A;
B = [0; gamma; 0;]; % input matrix
C = [0 0 w0]; % output matrix
D = 0; % feedthrough matrix

% Convert the state space representation to transfer function
SS_cont = ss(Anew, B, C, D);
tf_sys = tf(SS_cont);
p_cont=[-0.3577+0.159i -0.3577-0.159i -5]
place(Anew, B, p_cont)

%CONTROLLABLE CANONICAL FORM (CONTINUOUS)
Ac=[0 1 0; 0 0 1; 0 -1 -0.1];
Bc = [0; 0; 1;];
Cc=[0.09 0.009 0];
Dc = 0;
p=[-0.3557+0.159i -0.3557-0.159i -5]
Kappa_cont=place(Anew, B, p);
SS_CCF_cont = ss(Ac, Bc, Cc, Dc);
plant=SS_CCF_cont*Kappa_cont
isstable(plant);

% Display the transfer function
tf_sys %#ok<NOPTS>
sisotool(tf_sys)

%State space discretization
SS=ss(Anew,B,C,D)
SS_disc=c2d(SS, 0.5)
tf(SS_disc)

A_disc=[0.8796 0.4676 -0.4676; -0.4209 0.8495 0.1505; 0.0468 0.0167 0.9833;];
B_disc=[0.1084; 0.4238; 0.0029;];
C_disc=C;
D_disc=D;
p_disc=[0.997+0.00248i 0.997-0.00248i 0.997];
place(A_disc, B_disc, p_disc)

%CONTROLLABLE CANONICAL FORM (DISCRETE)
Ac_disc=[0 1 0; 0 0 1; 0.9512 -2.664 -2.712];
Bc_disc = [0; 0; 1;];
Cc_disc=[0.0007179 0.007116 002913];
Dc_disc = 0;
SS_CCF_disc=ss(Ac_disc, Bc_disc, Cc_disc, Dc_disc);