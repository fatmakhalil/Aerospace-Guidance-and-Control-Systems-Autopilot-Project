clc; clear all; close all;
load('Der');
% matrix a %
LBS=(Lb*iz*ix)/(ix*iz-ixz^2);
Nbts=(Nb*ixz*iz)/(ix*iz-ixz^2);
Nbtts=(iz*ix*Nb)/(ix*iz-ixz^2);
LBTS=(ix*ixz*Lb)/(ix*iz-ixz^2);
Lpst=(Lp*iz*ix)/(ix*iz-ixz^2);
Npst=(Np*ixz*iz)/(ix*iz-ixz^2);
Npss=(iz*ix*Np)/(ix*iz-ixz^2);
Lpps=(ix*ixz*Lp)/(ix*iz-ixz^2);
Lrrs=(Lr*iz*ix)/(ix*iz-ixz^2);
Nrrs=(Nr*ixz*iz)/(ix*iz-ixz^2);
Nr_s=(iz*ix*Nr)/(ix*iz-ixz^2);
Lr_s=(ix*ixz*Lr)/(ix*iz-ixz^2);


% matrix b
Lds=Lda/(1-ixz^2/(ix*iz));
Nds=(Nda*ixz)/(ix*(1-ixz^2/(ix*iz)));
Ndds=(Nda)/(1-ixz^2/(ix*iz));
Ldss=(Lda/(1-ixz^2/(ix*iz)))*ixz/iz;
L_drs=Ldr/(1-ixz^2/(ix*iz));
N_drs=(Ndr*ixz)/(ix*(1-ixz^2/(ix*iz)));
Ndru=(Ndr)/(1-ixz^2/(ix*iz));
Ndrs=(Ndr)/(1-ixz^2/(ix*iz));
Ldrs=(Ldr/(1-ixz^2/(ix*iz)))*ixz/iz;


Alr=[Yb/(u0*cos(theta0))  Yp/(u0*cos(theta0))+tan(theta0)      -(1-Yr/(u0*cos(theta0)))   g*cos(theta0)/(u0*cos(theta0)) ;...
       LBS+Nbts            Lpst+Npst                                Lrrs+Nrrs              0     ;...
       Nbtts+LBTS           Npss+Lpps                                  Nr_s+Lr_s           0    ;...
           0                  1                                   tan(theta0)        0];
eig(Alr)

B_aileron=[Yda/(u0*cos(theta0))  0;Lds+Nds 0;Ndds+Ldss 0;0 0];
B_rudder=[0 Ydr/(u0*cos(theta0));0 L_drs+N_drs;0 Ndrs+Ldrs;0 0];

C=[1 0 0 0;0 1 0 0;0 0 1 0;0 0 0 1];
D=[0 0;0 0;0 0;0 0];

%%
%% for delta_ aileron %%
sys=ss(Alr,B_aileron,C,D);
TF=tf(sys)
phi_da=TF(4,1)
%%
% for delta_rudder %%%
sys2=ss(Alr,B_rudder,C,D);
TF2=tf(sys2)
r_dr=TF2(3,2)
phi_dr=TF2(4,2)
beta_dr=TF2(1,2)
p_dr=TF2(2,2)

s=tf('s');
epsi_phi=((1/s)*r_dr)/(phi_dr)
in_loop=(phi_da*15.028)/(1+(phi_da*15.028)) % feedback
out_loop=in_loop*(epsi_phi)
ay_dr=(s*u0*beta_dr)+(u0*r_dr) %w0=0
