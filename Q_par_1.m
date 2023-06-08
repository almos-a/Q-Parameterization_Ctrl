% Q-parameterization control
clc
clear
close all

%plant transfer function
num = 1;
den = [1 -3 2];
P = tf(num,den)
pole(P)
%state space model
[A,B,C,E]=tf2ss(num,den)

%feedback design using pole placement
J=[-10 -20]; % setting the closed loop poles
F=place(A,B,J)
L=[-40 -50]; % setting the closed loop poles
H=place(A',C',L)'

%closed loop system
Ao = A-B*F
Aotilde = A-H*C

%double coprime factorisation
xp = tf(ss(Aotilde,H,F,0))
yp = tf(ss(Aotilde,(B-H*E),F,1))
xptilde = tf(ss(Ao,H,F,0))
yptilde = tf(ss(Ao,H,(C-E*F),1))
np = tf(ss(Ao,B,(C-E*F),E))
dp = tf(ss(Ao,B,-F,1))
nptilde = tf(ss(Aotilde,(B-H*E),C,E))
dptilde = tf(ss(Aotilde,H,-C,1))

%internally stable system
q0 = dcgain(yp)/dcgain(nptilde)

%sinusoidal disturbance rejection
[ny,dy] = ss2tf(Aotilde,(B-H*E),F,1);
ypw = polyval(ny,10*j)/polyval(dy,10*j);
[nt,dt] = ss2tf(Aotilde,(B-H*E),C,E);
ntpw = polyval(nt,10*j)/polyval(dt,10*j);
qw = ypw/ntpw

%Q and Controller transfer function
Q = tf([1.0487e+04 2.4358e+05 2.1068e+06],[1 40 400]);
[Aq,Bq,Cq,Dq] = tf2ss([1.0487e+04 2.4358e+05 2.1068e+06],[1 40 400]);
Ak = [A-B*F-H*C+B*Dq*C -B*Cq;-Bq*C Aq];
Bk = [H-B*Dq;Bq];
Ck = [F-Dq*C Cq];
Dk = Dq;
C = tf(ss(Ak,Bk,Ck,Dk))
pole(C)

% %alternative controller design
% %min(real) applied with general form to give tf with pole zero cancellation
% %applied
% Sp = (xp+Q*dptilde)/(yp-Q*nptilde);
% C = minreal(Sp) 

%System;
sys = P*C/(1+P*C);
figure
step(sys)
grid
title('step response')

%sinusoidal disturbance
sys1 = P/(1+P*C);
figure
step(sys1)
grid
title('step disturbance response')

figure
t = linspace(0,1);
u = sin(10*t);
y = lsim(sys1,u,t);
plot(t,y)
grid
title('system disturbance response at w=10')

figure
t = linspace(0,1);
u = sin(20*t);
y = lsim(sys1,u,t);
plot(t,y)
grid
title('system disturbance response at w=20')
