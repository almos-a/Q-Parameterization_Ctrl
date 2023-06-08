% Q-parameterization control
clc
clear
close all

%Plant transfer function
num=  4;
den = [1 3 4];
P = tf(num,den)
pole(P)
%state space model
[A,B,C,E] = tf2ss(num,den)

%stable controller transfer function
%internal stability
Q0 = 1/dcgain(P)

%sinusoidaldisturbance rejection
Qw = 1/(polyval(num,j*10)/polyval(den,j*10))

%controller transfer function
num1 = [106 -735 400];
den1 = [1 40 400]
Q = tf(num1,den1)
[Aq,Bq,Cq,Dq] = tf2ss(num1,den1);

F = zeros(1,2);
H = zeros(2,1);
Ak = [A-B*F-H*C+B*Dq*C -B*Cq;-Bq*C Aq];
Bk = [H-B*Dq;Bq];
Ck = [F-Dq*C Cq];
Dk = Dq;
C = tf(ss(Ak,Bk,Ck,Dk))
pole(C)

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
t = linspace(0,10);
u = sin(10*t);
y = lsim(sys1,u,t);
plot(t,y)
grid
title('system disturbance response at w=10')
figure
t = linspace(0,10);
u = sin(20*t);
y = lsim(sys1,u,t);
plot(t,y)
grid
title('system disturbance response at w=20')


% %alternative approach to controller tf
% Cp=Q/(1-P*Q)
% C=minreal(Cp)