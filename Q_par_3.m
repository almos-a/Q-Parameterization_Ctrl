% Q-parameterization control
clc
clear
close all

%transfer function and statespace model
num = [1 0 1];
den = [1 4 4];
P = tf(num,den)
pole(P)
%step(P)
%state space model
[A,B,C,E] = tf2ss(num,den)

%controller
%for internal stability
Q0 = 1/dcgain(P)
num1 = [1760 1600];
den1 = [1 40 400];
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
% figure
% step(sys)
% grid
% title('step response')
figure
t = 0:0.1:5;
y = lsim(sys,t,t);
plot(t,y)
grid
title('ramp response')

% %sinusoidal disturbance
% sys1 = P/(1+P*C);
% figure
% step(sys1)
% grid
% title('step disturbance response')

% %General form
% Ctrl = Q/(1-P*Q)
% pole(C)
% zero(C)
% C1 = realmin(Ctrl)