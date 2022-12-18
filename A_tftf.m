%% MKM506E | Modelling and Control of Mechanical Systems
%% Lecturer: Prof.Dr.Ata Muðan
%% Abdurrahim Bilal Özcan | 514191026
%% 
clc,clear all

% num=-0.3924;
% denum=[1 0 0];
% 
% [A,B,C,D]=tf2ss(num,denum)
%% State-Space Model
% A=[0 0 1;0 0 0;0 0 0];
% B=[0;0;1];
% C=[0 0 -1.72656;0 0 -0.3924;0 0 -1.96];
% D=[0;0;0];
% 
% [NUM,DENUM]=ss2tf(A,B,C,D)
%% Transfer Function

s=tf('s');
% TrafFnc=C*inv(eye(3)*s^2-A)*B+D;

%% Controllability and Observability
% 
% Qb=obsv(A,C);
% if(rank(Qb)== rank(A))
%     disp('Given System is Observable')
% else
%      disp('Given System is not Observable')
% end
% 
% Cr=ctrb(A,B);
% if(rank(Cr)== rank(A))
%     disp('Given System is Controllable')
% else
%      disp('Given System is not Controllable')
% end

%% Poles ans zeros of the system with without DC Motor
% With DC Motor
%For q_1
% DCnum=[2 0.1];
% DCdenum=[40 5];
% DCtf=tf(DCnum,DCdenum);
% 
% sysnumq1=[-1.7658];
% sysdenumq1=[1 0 0];
% systf=tf(sysnumq1,sysdenumq1);
% 
% Gs=series(-0.02*DCtf,systf);
% controlSystemDesigner('bode',Gs);  %Bode Diagram for q_1
% margin(Gs);
% [Gm,Pm,Wcg,Wcp] = margin(Gs); % Gain and phase margins and crossover frequencies
% fb = bandwidth(Gs)
% controlSystemDesigner('bode',Gs);  %Bode Diagram for q_1

% pzmap(Gs)
% 
%For q_3
% sysnumq2=[-0.3924];
% sysdenumq2=[1 0 0];
% systf=tf(sysnumq2,sysdenumq2);
% 
% Gs2=series(-0.02*DCtf,systf)
% pzmap(Gs2)

%% Without DC Motor

% numq1=[-1.7658];
% denumq1=[1 0 0];
% q1tf=tf(numq1,denumq1);
% pzmap(q1tf)
% 
% numq3=[-0.3924];
% denumq3=[1 0 0];
% q3tf=tf(numq3,denumq3);
% pzmap(q3tf)

% numq2=[-49.05];
% denumq2=[25 0 0];
% q2tf=tf(numq2,denumq2);
% pzmap(q2tf)

%% Ziegler - Nichols Method
% KU=180;    %From Simulation
% TU=3.9;     %From Simulation
% [KP, KI, KD] = ZieglerNichols(KU,TU,'NoOvershoot')

%% Poles-Zeros Map of Closed-Loop System

% % For q1
% DCnum=[2 0.1];
% DCdenum=[40 5];
% DCtf=tf(DCnum,DCdenum);
% 
% sysnumq1=[-1.7658];
% sysdenumq1=[1 0 0];
% systf=tf(sysnumq1,sysdenumq1);
% 
% TF=series(DCtf,systf);
% % 
% % C=pid(350.7,32.6,893,2);
% % 
% Control=series(DCtf,systf);
% 

% last=feedback(C*Control,-1); %feedback
% controlSystemDesigner('bode',last);  %Bode Diagram for q_1

% For q3
% DCnum=[2 0.1];
% DCdenum=[40 5];
% DCtf=tf(DCnum,DCdenum);
% 
% numq3=[-0.3924];
% denumq3=[1 0 0];
% q3tf=tf(numq3,denumq3);
% C=pid(350.7,32.6,893,2);
% 
% Control=series(DCtf,systf);
% 
% last=feedback(C*Control,-1); %feedbask
% 
% % controlSystemDesigner('bode',last);  %Bode Diagram for q_1


%% LQR Controller Design


% num=[-3.532 -0.1766];
% denum=[40 5 0 0];
% [A,B,C,D]=tf2ss(num,denum);  % I pass state-space form from transfer function
% 
A1=[-0.125 0 0;1 0 0;0 1 0];
B1=[1;0;0];
C1=[0 -0.0883 -0.0044];
D1=0;
Q=10^-7*[100 0 0;0 0.001 0;0 0 1];
R=[1];
K=lqr(A1,B1,Q,R);    %   K = 11.5605   18.2677   10.0000
% 
sys_cl = ss(A1-B1*K, B1, C1, D1);
step(sys_cl) %Closed loop 
 


%% Poles and Zeros of the LQR Controlled System 

% TfLQR=K*inv(s*eye(3)-A1)*B1
% pole(TfLQR)
% zero(TfLQR)
% controlSystemDesigner('bode',TfLQR); 

%% Discrete Control Codes written in MATLAB
T = 0.01;  %Sampling Time
 
d_sys = c2d(sys_cl,T,'zoh'); % Converts continuous-time dynamic system to discrete time
% impulse(d_sys);
% step(d_sys)
 
% Discrete-Time State Space Model which is obtained by "c2d" command
%T=1
% a =[   0.8184    -0.01021  -0.0002868;0.9071      0.9948  -0.0001482;0.4687      0.9982      0.9999];
% b = [ 0.9071;0.4687;0.1588];
% c = [0  -0.0883  -0.0044];
% d=0;
 
%T=0.1

% a =[0.9807     -0.0011  0;0.09903 0.9999  0;0.004968 0.1   1];
% b = [0.09903;0.004968;0.0001659];
% c = [0  -0.0883  -0.0044];
% d=0;
%T=0.01

a =[0.9981  -0.0001108  0;0.00999 1   -1.58e-08;4.997e-05        0.01           1];
b = [ 0.00999;4.997e-05;1.666e-07];
c = [0  -0.0883  -0.0044];
d=0;


Q=[1 0 0;0 1 0;0 0 1];
R=[1];
K=lqr(a,b,Q,R);    %   K = 11.5605   18.2677   10.0000
 
% sys_discrete= ss(a-b*K, b, c, d);
% step(sys_discrete) %Closed loop 
%  
 
TfLQRDiscrete=K*inv(s*eye(3)-a)*b; % Transfer Function is obtained
 
Kp=21.1;
Ki=26;
Kd=0;
 
H=1;
Gc=pid(Kp,Ki,Kd);
 
DscConPID=feedback(Gc*TfLQRDiscrete,H);
step(DscConPID);


