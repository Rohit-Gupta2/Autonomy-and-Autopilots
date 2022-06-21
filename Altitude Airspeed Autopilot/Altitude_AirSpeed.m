clear all; close all; clc
% Altitude/Air speed Hold Autopilot
% Goal is to hold a refrence altitude and airspeed like in  cruise condition
% Define the state-space representation of the aircraft dynamics.
% state: xlong = [U alpha Q teta H n]' i.e., [TAS att.ang. pitch_rate elev.angle altitude rpm]
% Define the state-space matrices A represents the aircraft dynamics, B represents control inputs, and C is the output matrix.
% Define the A matrix representing the aircraft dynamics for each state variable.
%state:  u[m/s]      alpha[rad]   q[rad/s]   teta[rad]  h[m]    n[rpm]
A=[     -0.0447      5.6993       0         -9.8056     0       0.0004     %----> u dynamics
        -0.0076     -1.4866       0.9399     0          0       0          %----> alpha dynamics
         0.0293     -12.1418     -7.9809     0          0       0          %----> q dynamics
              0      0            1          0          0       0          %----> teta dynamics
              0     -50           0          50         0       0          %----> h dynamics
              0      0            0          0          0      -0.4462]    %----> n dynamics
 
% remember h_dot=U0*teta-w          
%ulong=[deltae throttle] i.e. [deflection.elevator   percentage_throttle]
%state:  de        dtc
% B represents control inputs (deltae, throttle) for each state variable.
B=[      0         1.7371        %----> u dynamics
        -0.3067   -0.0025        %----> alpha dynamics
        -40.6088  -0.2448        %----> q dynamics
         0         0             %----> teta dynamics
         0         0             %----> h dynamics
         0         706.5875]     %----> n dynamics
% Define the output matrix C as an identity matrix and D as a zero matrix.     
C = eye(6);
D = zeros(6, 2);
% Convert the state-space representation to transfer function representation.
[num, den] = ss2tf(A, B, C, D, 1);
lon_tf = tf(1, den);
%% CHECK STABILITY OF OPEN LOOP SYSTEM (without SAS)
% Calculate eigenvalues of matrix A to check stability.
% Eigenvalues of matrix A => gives poles of transfer function and tells about stability
[eig_vec, eigval] = eig(A);
eig_val = diag(eigval);
% Calculate eigenvalues of matrix A to check stability.
% Eigenvalues of matrix A give the poles of the transfer function, indicating stability.
% Check if all real parts of eigenvalues are negative (asymptotically stable).
% Also, display damping coefficients, modes, and poles.
% we see from poles that all real part are negative and on left side of plane
% Hence open loop system is asymptotically stable
damp(lon_tf)         % Display poles, modes, damping coefficients,etc
pzmap(lon_tf)        % Plot the poles

% Calculate dynamic accuracy parameters for open-loop system.
% CHECK THE DYNAMIC ACCURACY OF OPEN LOOP SYSTEM
en  = eig_val(1);  % Energy mode pole
sp1 = eig_val(2);  % Short period pole 1
sp2 = eig_val(3);  % Short period pole 2
p1  = eig_val(4);  % Phugoid mode pole 1
p2  = eig_val(5);  % Phugoid mode pole 2
n   = eig_val(6);  % Engine mode pole

% check maximum overshoot and settling time for each mode.

%-----------------------------Short period Mode----------------------------
wn_sp  =  abs(sp1);                                  % Natural fequency
xi_sp  =  -(real(sp1)/wn_sp);                        % Damping coefficient
OS_max_sp = 100*exp(-pi*xi_sp/(sqrt(1-xi_sp^2)));  % Max percent overshoot
ts_sp  =  3/(wn_sp*xi_sp);                           % Settling time   

%--------------------------Phugoid Parameters------------------------------
wn_p  =  abs(p1);                                    % Natural fequency
xi_p  =  -(real(p1)/wn_p);                           % Damping coff
OS_max_p = 100*exp(-pi*xi_p/(sqrt(1-xi_p^2)));     % Max percent overshoot
ts_p  =  3/(wn_p*xi_p);                              % Settling time                                           
%-------------------------- Engine Parameters------------------------------
wn_en  =  abs(en);                                   % Natural fequency
xi_en  =  -(real(en)/wn_en);                         % Damping coff
OS_max_en = 100*exp(-pi*xi_en/(sqrt(1-xi_en^2)));  % Max percent overshoot
ts_en  =  3/(wn_en*xi_en);                           % Settling time                                                   
%---------------------------Energy Parameters -----------------------------
wn_e  =  abs(en);                                    % Natural fequency
xi_e  =  -(real(en)/wn_e);                           % Damping coff
OS_max_e = 100*exp(-pi*xi_e/(sqrt(1-xi_e^2)));     % Max percent overshoot
ts_e  =  3/(wn_e*xi_e);                              % Settling time      

% Create a table to display the parameters for each mode.
modes = {'Short Period'; 'Phugoid'; 'Engine Mode'; 'Energy Mode'};
natural_freq = [wn_sp; wn_p; wn_en; wn_e];
damping_coef = [xi_sp; xi_p; xi_en; xi_e];
max_overshoot = [OS_max_sp; OS_max_p; OS_max_en; OS_max_e];
settling_time = [ts_sp; ts_p; ts_en; ts_e];
Ta = table(natural_freq, damping_coef, max_overshoot, settling_time, 'RowNames', modes);
disp(Ta)

%% Check controllability and observability of open loop system
% For this we first do modal analysis of the system
% we'll create the matrix T in order to make the change of coordinates 
% then we'll compute A_bar and B_bar 
% Create matrix T for modal analysis.
% We'll analyze the controllability and observability of the system.
% Compute A_bar and B_bar.

%        en            n(engine)      sp                  sp                  ph                  ph      
T = [eig_vec(:, 1) eig_vec(:, 6) real(eig_vec(:, 2)) imag(eig_vec(:, 2)) real(eig_vec(:, 4)) imag(eig_vec(:, 4))];
A_bar = inv(T) * A * T;
B_bar = inv(T) * B;
C_bar = C * T;
D_bar = D;


% Calculate observability for Short Period and Phugoid modes.
Obs_sp = abs(C_bar(:, 3)) + abs(C_bar(:, 4));
Obs_ph = abs(C_bar(:, 5)) + abs(C_bar(:, 6));

% Display observability for each state variable.
states = char('u[m/s]  ', 'alpha[rad]', 'q[rad/s]', 'teta[rad]', 'h[m]', 'n[rpm]');
obs_sp = num2str(Obs_sp);
Observability_sp = [states, obs_sp];
obs_ph = num2str(Obs_ph);
Observability_ph = [states, obs_ph];

% we see the short period effect in q and h; (alpha=less significant)
% we see the phugoid effect in u and h (most observable variables)
%------------Exiting Modes=> Disturbability -------------------------------------
% we see which modes are more observable after disturbance on a variable
% Determine which variables are more observable.
% Phugoid mode is more observable.
T_inv = inv(T);
T_inv_norm = [T_inv(:, 1) / norm(T_inv(:, 1)), T_inv(:, 2) / norm(T_inv(:, 2)), T_inv(:, 3) / norm(T_inv(:, 3)), ...
    T_inv(:, 4) / norm(T_inv(:, 4)), T_inv(:, 5) / norm(T_inv(:, 5)), T_inv(:, 6) / norm(T_inv(:, 6))];
l2 = char('en', 'n(eng)', 'sp', 'sp', 'ph', 'ph');
l1 = '               u       alpha           q        teta           h           n';
T_exiting = num2str(T_inv_norm);
T_exiting = [l2, T_exiting];
Exiting_Modes = char(l1, T_exiting);   % phugoid is more observable

%% Steady state control problem 

%consider LTI system: xdot(t)=Ax(t)+Bu(t) ; y(t)=Cx(t)+Du(t)
% the set point of this system is (xs,us,ys) at equilibrium where xdot(t)=0
% To design a servomechanism: automatic device used to correct the performance of a mechanism by means of an error-sensing feedback.
% we consider case where r=m (no. of input= no. of output)
% OPEN LOOP SOLUTION
% u_s = input of pilot; y_s= desired equilibrium output
% system is: 0= A*x_s + B*u_s ; y_s = C*x_s + D*u_s
% We calculate the feedforward gain for open loop N_uy_bar
% solve the following problem
% Steady State Control Problem
% Consider an LTI system: xdot(t) = Ax(t) + Bu(t), y(t) = Cx(t) + Du(t)
% Calculate the feedforward gain for open-loop system N_uy_bar.
% Define matrices C and D for the performance index calculation.
C = [1 0 0 0 0 0;
    0 0 0 0 1 0];
save C;
D = [0 0;
    0 0];
% Solve for the feedforward gain N_bar.
N_bar = ([A B; C D]^-1) * [zeros(6, 2); eye(2)];
N_xy_bar = [N_bar(1, :); N_bar(2, :); N_bar(3, :); N_bar(4, :); N_bar(5, :); N_bar(6, :)];
N_uy_bar = [N_bar(7, :); N_bar(8, :)];
% Set desired values for velocity and altitude.
u_s = 0;  % Velocity in m/s
h_s = 5;  % Desired altitude in meters
y_s = [u_s; h_s];
% Calculate desired control input U_s and desired state x_s.
U_s = N_uy_bar * y_s;  % Desired Control
x_s = N_xy_bar * y_s;  % Desired State
save N_uy_bar  ;   %the inputs are not acting directly on h

%% CLOSED LOOP SYSTEM with SAS (to get optimal feedback gain K)
% as we are introducing feedback the problem changes to optimal control
% we introduce a performance index J with weighing matrices Q and R
% If there are some disturbances on (A,B), we may obtain a tracking error
% e(t), where obtained state is not same as desired
% so to avoid this error we may use proportonal intergral (PI) controller,
% that applies correction to a control function.
% so new system includes integral of output(past values) y(t)
% xa_dot= F*xa + G*u ; x_a=[u, alpha, q, teta, h, n, integrated(H), integrated(L)]
% y(t)= H*xa + L*u ; yf= integrate(y(t)) over 0 to t
F=[A  zeros(6,2)
   C  zeros(2,2)];
 
G=[B
   zeros(2,2)];

H=[C          zeros(2,2)
   zeros(2,6) eye(2)];

L=zeros(4,2);
sys_PI=ss(F,G,H,L);

%---------------------- Controllability & observability -------------------
P=ctrb(F,G)  %controllability
d=length(F)
if rank(P)==d
    disp('"F" and "G" of the PI model is completely controllable')
else
    disp('the system is not completely controllable')
end

O=obsv(F,H)  % observability
d=length(F)
if rank(O)==d
     disp('"F" and "H" of the PI model is completely observable')
 else
     disp('the system is not completely observable,modify matrix C')
 end


%---------------------- R Matrix ------------------------------------------
dt_max=0.035;   % maximum throttle input = 3.5 ft/s
de_max=0.1;     % maximum elevator deflection = 10 rad
%r=(1/(dt_max^2))/(1/(de_max^2));
r=9;
R=[1 0
   0 r];
%---------------------- Interative procedure ------------------------------
%  Parametrization of weigthen matrix Qy= [ kq 0; 0 kq]
q=1/16;
qf=q;
i=1;
figure
for k=0.001:0.01:3;
    
    Qy=[k,0;0,k*q];     % Original Matrix from the system without PI
    Qf=[k,0;0,qf*k];    % Matrix coming from the PI augmented system
    
    Qa=[Qy,zeros(2,2);zeros(2,2),Qf];  % new matrix Qa for the augmented state
    
    % I need to solve the ARE ==> S A + (A^t) S - S B(R^-1)(B^t) S + (C^t) Qy C = 0 where S>=0
    [K,S,eigVAL(:,i)]=lqry(sys_PI,Qa,R);   % S --------------> solution of ARE for k=k(i)should be S>=0
                                      % K_opt ----------> optimal feedback gain Function of k
                                      % eigval(:,i) ----> eigenvalue of closed loop system A+BK_opt(k)
     title('EigVAL Locus')
    xlabel('Re')
    ylabel('Im')
    grid on
    hold on
    plot(eigVAL(:,i),'k*')
    
    i=i+1;
end
% Fixing a value of the parameter k = 1

k_opt=1;
Qy_opt=[k_opt,0;0,k_opt*q];     % Original Matrix from the system without PI
Qf_opt=[k_opt,0;0,qf*k_opt];    % Matrix coming from the PI augmented system
Qa_opt=[Qy_opt,zeros(2,2);zeros(2,2),Qf_opt];  % new matrix Qa for the augmented state
[K_opt,S_opt,eigVAL_opt]=lqry(sys_PI,Qa_opt,R);     % Final Solution to ARE
disp('Optimal Gain Altitude/Airspeed SAS Piper using a PI')
K_opt;

K=K_opt(:,1:6);   % Optimal Gain from the system without PI
Kf=K_opt(:,7:8);  % Optimal Gain coming from the PI augmented system
save K
save Kf


%---------------Flying qualities parameters to fullfiled------------------%
% -Aircraft Class ====> Class II 
% -Mission Fligth ====> Category B 
% -Level of flying quality ====> Level 1
% 1 - Phugoid mode Specifications -----------
xi_p_min=0.04;  % Damping Coeficient

% 2- Short Period mode specifications -------
xi_sp_rg=[0.30,2]; % Damping Coeficient

% 3 - Natural frequency (Short Period) wn -------------------
U0= 50;
Z_w=-1.4866;
M_ws=-12.1418;
M_de=-40.6088;
Z_de=-0.3067;
g=9.81;
T_teta2=-Z_w+(M_ws/M_de)*Z_de;     % 1 / T_teta2
n_aoa=T_teta2*U0/g;
wn_sp_rg=[0.085*n_aoa 3.6*n_aoa];  % Natural frequency wn

% --CHECK OF FLYING REQUIREMENTS -----------------------------------------
%----------------------------Short Period Parameters-----------------------
wn_sp_opt=abs(eigVAL_opt(1));                                % natural fequency
xi_sp_opt=-(real(eigVAL_opt(1))/wn_sp_opt);                  % damping coff
OS_max_sp_opt=100*exp(-pi*xi_sp_opt/(sqrt(1-xi_sp_opt^2)));  % max percent overshoot
ts_sp_opt=3/(wn_sp_opt*xi_sp_opt);                           % settling time                                         
%-------------------------- Phugoid Parameters ----------------------------
wn_p_opt=abs(eigVAL_opt(5));                                 % natural fequency
xi_p_opt=-(real(eigVAL_opt(5))/wn_p_opt);                    % damping coff
OS_max_p_opt=100*exp(-pi*xi_p_opt/(sqrt(1-xi_p_opt^2)));     % max percent overshoot
ts_p_opt=3/(wn_p_opt*xi_p_opt);                              % settling time                                         
%--------------------------- Engine Parameters ----------------------------
wn_en_opt=abs(eigVAL_opt(7));                                % natural fequency
xi_en_opt=-(real(eigVAL_opt(7))/wn_en_opt);                  % damping coff
OS_max_en_opt=100*exp(-pi*xi_en_opt/(sqrt(1-xi_en_opt^2)));  % max percent overshoot
ts_en_opt=3/(wn_en_opt*xi_en_opt);                           % settling time                                            
%---------------------- Energy Parameters ---------------------------------
wn_e_opt=abs(eigVAL_opt(8));                                 % natural fequency
xi_e_opt=-(real(eigVAL_opt(8))/wn_e_opt);                    % damping coff
OS_max_e_opt=100*exp(-pi*xi_e_opt/(sqrt(1-xi_e_opt^2)));     % max percent overshoot
ts_e_opt=3/(wn_e_opt*xi_e_opt);                              % settling time                                             

%----------------- Table with Flying Qualities Comparision ----------------
parameters={'Nat Freq sp';'Damping  sp';'Nat Freq  p';'Damping   p';'Max overshoot sp';'Max overshoot p';'settling time sp';'settling time p'};
Original_System=[wn_sp;xi_sp;wn_p;xi_p;OS_max_sp;OS_max_p;ts_sp;ts_p];
Feedback_System=[wn_sp_opt;xi_sp_opt;wn_p_opt;xi_p_opt;OS_max_sp_opt;OS_max_p_opt;ts_sp_opt;ts_p_opt];
min_Flying_Qualities=[string(wn_sp_rg(1));string(xi_sp_rg(1));'-';xi_p_min;NaN;NaN;NaN;NaN];
max_Flying_Qualities=[string(wn_sp_rg(2));string(xi_sp_rg(2));'-';'-';NaN;NaN;NaN;NaN];
Tb=table(Original_System,Feedback_System,min_Flying_Qualities,max_Flying_Qualities,'RowNames',parameters);
disp(Tb)

% the phugoid has 143 sec settling time and is highly oscillatory in original system 
% the closed-loop response is much faster, almost 3 seconds, and it is also much less oscillatory
[eigVEC,eigVAL]=eig(F);
eigVAL=diag(eigVAL); 
eigVAL_opt;
hold on
plot(eigVAL,'o','MarkerFaceColor','g')
hold  on
plot(eigVAL_opt,'d','MarkerFaceColor','r')

%% response improving at worst condition
[vec, val] = eig(S);

% the worst initial condition has basically only component in u
[J,l]=max(diag(val));
x0=vec(:,l)/norm(vec(:,l));
J= (x0')*S*x0

% with feedback

J_opt=(x0')*S_opt*x0

% we see that the performance index has reduced from 108.5 to 69.39 hence optimal

%% Altitude/ Airspeed Hold autopilot
% calculate feedforward gain N_uy

N=([A+B*K B;C+D*K D]^-1)*[zeros(6,2);eye(2)];
N_xy=[N(1,:);N(2,:);N(3,:);N(4,:);N(5,:);N(6,:)];
N_uy=[N(7,:);N(8,:)];
u_s=0;            %velocity m/s 
h_s=5;            %setting desired attitude
y_s=[u_s;h_s];
p_s= N_uy*y_s;
U_ss=p_s-(K*N_xy*y_s)      %desired Control

save N_uy  ;   %the inputs are not acting directly on h

xdot = A*x_s + B*p_s  % xdot vector or state is almost null(means equilibrium)

%% observer
% allows to estimate the state variables that are unaccessible from output measurement
% assume that refernce input is 0, therefore u=-Kx
% state-space equations for the closed-loop feedback system are: xdot=(A-BK)x
% stability and time-domain performance of the closed-loop feedback system are determined primarily by 
%the location of the eigenvalues of the matrix (A-BK),which are equal to the closed-loop poles

% --------------------- IDENTITY OBSERVER ------------------------------%%
% we can place these closed-loop poles anywhere we like (because the system is controllable)
% we now decide the position (poles) where we want to place closed loop poles
% Sice we want the dynamics of the observer to be much nfaster than the system itself,
% we place the poles at least five times farther to the left than the dominant poles of the system
% Design an observer to estimate unobservable state variables from output measurements.

% Define observer poles to place them five times farther to the left than the dominant poles of the system.
eigVAL_obs = eig(A - B * K);  % Eigenvalues of the closed-loop system without observer
e_r = max(abs(eigVAL_obs));  % Fastest eigenvalue
p_slow = -1 * e_r;  % Slowest observer pole
p2 = 1.001 * p_slow;
p3 = 1.012 * p_slow;
p4 = 1.013 * p_slow;
p5 = 1.014 * p_slow;
p6 = 1.015 * p_slow;

% Define the observer poles.
poles_observer = [p_slow, p2, p3, p4, p5, p6];

% Calculate the observer gain K_observer to place the observer poles.
% we have to find observer gain to control arbitary dynamics of error.
K_observer = place(A', C', poles_observer)';

% Check if the observer is asymptotically stable with the chosen gain.
% we see that the eigenvalues are negative with choice of K_obs 
 % hence the error e(t) vanishes asymptotically 
error_dynamics = eig(A - K_observer * C);  % Eigenvalues of the observer error dynamics

% Display the observer eigenvalues.
disp('Observer Eigenvalues:');
disp(error_dynamics);

%% OVERALL SYSTEM
% In general all state variables are not directly measurable.
% we use identity observer to estimate state and use this info in regulator feedback
An=[(A-B*K)  (B*K); zeros(6) (A-(K_observer)*C)];  % overall system A matrix
eigval_overall = eig(An)   % we see all the real parts are negative, hence asymp. stable

%PLOT OBSERVER POLES
figure 
title('Observer poles')
xlabel('Re')
ylabel('Im')
hold on;
plot(eigVAL_opt,'o','MarkerFaceColor','r')
hold on;
plot(poles_observer,0,'o','MarkerFaceColor','g')
legend('System eigenvalues','Observer eigenvalues')
grid on
hold off
%% Wind model

% dot(x)=A*x+B*u+Elong*wlong
%dot(x)=A*x+B_wind*u_wind       ; B_wind=[B Elong}; u_wind=[u; wlong]
% wlong=[gust_x  gust_z]'
%wind covariance Pwind=0.7

%wind correlation time constant
%tau_u=2.326;       longitudinal
%tau_w=0.9434;      vertical 
Elong =[
        -0.07224216819218   0.21477104529793
        -0.00954723493499  -0.03061702460982
         0.02878416744368  -0.24103859236200
                        0                  0
         0.07219231056339  -0.99739073100542
                        0                  0]
B_wind=[B Elong];
save Elong
save B_wind
