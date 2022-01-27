% Written by: Hrushikesh Budhale

%% Define necessary Symbols
syms F g M m1 m2 L1 L2 x t1 t2 xd t1d t2d;

%% Lagrangian equations
xdd = (F - t1^2*m1*L1*sin(t1) - m1*g*sin(t1)*cos(t1) - t2^2*m2*L2*sin(t2) - m2*g*sin(t2)*cos(t2)) / (M+m1+m2 - m1*cos(t1)^2 - m2*cos(t2)^2);
t1dd = (xdd*cos(t1) - g*sin(t1))/L1;
t2dd = (xdd*cos(t2) - g*sin(t2))/L2;

%% Linearizing system
input_vars = F;
state_vars = [x xd t1 t1d t2 t2d];
state_vals = [0 0 0 0 0 0];

A = jacobian([xd xdd t1d t1dd t2d t2dd], state_vars);
B = jacobian([xd xdd t1d t1dd t2d t2dd], input_vars);
C = jacobian([x t1 t2], state_vars);
D = jacobian([x t1 t2], input_vars);

As = subs(A, state_vars, state_vals);
Bs = subs(B, state_vars, state_vals);
Cs = subs(C, state_vars, state_vals);
Ds = subs(D, state_vars, state_vals);


%% Stability analysis
consts = [g M m1 m2 L1 L2];
c_vals = [9.8 1000 100 100 20 10];
len1 = subs(L1,consts, c_vals);
len2 = subs(L2,consts, c_vals);

ctrb_mat = [Bs As*Bs As^2*Bs As^3*Bs As^4*Bs As^5*Bs];
r = rank(subs(ctrb_mat, consts, c_vals));
if r == length(state_vars)
    disp("System is controllable")
end
% rank is 6 == number of state variables hence system is controllable

%% %% Closing loop and simulating (without tuning gains)
A = double(subs(As, consts, c_vals));
B = double(subs(Bs, consts, c_vals));
C = double(subs(Cs, consts, c_vals));
D = double(subs(Ds, consts, c_vals));

% before tuning
Q = eye(6,6);
R = 1;
L = lqr(A, B, Q, R);

tspan = 0:0.5:30;
X0 = [0; 0; deg2rad(10); 0; deg2rad(-10); 0];   % initial state
XT = [0; 0; 0; 0; 0; 0];                        % terminal state
u = @(X) -L*(X - XT);                           % control law
[t,X] = ode45(@(t,X) A*X + B*u(X), tspan, X0);

%% Show animation
animate_scene(t,X,len1,len2,"Open_loop",0);

%% Setting gains for LQR (Linear Quadratic Regulator)

Q = eye(6,6);
Q(3,3) = 1000;      % penalize t1
Q(4,4) = 1000;      % penalize t1d
Q(5,5) = 1000;      % penalize t2
Q(6,6) = 1000;      % penalize t2d
R = 0.00001;

L = lqr(A, B, Q, R);

%% Closing the loop and simulating (after tuning gains)

tspan = 0:0.5:30;
X0 = [0; 0; deg2rad(10); 0; deg2rad(-10); 0];    % initial state
XT = [10; 0; 0; 0; 0; 0];                        % terminal state
u = @(X) -L*(X - XT);                           % control law
[t,X] = ode45(@(t,X) A*X + B*u(X), tspan, X0);

%% Show animation
animate_scene(t,X,len1,len2,"LQR_reference_tracking",0);

%% Plot results
figure('Name','Closed Loop LQR','NumberTitle','off');
subplot(4,1,1);
plot(t,X(:,1));
title('Cart position (m)');
subplot(4,1,2);
plot(t,X(:,3));
title('Theta1 (raddian)');
subplot(4,1,3);
plot(t,X(:,5));
title('Theta2 (raddian)');

in = -repmat(L,length(X),1).*(X-repmat(XT',length(X),1));
subplot(4,1,4);
plot(t,in(:,4));
title('Input force (N)');

%% Observability for 4 cases

% case 1: Y = [x]
Cs = jacobian(x, state_vars);
obsr_mat = [Cs; Cs*As; Cs*As^2; Cs*As^3; Cs*As^4; Cs*As^5];
rank(subs(obsr_mat, consts, c_vals));
C1 = double(subs(Cs, consts, c_vals));
% rank is 6 == number of state_variable => system is observable

% case 2: Y = [t1; t2]
Cs = jacobian([t1 t2], state_vars);
obsr_mat = [Cs; Cs*As; Cs*As^2; Cs*As^3; Cs*As^4; Cs*As^5];
rank(subs(obsr_mat, consts, c_vals));
C2 = double(subs(Cs, consts, c_vals));
% rank is 4 ~= number of state_variable => system is not observable

% case 3: Y = [x; t2]
Cs = jacobian([x t2], state_vars);
obsr_mat = [Cs; Cs*As; Cs*As^2; Cs*As^3; Cs*As^4; Cs*As^5];
rank(subs(obsr_mat, consts, c_vals));
C3 = double(subs(Cs, consts, c_vals));
% rank is 6 == number of state_variable => system is observable

% case 4: Y = [x; t1; t2]
Cs = jacobian([x t1 t2], state_vars);
obsr_mat = [Cs; Cs*As; Cs*As^2; Cs*As^3; Cs*As^4; Cs*As^5];
rank(subs(obsr_mat, consts, c_vals));
C4 = double(subs(Cs, consts, c_vals));
% rank is 6 == number of state_variable => system is observable

%% Luenberger state observer

% Let's assume we have acces to only x, t1 & t2 i.e. C4
% Now Luenberger observer gain L can be found by using, eigen values of
% (A-LC) or using place() function in matlab
poles = [-2 -3 -4 -5 -6 -7];
L = place(A', C4', poles)';

tspan = 0:0.02:3;
X0 = [0; 0; deg2rad(10); 0; deg2rad(-10); 0];   % initial state
X0h =[5; 0; deg2rad(-8); 0; deg2rad(5); 0];     % initial estimate

% creating combined ss equation of form
% |X | = |A    0 | |X |
% |Xh| = |LC A-LC| |Xh|

X0c = [X0; X0h];
% Combined matrix
Ac = [A, zeros(6,6); L*C4, A-L*C4];
[t,X] = ode45(@(t,X) Ac*X, tspan, X0c);
Xh = X(:,7:12);
X = X(:,1:6);

%% Show animation
animate_LQG(t,X,0,Xh,len1,len2,"LuenbergerStateEstimation(slow_mo)",0);

%% Plot results
plot_graph_LQG(t, X,Xh,0);

%% Luenberger observer with LQR control for Reference tracking

tspan = 0:0.5:30;
X0 = [0; 0; deg2rad(-10); 0; deg2rad(10); 0];   % initial state
X0h =[5; 0; deg2rad(8); 0; deg2rad(-5); 0];     % initial estimate
XT = [10; 0; 0; 0; 0; 0];                       % terminal state (desired)

% creating combined ss equation of form
% |X | = |A    0 | |X | + |B | |-K(X-XT) -K(X-XT)|
% |Xh| = |LC A-LC| |Xh| + |B |

% Combined matrices
X0c = [X0; X0h];
XTc = [XT; XT];
Kc = [Kf, Kf];
Bc = [B; B];
Ac = [A, zeros(6,6); L*C4, A-L*C4];
u = @(X) -Kc*(X - XTc);                          % control law
[t,X] = ode45(@(t,X) Ac*X + Bc*u(X), tspan, X0c);
Xh = X(:,7:12);
X = X(:,1:6);

%% Show animation
animate_LQG(t,X,0,Xh,len1,len2,"LQG_reference_tracking",0);

%% Plot results
plot_graph_LQG(t, X,Xh,0);


%% Kalman filter
% Define Noise varience
Vd = 0.001*eye(6);    % Process noise (disturbance) covarience
Vm = 0.1*eye(3);      % measurement noise covarience
% Vm(1,1) = 0.5;

% Construct input
tspan = 0:0.1:25;
u = 0*tspan;
u(5:100) = 5;
u(120:160) = -11;
uDIST = randn(6,size(tspan,2));
uNOISE = randn(3,size(tspan,2))*0.5;
uNOISE(1,:) = uNOISE(1,:)*2;
uAUG = [u; Vd*Vd*uDIST; uNOISE];

% augment input with disturbance and noise
% |X| = |A| |X| + |B I 0| |u |
%                         |Vd|
% |y| = |C| |X| + |0 0 I| |Vm|
% System with noise
BF = [B Vd zeros(6,3)];
sysC = ss(A,BF,C4,[zeros(3,1), zeros(3,6), Vm]);
% Actual full state for comparison
sysFullOutput = ss(A,BF,eye(6),zeros(6,size(BF,2)));

% Build Kalman filter
% [Kf,P,E] = lqe(A,Vd,C4,Vd,Vm);
Kf = (lqr(A',C4',Vd,Vm))';
sysKF = ss(A-Kf*C4,[B Kf],eye(6), 0*[B Kf]);

X0 = [1; 0; deg2rad(-10); 0; deg2rad(10); 0];   % initial state
X0h =[1.2; 0; deg2rad(-6); 0; deg2rad(7); 0];     % initial estimate

[xtrue,~] = lsim(sysFullOutput,uAUG,tspan,X0);  % true system
[y,~] = lsim(sysC,uAUG',tspan,X0);              % system with noise
[xest,t] = lsim(sysKF,[u; y'],tspan,X0h);       % Kalman filter

%% Show animation
% modyfy output for plotting
animate_LQG(t,xtrue,y,xest,len1,len2,"Kalman filtering on noisy output",1);

%% Plot graphs
plot_graph_LQG(t,xtrue,xest,y)
