% Reinitialise workspace
clear referenceTrajectory;
clear inputMatrix
close all;
clc;


% Construct the set of necessary coefficients needed for the simulation
m = 1.4; % Mass of the quadcopter (kg)
g = 9.81; % Gravitational Acceleration (m/s^2)
I_xx = 0.08; % Moment of Inertia in the X direction (kgm^2)
I_yy = 0.08; % Moment of Inertia in the Y direction (kgm^2)
I_zz = 0.16; % Moment of Inertia in the Z direction (kgm^2)
l = 223.5e-3; % Length of the Arm (m)
K_t = [0.1 0.1 0.1]; % Coefficients of Drag Forces
K_r = [0.1 0.1 0.1]; % Coefficients of Drag Forces
n_stateDimension = 12; % We define the dimension of the system to be 12
t_simulationTime = 9 ; % Set a reasonable simulation time measured in seconds
numSamples = 1000 ;
x_test = referenceTrajectory(3); % For debugging

% Construct statespace matrices that implement linearised quadrotor
% dynamics
A_diagVector = [0 (-K_t(3)/m) 0 (-K_r(3)/I_zz) 0 (-K_t(1)/m) 0 (-K_r(1)/I_xx) 0 (-K_t(2)/m) 0 (-K_r(2)/I_yy)];
A_diag = diag(A_diagVector); % Diagonalise A according to A_diagVector
A_offDiagVector = [1 0 1 0 1 0 1 0 1 0 1];
A_offDiag = diag(A_offDiagVector, 1); % Set it one left of the main diag
A = A_diag + A_offDiag;
A(10,7) = g;
A(6,11) = g;

B = zeros(n_stateDimension, 4);
B(2,1) = 1/m;
B(8,2) = 1/I_xx;
B(12,3) = 1/I_yy;
B(4,4) = 1 / I_zz;

% Use lqr to find optimal K and E (the system poles are to validate
% stability)

    % Define the Q matrix (12x12) with specified weights
Q = 1*diag([20000 1 1 1 10 1 1 1 10 1 1 1]);
    % Define the R matrix (4x4) with specified weights
R = diag([0.05 1 1 1]);

% Calculate the optimal feedback gain K and State Space Poles for this
% configuration
[K, unused, E] = lqr(A, B, Q, R);

% Use K to create the State space using u = -K(x - x_ref)
C = eye(n_stateDimension); % NOTE - in this case y = x (the states are the outputs)
D = 0; % There is no influence of the inputs on the outputs

syso = ss(A,B,C,D); % Create the open-loop state space model

% Define a function handle for your control law
control_law = @(t, x) - (K * (x - referenceTrajectory(t) ));

% Take the impulse response of the created state space simulated over
% t_simulationTime seconds
T = linspace(0, t_simulationTime, numSamples);
T_span = [0, t_simulationTime];

% Define initial condition
x0 = zeros(n_stateDimension,1); % Your initial condition

% Simulate the system using the control law
wx = zeros(n_stateDimension,1);
w = syso.A*wx + syso.B*control_law(3, wx) ;
[t, x] = ode45(@(t, x) syso.A*x + syso.B*control_law(t, x), T_span, x0);



% Extract to new variables the x, y and z co-ordinates from the resultant
% state vector
% Note x := [z z_dot psi psi_dot x x_dot phi phi_dot y y_dot theta theta_dot]^T
sys_x = x(:,5,1); % For this excercise we plot the outputs w.r.t input U1
sys_y = x(:,9,1);
sys_z = x(:,1,1);

% set u to simply be (x_ref)
u_func = @(t) (referenceTrajectory(t));

% Extract this u vector into three variables indicating x,y,z data
u_x = zeros(length(t),1); u_y = zeros(length(t),1); u_z = zeros(length(t),1);
for i = 1:length(t)
    u = u_func(t(i));
    u_x(i) = u(5);
    u_y(i) = u(9);
    u_z(i) = u(1);
end

% Plot the evolution of x, y and z in a 3D plot
figure1 = figure;

    % Create axes
axes1 = axes('Parent',figure1);

plot3(sys_x, sys_y, sys_z,'DisplayName','$ \dot{X} = (A-BK)X + BKx_{ref}$',...
    'LineWidth',1.5)
%title(sprintf('Impulse Response for $Q = %.d * Q_{research}$  and $R = %.f$ ', Q_scalar,R),'Interpreter','latex')
grid on
set(axes1,'XMinorGrid','on','YMinorGrid','on','ZMinorGrid','on');
hold on
% Construct a reference plot (simulated over t_simulationTime seconds) and
% superimpose it over the system response plot
plot3(u_x, u_y, u_z,'DisplayName','$x_{ref}$')

% Create title
title('LQR-controlled System Dynamics of Quadrotor with respect to reference trajectory',...
    'FontSize',18,...
    'Interpreter','latex');

% Create legend
legend1 = legend(axes1,'show');
set(legend1,...
    'Position',[0.661277612782192 0.78517901927536 0.184402052130599 0.0472622823048963],...
    'Interpreter','latex',...
    'FontSize',12);

hold off


