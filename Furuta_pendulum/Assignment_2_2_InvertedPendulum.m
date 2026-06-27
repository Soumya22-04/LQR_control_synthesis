%% ----------------------------------------------------------------------------
% -----------------------------------------------------------------------------
%  Name: Soumya Pramanik
%  Roll no. : 24EC10052
%  Part 2 of Assignment 2: Designing a controller for an inverted pendullum
%  system
% ----------------------------------------------------------------------------
% ----------------------------------------------------------------------------
%%
clc;
close all;
clear;

% Masses and dimensions
m = [0.5 0.04];  % mass of moving rods
l = [0.1 0.08];    % length of moving rods
J = 6e-3;     % MOI of the axis
b = [5e-6, 3e-6];  %Drag coefficients due to joints
g = 9.81;
h = 0.15;  %Height of the axis

%Coefficients for calculation
alpha = zeros(1,4);
alpha(1) = (m(1)*l(1)^2)/3;
alpha(2) = m(2)*l(1)^2;
alpha(3) = m(2)*l(2)^2/3;
alpha(4) = m(2)*l(1)*l(2)/2;
d = alpha(3)*(alpha(1)+alpha(2))-alpha(4)^2;


%%
%Linearised system for phi = pi
A_pi = [[0 1 0 0]
        [0 -alpha(3)*b(1)/d -alpha(4)*m(2)*l(2)*g/(2*d) alpha(4)*b(2)/d]
        [0 0 0 1]
        [0 -alpha(4)*b(1)/d (alpha(1)+alpha(2))*m(2)*l(2)*g/(2*d) (alpha(1)+alpha(2))*b(2)/(2*d)]];
B_pi = [0;alpha(3)/d;0;alpha(4)/d];
C_pi = [1 0 0 0;0 0 1 0];

%Controllability and observability tests
C_script = [B_pi A_pi*B_pi A_pi^2*B_pi A_pi^3*B_pi];
O_script = [C_pi;C_pi*A_pi;C_pi*A_pi^2;C_pi*A_pi^3];

if rank(C_script) == size(A_pi,2)
    disp("The system is controllable")
else
    disp("The system is not controllable");
end

if rank(O_script) == size(A_pi,2)
    disp("The system is observable");
else
    disp("The system is not observable");
end

% I haven't derived the co subspace separately as the system is already
% controllable and observable

%% Luenberger observer
Q = null(C_pi,1e-4)';
P_observer = [C_pi;Q];
A_observer = P_observer*A_pi/P_observer;
B_observer = P_observer*B_pi;
C_observer = C_pi/P_observer;
A11 = A_observer(1:size(C_pi,1),1:size(C_pi,1));
A21 = A_observer(size(C_pi,1)+1:end,1:size(C_pi,1));
A12 = A_observer(1:size(C_pi,1),size(C_pi,1)+1:end);
A22 = A_observer(size(C_pi,1)+1:end,size(C_pi,1)+1:end);

B1 = B_observer(1:size(C_pi,1),:);
B2 = B_observer(size(C_pi,1)+1:end,:);

mu = 100;
W = lyap((-mu*eye(size(A22))-A22)', A12'*A12);
L = 0.5*A12/W;

%Simulation of observer dynamics
function E = simobs(t,X,A11,A12,A21,A22,L,A_pi,C_pi)
    x = X(1:4);
    dx_dt = A_pi*X(1:4);
    z = X(5:end);
    y = C_pi*x;
    %z = x_hat - L*y
    dz_dt = (A21-L*A11)*y+(A22-L*A12)*L*y + (A22 - L*A12)*z;
    E = [dx_dt;dz_dt];
    % E = (A22-L*A12)*X;
end

terr = [0 1];
X_err0 = 0.01*rand(1,6);
[terr, X_obs] = ode15s(@(t,X) simobs(t,X,A11,A12,A21,A22,L,A_pi,C_pi),terr,X_err0);
y = C_pi*X_obs(:,1:4)';
x_hat = X_obs(:,5:6)' + L*y;
X_lo = P_observer\[y;x_hat];
err = X_lo - X_obs(:,1:4)';


%Plotting error estimates
figure;
plot(terr,err);
xlabel("Time");
ylabel("Error");
title("Error of observer derived state variables near phi = pi");

%% LQR design
R_tilda = 0.2;
Q = diag([15 7 60 5]);

R = -(B_pi/R_tilda)*B_pi';

H = [A_pi R;-Q -A_pi'];
[eigvec_H,eig_H] = eig(H);
eigvec_neg = eigvec_H(:,real(diag(eig_H))<0);

X1 = eigvec_neg(1:length(eig_H)/2,:);
X2 = eigvec_neg(length(eig_H)/2+1:length(eig_H),:);

K = R_tilda\B_pi'*real(X2/X1);
eig(A_pi - B_pi*K)

disp("LQR Gain matrix: ");
disp(K);


%%
%Non linear model
function dA_dt = odefun(t, X,alpha,b,J,m,l,g,K)
    tau = -3;
    angle_error = wrapToPi(X(3) - pi);

    if abs(angle_error)<deg2rad(40)
        goal = [0;0;pi;0];
        tau = -K*(X - goal);
    else
        % Energy based swing-up
        I_p = 1/3*m(2)*l(2)^2;
        E =  0.5*I_p*X(4)^2 + m(2)*g*l(2)*(1-cos(X(3)));
        E_target = 2*m(2)*g*l(2);
        E_err = E - E_target;

        kE = 5;
        k_centre = 2;
        k_damp = 0.01;
        tau = kE*E_err*X(4)*cos(X(3)) - k_centre*X(1) - k_damp*X(2);
    end

    %Torque saturation
    tau = max(min(tau,8), -8);

dA_dt = zeros(4,1);
    dA_dt(1) = X(2);
    dA_dt(2) = (alpha(3)*(-b(1)*X(2)-alpha(3)*sin(2*X(3))*X(4)*X(2)-alpha(4)*sin(X(3))*(X(4))^2 + tau)+alpha(4)*cos(X(3))*(-b(2)*X(4)+alpha(3)*sin(2*X(3))*(X(2))^2/2-m(2)*l(2)*g*sin(X(3))/2))/(alpha(3)*(J + alpha(1)+alpha(2)+alpha(3)*sin(X(3))^2)-alpha(4)^2*cos(X(3))^2);
    dA_dt(3) = X(4);
    dA_dt(4) = ((J + alpha(1)+alpha(2)+alpha(3)*sin(X(3))^2)*(-b(2)*X(4)+alpha(3)*sin(2*X(3))*(X(2))^2/2-m(2)*l(2)*g*sin(X(3))/2)+alpha(4)*(tau-b(1)*X(2)-alpha(3)*sin(2*X(3))*X(4)*X(2)-alpha(4)*sin(X(3))*(X(4))^2))/(alpha(3)*(J + alpha(1)+alpha(2)+alpha(3)*sin(X(3))^2)-alpha(4)^2*cos(X(3))^2);
    
  
end

X0 = [0.2*ones(1,1)+randn(1,1) zeros(1,1) randn(1,1) -0.002*ones(1,1)+randn(1,1)];
tspan = [0 4];

[t, A] = ode45(@(t,y) odefun(t, y, alpha, b, J, m, l,g,K), tspan, X0);
    
%% Error plots
figure;
subplot(2,1,1);
plot(t,A(:,1));
xlabel("Time");
ylabel("\theta (rad)");
yline(0, "--b");
title("Error Plot (\theta)");
grid on;

subplot(2,1,2);
plot(t, A(:,3));
xlabel("Time");
ylabel("\phi (rad)");
yline(pi, "--b");
title("Error Plot (\phi)");
grid on;


%% Animation

v = VideoWriter('furuta_pendulum.mp4','MPEG-4');
v.FrameRate = 30;
open(v);


%Coordinates

theta = A(:,1);
phi   = A(:,3);

xp2 = l(1)*cos(theta) + l(2)*sin(phi).*sin(theta);
yp2 = l(1)*sin(theta) - l(2)*sin(phi).*cos(theta);
zp2 = h - l(2)*cos(phi);

xp1 = l(1)*cos(theta);
yp1 = l(1)*sin(theta);
zp1 = h*ones(size(phi));


% Figure setup

figure('Color','w');
grid on;
axis equal;
axis([-2 2 -2 2 0 2.5]);
view(45,30);
hold on;


h_axis = plot3([0 0], [0 0], [0 10*h], 'k', 'LineWidth',2);
h_arm1 = plot3([0 10*xp1(1)], [0 10*yp1(1)], [10*h 10*zp1(1)], 'b', 'LineWidth', 2.5);
h_arm2 = plot3([10*xp1(1) 10*xp2(1)], [10*yp1(1) 10*yp2(1)], [10*zp1(1) 10*zp2(1)], 'b', 'LineWidth', 2.5);


targetFPS = 30;
simDuration = t(end) - t(1);
nFrames = round(targetFPS * simDuration * 1.5);
idx = round(linspace(1,length(t),nFrames));

% Animation loop

for k = 1:length(idx)
    i = idx(k);


    % Update data
    set(h_axis, 'ZData',[0 10*zp1(i)]);
    
    set(h_arm1, ...
        'XData',[0 10*xp1(i)], ...
        'YData',[0 10*yp1(i)], ...
        'ZData',[10*h 10*zp1(i)]);
    
    set(h_arm2, ...
        'XData',[10*xp1(i) 10*xp2(i)], ...
        'YData',[10*yp1(i) 10*yp2(i)], ...
        'ZData',[10*zp1(i) 10*zp2(i)]);
    
    
    drawnow limitrate nocallbacks;

    % Capture frame
    frame = getframe(gcf);
    writeVideo(v, frame);

end

close(v);






