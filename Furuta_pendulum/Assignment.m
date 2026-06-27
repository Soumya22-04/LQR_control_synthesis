clc;
close all;
clear;

% Masses and dimensions
m = [0.5 0.04];  % mass of moving rods
l = [0.1 0.08];    % length of moving rods
J = 6e-7;     % MOI of the axis
b = [5e-6, 3e-6];  %Drag coefficients due to joints
g = 9.8;
h = 0.15;  %Height of the axis

%Coefficients for calculation
alpha = zeros(1,4);
alpha(1) = (m(1)*l(1)^2)/3;
alpha(2) = m(2)*l(1)^2;
alpha(3) = m(2)*l(2)^2/3;
alpha(4) = m(2)*l(1)*l(2)/2;

%Non linear model
function dA_dt = odefun(t, X,alpha,b,J,m,l,g)
    dA_dt = zeros(4,1);
    dA_dt(1) = X(2);
    dA_dt(2) = (alpha(3)*(-b(1)*X(2)-alpha(3)*sin(2*X(3))*X(4)*X(2)-alpha(4)*sin(X(3))*(X(4))^2)+alpha(4)*cos(X(3))*(-b(2)*X(4)+alpha(3)*sin(2*X(3))*(X(2))^2/2-m(2)*l(2)*g*sin(X(3))/2))/(alpha(3)*(J + alpha(1)+alpha(2)+alpha(3)*sin(X(3))^2)-alpha(4)^2*cos(X(3))^2);
    dA_dt(3) = X(4);
    dA_dt(4) = ((J + alpha(1)+alpha(2)+alpha(3)*sin(X(3))^2)*(-b(2)*X(4)+alpha(3)*sin(2*X(3))*(X(2))^2/2-m(2)*l(2)*g*sin(X(3))/2)+alpha(4)*(-b(1)*X(2)-alpha(3)*sin(2*X(3))*X(4)*X(2)-alpha(4)*sin(X(3))*(X(4))^2))/(alpha(3)*(J + alpha(1)+alpha(2)+alpha(3)*sin(X(3))^2)-alpha(4)^2*cos(X(3))^2);
end

X0 = [0.2 0.5 pi -1.002];
tspan = [0 3];
[t, A] = ode45(@(t,y) odefun(t, y, alpha, b, J, m, l,g), tspan, X0);

%Phase potraits
h1 = animatedline('Color','b','Linewidth', 1.5);

for i=1:length(t)     %Relation between phi and phi dot

    addpoints(h1,A(i,3),A(i,4));

    pause(0.005);
    drawnow;
end
title("Phase potrait phi vs phi dot");
xlabel("phi")
ylabel("phi dot")
grid on;
hold off;
figure;
h2 = animatedline('Color','b','Linewidth', 1.5);

for j=1:length(t)    %Relation between theta and theta dot

    addpoints(h2,A(j,1),A(j,2));

    pause(0.005);
    drawnow;
end
title("Phase potrait theta vs theta dot");
xlabel("theta")
ylabel("theta dot")
grid on;
hold off;
figure;

% 3D plots:
theta = A(:,1);
phi = A(:,3);

% Coordinates of the pendulum
xp2 = l(1)*cos(theta)+l(2)*sin(phi).*sin(theta);
yp2 = l(1)*sin(theta)-l(2)*sin(phi).*cos(theta);
zp2 = h*ones(size(phi)) - l(2)*cos(phi);

%Coordinates of the rotating rod
xp1 = l(1)*cos(theta);
yp1 = l(1)*sin(theta);
zp1 = h*ones(size(phi));



h_axis = plot3([0 0], [0 0], [0 10*h], 'k', 'LineWidth',2);
hold on;
h_arm1 = plot3([0 10*xp1(1)], [0 10*yp1(1)], [10*h 10*zp1(1)], 'k', 'LineWidth', 2);
hold on;
h_arm2 = plot3([10*xp1(1) 10*xp2(1)], [10*yp1(1) 10*yp2(1)], [10*zp1(1) 10*zp2(1)], 'b', 'LineWidth', 2);
h_bob = plot3(10*xp2(1), 10*yp2(1), 10*zp2(1), 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 5);

grid on; 
axis equal;

axis([-2 2 -2 2 -0.5 2]); 
view(45, 30); 



%Pendulum animation
for i = 1:length(t)
    set(h_axis, 'XData',[0 0], 'YData',[0 0], 'ZData',[0 10*zp1(i)]);
    set(h_arm1, 'XData', [0 10*xp1(i)], 'YData', [0 10*yp1(i)], 'ZData', [10*h 10*zp1(i)]);
    set(h_arm2, 'XData', [10*xp1(i) 10*xp2(i)], 'YData', [10*yp1(i) 10*yp2(i)], 'ZData', [10*zp1(i) 10*zp2(i)]);
    set(h_bob, 'XData', 10*xp2(i), 'YData', 10*yp2(i), 'ZData', 10*zp2(i));

    drawnow;

    pause(0.01); 
   
end




%Linearised model
X1 = [0.2 1.5 -0.001 -0.02];
X2 = [0.2 1.5 pi-0.001 -0.02];
d = alpha(3)*(alpha(1)+alpha(2))-alpha(4)^2;

% Linearised system for phi = 0
A_0 = [[0 1 0 0]
        [0 -alpha(3)*b(1)/d -alpha(4)*m(2)*l(2)*g/(2*d) -alpha(4)*b(2)/d]
        [0 0 0 1]
        [0 -alpha(4)*b(1)/d -(alpha(1)+alpha(2))*m(2)*l(2)*g/(2*d) -(alpha(1)+alpha(2))*b(2)/(2*d)]];

%Linearised system for phi = pi
A_pi = [[0 1 0 0]
        [0 -alpha(3)*b(1)/d -alpha(4)*m(2)*l(2)*g/(2*d) alpha(4)*b(2)/d]
        [0 0 0 1]
        [0 alpha(4)*b(1)/d (alpha(1)+alpha(2))*m(2)*l(2)*g/(2*d) -(alpha(1)+alpha(2))*b(2)/(2*d)]];

function dA_dt = odefun1(t,X,A_0)
    dA_dt = zeros(4,1);
    dA_dt = A_0*X;
end


function dA_dt = odefun2(t,X,A_pi)
    dA_dt = zeros(4,1);
    dA_dt = A_pi*X;
end

%ODE solvers and phase potraits
figure;
tspan = [0 2];
[t, A] = ode45(@(t,y) odefun1(t, y, A_0), tspan, X1);

h3 = animatedline('Color','b','Linewidth', 1.5);

for i=1:length(t)   % phi vs phi dot for phi close to 0

    addpoints(h3,A(i,3),A(i,4));

    pause(0.005);
    drawnow;
end
title("Phase potrait phi vs phi dot close to phi = 0");
xlabel("phi")
ylabel("phi dot")
grid on;
hold off;

figure;
[t, A] = ode45(@(t,y) odefun2(t, y, A_pi), tspan, X2);

h4 = animatedline('Color','b','Linewidth', 1.5);

for i=1:length(t)   % phi vs phi dot for phi close to pi

    addpoints(h4,A(i,3),A(i,4));

    pause(0.005);
    drawnow;
end
title("Phase potrait phi vs phi dot close to phi = pi");
xlabel("phi")
ylabel("phi dot")
grid on;
hold off;