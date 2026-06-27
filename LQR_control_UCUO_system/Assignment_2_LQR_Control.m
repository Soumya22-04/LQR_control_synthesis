%% ----------------------------------------------------------------------------
% -----------------------------------------------------------------------------
%  Name: Soumya Pramanik
%  Roll no. : 24EC10052
%  Part 1 of Assignment 2: Designing an LQR controller for a given system
% ----------------------------------------------------------------------------
% ----------------------------------------------------------------------------
%%

clc;
close all;
clear;

%% System parameters
A = [[-1.213213255856908 -0.052699540857194 0.114452537819400 -0.458503457153880 -0.147596519643947 0.330216171993562 -0.216306142694950 0.108524691675982 -0.277136364174417 0.213306124705031 -0.234400418544842 -0.694904339299891 -0.372985044024789 -0.121052395899524] 
     [-0.445138204194362 -1.087338372330932 0.506316210864373 0.420119557543333 0.489063639965910 -0.265671000243473 -0.336650153174219 -0.266447453528354 0.326773051102463 -0.796414105574600 -0.152486416125726 -0.360052673382867 -0.509891627577762 0.880961088466860]
     [0.484525786600994 0.174803353899379 -0.108630387296726 -0.147473032367577 -0.370609448513368 0.123445036090085 -0.391828220999852 0.432403506843010 0.040561080322674 1.548617577592084 -0.375626137358579 -0.964927403783107 0.116685942887537 0.195907069263956] 
     [-0.628653100827204 0.441686129222833 -0.196853639475424 -0.740518371993041 -0.203043754424680 -0.312951739221684 0.459277316310521 -1.144480648269924 -0.227661184417581 0.621646540385829 0.773529276032394 -0.824173422452114 -0.511336138546041 -0.143886170459940]
     [-0.526879261134897 0.496721354898758 -0.789140781845308 -0.351413813417401 -1.781412350680168 0.300097072779868 0.361009179150026 0.821998420305273 0.197670222666982 0.103894550459810 -0.518715568276280 0.458711846080412 -0.050433972884675 0.017439085643547] 
     [-0.162938036835619 0.115227089326697 0.230518490031468 0.226985493452003 -0.147401905420421 -1.073949490350060 0.492057044397738 0.809216490217785 -0.106022553236031 -0.200408629363208 0.219266594012242 -1.284874915967967 -0.219452186333267 0.086424084364871]
     [-0.937922024260352 0.295340626262125 0.081848014134194 0.702150763890030 0.460918049982761 -0.410290741684858 -1.656970404290512 0.515521785489352 0.039634647589333 0.551677571844913 0.910747253955778 -0.502106754099405 -1.258898348872255 0.174292978828746] 
     [0.727293350401215 -0.399923820740790 0.252644270324252 -1.022331509304516 0.714779970671842 0.160297998212142 -0.683617227260968 -0.862980196841881 -0.434983508111977 -0.749659627595721 -0.848332491977190 -0.073701377614643 0.047794275307910 -0.291037548459371]
     [-0.883478418688157 0.807423337770903 -0.272683699489586 0.267422703430696 -0.335917976087635 -0.355488407278101 1.117546803497625 0.475823305013847 -1.343243368591083 1.216242923480291 -0.117171815598981 -0.380364549164507 -0.827852126523950 0.367288610907304] 
     [-0.432084382768220 -0.208121861637220 1.692305102882171 0.996853318049892 -0.253373736345148 -1.076336539502446 0.349952751653385 0.243525258046507 0.467699882747476 -0.138850036162691 0.058652839684074 -0.660320581520629 0.238591279631722 -0.021125539243573]
     [-0.162352919624227 -0.976155299865440 -0.243225326820792 0.836239633325489 -0.433802463517382 0.411365862850318 0.130321781352260 -0.897920092095636 -0.228424553357463 -0.568131183985648 -0.593732855151864 -0.725643142274004 0.182526229046144 -0.147385751504529] 
     [-0.264370906345628 -0.184742644441694 -1.003940450295694 -0.848369091986984 0.494546791639666 -1.104312423847451 -0.472200324956500 -0.339323185019347 -0.108101954544411 -0.430160178902030 -0.930981520180108 0.321055052198516 0.688882757485099 -0.611757590647088]
     [0.575439200692222 -1.267310249593962 -0.215460248084041 -0.312938157224291 0.021629740368688 0.430071077232889 -1.163510493607768 -0.659695990919597 -0.382659856177330 0.430530215423247 -0.196984819655413 0.942614992066365 -0.194564541005406 -0.908760104887207] 
     [-0.214375145115562 0.987648714149219 0.442806720234774 -0.081107514151397 -0.216286104227307 -0.199881487887741 0.281210418831350 -0.068571314905706 0.190937766594154 -0.003347907463792 -0.101054426075363 -0.641224659163822 -1.242848391866568 -1.725651421647249]];

B = [[-0.051723763561734 -0.305270449184583 -0.015315868248594] 
     [-0.116732420024660 -0.475945089926195 0.149955878770381] 
     [-0.112898591360926 -0.041230352019466 -0.257329596734228] 
     [0.450703580365998 -0.195616995582868 0.021668047239566]
     [-0.175175023018630 -0.164833619607650 0.126111050083874] 
     [0.193325591568345 -0.201825450079648 0.001812952706111] 
     [-0.746946115214500 -0.888561633091335 -0.534882101127978]
     [0.118896143573814 0.003278035071589 0.438395454885899] 
     [-0.834413562808927 -0.496999682104461 -0.647097984062711] 
     [-0.486371252402602 -0.307207657585152 -0.536947426548773] 
     [0.293031461319545 0.647280729575534 -0.241442432395560] 
     [0.281522419773345 0.703605033443511 0.496061513668761] 
     [0.401731276643778 0.718304876069954 0.425248352781694] 
     [-0.207634639546252 -0.326879707471223 -0.260613735300743]];

C = [[0.695053806816924 -0.653901035211257 -0.216669043964540 -0.406643748943393 -0.337496928708196 0.800344919841996 -0.734680898447967 -0.383177337555783 -0.062081713564930 -0.164440009729601 -0.052596268307648 0.283621895206324 0.826159797514201 -0.281394936691068] 
     [0.170497399997020 -0.095533189106804 -0.211894215368707 -0.103936550394383 -0.119769092888598 0.354854849584245 -0.625481232333294 -0.485883175434064 0.198880592962716 -0.214578324733530 0.086360044518748 0.383477729131322 0.209925692065307 0.087002975029789]];

n = size(A,1);
%%Stability of A matrix

function dA_dt = odesol(t, A, X)
    dA_dt = A*X;
end

X0 = 0.01*ones(14,1);
tspan = linspace(0,10,50);
[t, x] = ode45(@(t,X) odesol(t, A, X), tspan, X0);
y = C*x';

%Phase portraits:
% figure;
% title("Relation between y_1 and y_2");
% xlabel("y_1");
% ylabel("y_2");
% h1 = animatedline("Color",'g','LineWidth',1.5);
% for i = 1:length(t)
%     addpoints(h1,y(1,i),y(2,i));
%     drawnow;
% end
figure;

%%Controllability and observability tests for the system
[v_a, d_a] = eig(A);
e_values = diag(d_a);
I = diag(ones(1,14));
flag1 = 0;
flag2 = 0;
for i=1:length(e_values)
    if rank([A-e_values(i)*I B])<n
        flag1 = 1;
        break;
    end
end

for i=1:length(e_values)
    if rank([(A-e_values(i)*I)' C']')<n
        flag2 = 1;
        break;
    end
end

if flag1==0
    disp("Controllable");
else
    disp("Non controllable")
end

if flag2==0
    disp("Observable");
else
    disp("Non observable");
end

%% Controllablility and observability matrices

C_script = [];
O_script = [];
for i = 1:n
    C_script = [C_script A^(i-1)*B];
    O_script = [[O_script]
                [C*A^(i-1)]];
end

%% Column spaces of controllability and observability matrices
V_c = orth(C_script);
V_o = orth(O_script');

RankC = size(V_c,2);
RankO = size(V_o,2);

U_c = null(C_script');
U_o = null(O_script);

%% Checking the stabilisability and detectability
A_c = V_c'*(A*V_c);
D_ac = eig(A_c);

A_o = V_o'*A*V_o;
D_ao = eig(A_o);

s1 = sum(D_ac>0);
S = sum(e_values>0);
s2 = sum(D_ao>0);

if s1==S && s2==S
    disp("The system maybe stabilisable and detectable");
end
flag1 = 0;
flag2 = 0;

for i = 1:length(e_values)
    if e_values(i)>0 && rank([A-e_values(i)*I B])<n
        flag1 = 1;
        break;
    end
end

for i = 1:length(e_values)
    if e_values(i)>0 && rank([(A-e_values(i)*I)' C']')<n
        flag2 = 1;
        break;
    end
end

if flag1==0 && flag2==0
    disp("This system is stabilisable and detectable");
end

%%Finding subspaces for different controllability and observability
%conditions(Kalman decomposition)
P_vc = V_c*V_c';
P_vo = V_o*V_o';
P_uo = U_o*U_o';
P_uc = U_c*U_c';
[Evec_co,e_co] = eig(P_vc*P_vo);
[Evec_uco,e_uco] = eig(P_uc*P_vo);
[Evec_cuo,e_cuo] = eig(P_vc*P_uo);
[Evec_ucuo,e_ucuo] = eig(P_uc*P_uo);

e_co = diag(real(e_co));
e_uco = diag(real(e_uco));
e_cuo = diag(real(e_cuo));
e_ucuo = diag(real(e_ucuo));

SSet_co = Evec_co(:,abs((e_co)-1)<1e-6);
SSet_uco = Evec_uco(:,abs((e_uco)-1)<1e-6);
SSet_cuo = Evec_cuo(:,abs((e_cuo)-1)<1e-6);
SSet_ucuo = Evec_ucuo(:,abs((e_ucuo)-1)<1e-6);

T = [SSet_cuo SSet_co SSet_ucuo SSet_uco];
A_decomposition = T\A*T;
B_decomposition = T\B;
C_decomposition = C*T;

A_co = A_decomposition(size(SSet_cuo,2)+1:size(SSet_cuo,2)+size(SSet_co,2),size(SSet_cuo,2)+1:size(SSet_cuo,2)+size(SSet_co,2));
B_co = B_decomposition(size(SSet_cuo,2)+1:size(SSet_cuo,2)+size(SSet_co,2),:);
C_co = C_decomposition(:,size(SSet_cuo,2)+1:size(SSet_cuo,2)+size(SSet_co,2));

sys = ss(A, B, C, zeros(2,3));
G = tf(sys);
step(G);
grid on;
sys2 = ss(A_co,B_co,C_co, zeros(2,3));
G2 = tf(sys2);
hold on;
step(G2);
legend('Direct','Reduced');


%% Luenberger observer
Q = null(C_co)';
P = [C_co' Q']';

A_observer = P*A_co/P;
B_observer = P*B_co;
C_observer = C_co/P;

A_11 = A_observer(1:2,1:2);
A_12 = A_observer(1:2,3:5);
A_21 = A_observer(3:5,1:2);
A_22 = A_observer(3:5,3:5);

B_1 = B_observer(1:2,:);
B_2 = B_observer(3:5,:);

mu = 20;
W = lyap((-mu*eye(3)-A_22)' ,A_12'*A_12);
L = 0.5*(A_12/(W))';
% % L = place(A_22',A_12',[-20 -21 -22])';

% I_ = eye(3);
% A_ro_ = (-18*I_ - A_22');
% M = kron(I_, A_ro_) + kron(A_ro_, I_);
% Q = A_12'*A_12;
% vec_Q = Q(:);
% vec_W = M \ (-vec_Q);
% W = reshape(vec_W, 3, 3);
% L = 0.5*(A_12/W)';


eig_observer = eig(A_22-L*A_12);

function dE_dT = errconv(t,E,A_22,A_12,L)
dE_dT = (A_22-L*A_12)*E;
end

E0 = [-1 0.35 -6];
tspan_err = [0 3];
[t,Y] = ode45(@(t,E) errconv(t,E,A_22,A_12,L), tspan_err,E0);

% Plot the error convergence
figure;
plot(t, Y);
xlabel('Time (s)');
ylabel('Error');
title('Error Convergence of the Luenberger Observer (system free estimate)');
grid on;

% System simulation
function dX_dt = obserr(t,X,A_co,A_22,A_12,C_co,L,A_21,A_11)
dx_dt = A_co*X(1:5);
% z = x_hat - L*y
x = X(1:5,:);
z = X(6:8,:);
y = C_co*x;
%dz_dt = (A_22-L*A_12)*(X(6:8)+L*C_co*X(1:5)) + (A_21 - L*A_11)*C_co*X(1:5);
dz_dt = (A_21-L*A_11+(A_22-L*A_12)*L)*y + (A_22 - L*A_12)*z;
dX_dt = [dx_dt;dz_dt];
end 

x = rand(5,1);
x_hat = rand(3,1);
y = C_co*x;
X = [x;x_hat];
terr = [0 1];
[t,Y] = ode45(@(t,X) obserr(t,X,A_co,A_22,A_12,C_co,L,A_21,A_11), terr,X);

Y = Y';
x_real = Y(1:5,:);
x_bar = [C_co*Y(1:5,:); Y(6:8,:)+L*C_co*x_real];
x_bar2 = P\x_bar;

err = x_real-x_bar2;
% Plot the error convergence
figure;
plot(t, err);
xlabel('Time (s)');
ylabel('Error');
title('Error Convergence of the Luenberger Observer (estimated with system)');
grid on;

%% ARE Solution

R_tilda = eye(size(B_co,2));
Q_tilda = eye(size(C_co,1));

R = -B_co*inv(R_tilda)*B_co';
Q = C_co'*inv(Q_tilda)*C_co;

H = [A_co R;-Q -A_co'];

[Evec_H,eig_H] = eig(H);
eig_H = diag(eig_H);
nEvec_H = [];
pEvec_H = [];

for i=1:length(eig_H)
    if real(eig_H(i)) <0
        nEvec_H = [nEvec_H Evec_H(:,i)];
    end
end

X1 = nEvec_H(1:size(A_co,1),:);
X2 = nEvec_H(size(A_co,1)+1:2*size(A_co,1),:);
P_sol = X2/X1;

err_matrix = A_co'*P_sol+P_sol*A_co+ Q + P_sol*R*P_sol;

%% System simulation under LQR contol (system state used)

K = R_tilda\B_co'*P_sol;
A_cl = A_co-B_co*K;
eig_cl = eig(A_cl);

function c = control(t, x, A, K, B)
u = -K*x;
c = A*x + B*u;
end

figure;
x0 = rand(5,14);
t_con = [0 10];
for i = 1:size(x0,2)
    [t_con, x_con]  = ode45(@(t,x) control(t, x, A_co, K, B_co), t_con, x0(:,i));
    y = C_co*x_con';
    hold on;
    plot(y(1,:),y(2,:),'LineWidth',1);
    hold on;
    plot(y(1,1),y(2,1),'o','MarkerFaceColor','none');
    hold on;
    plot(y(1,end),y(2,end),'x','MarkerFaceColor','none');
end
xlabel("Y1");
ylabel("Y2");
title("Phase portrait of the reduced system");
grid on;
hold off;
    
figure;
plot(t_con,y);
legend('y_1', 'y_2');
xlabel("Time");
ylabel("Y");
yline(0);
title("Reduced system output with LQR control");

%% System simulation under LQR control (using observed state)
function S = sys_controller(t,x,A_11,A_12,A_21,A_22,A,B,C,L,P,T,K)
    B_ = T\B;
    B_co = B_(5:9,:);
    B_observer = P*B_co;
    B1 = B_observer(1:2,:);
    B2 = B_observer(3:5,:);
    %z = x2_hat - Ly
    z = x(15:17);
    y = C*x(1:14);
    x_obs = P\[y;z+L*y];
    u = -K*x_obs;
    dx_dt = A*x(1:14) + B*u;
    dz_dt = (A_22-L*A_12)*(z+L*y) + (B2 - L*B1)*u + (A_21 - L*A_11)*y;
    S = [dx_dt;dz_dt];
end

figure;
x0 = rand(17,6);
t_con = [0 10];
desired_end = [1 1 1 2 2 2 4 4 4 0 0 0 0 0];
for i = 1:size(x0,2)
    [t_con, S]  = ode45(@(t,x) sys_controller(t,x,A_11,A_12,A_21,A_22,A,B,C,L,P,T,K), t_con, x0(:,i));
    y = C*(S(:,1:14))';
    hold on;
    plot(y(1,:),y(2,:),'LineWidth',1);
    hold on;
    plot(y(1,end),y(2,end),'o','MarkerFaceColor','none');
end
xlabel("Y1");
ylabel("Y2");
title("Phase portrait of the full system contol");
grid on;
hold off;

figure;
plot(t_con,y);
legend('y_1', 'y_2');
xlabel("Time");
ylabel("Y");
yline(0);
title("Full system simulation output");

%% Full system simulation with noise

function S = sys_controller_noisy(t,x,A_11,A_12,A_21,A_22,A,B,C,L,P,T,K)
    B_ = T\B;
    B_co = B_(5:9,:);
    B_observer = P*B_co;
    B1 = B_observer(1:2,:);
    B2 = B_observer(3:5,:);
    %z = x2_hat - Ly
    z = x(15:17);
    y = C*x(1:14);
    x_obs = P\[y;z+L*y];
    u = -K*x_obs + 0.5*randn(size(K*x_obs)); 
    dx_dt = A*x(1:14) + B*u;
    dz_dt = (A_22-L*A_12)*(z+L*y) + (B2 - L*B1)*u + (A_21 - L*A_11)*y;
    S = [dx_dt;dz_dt];
end


figure;
x0 = rand(17,3);
t_con = [0 10];
desired_end = [1 1 1 2 2 2 4 4 4 0 0 0 0 0];
for i = 1:size(x0,2)
    [t_con, S]  = ode45(@(t,x) sys_controller_noisy(t,x,A_11,A_12,A_21,A_22,A,B,C,L,P,T,K), t_con, x0(:,i));
    y = C*(S(:,1:14))';
    hold on;
    plot(y(1,:),y(2,:),'LineWidth',1);
    hold on;
    plot(y(1,end),y(2,end),'o','MarkerFaceColor','none');
end
xlabel("Y1");
ylabel("Y2");
title("Phase portrait of the full system contol(added noise)");
grid on;
hold off;

figure;
plot(t_con,y);
legend('y_1', 'y_2');
xlabel("Time");
ylabel("Y");
yline(0);
title("Full system simulation output (noise added)");

%% Printed results
% 
% Non controllable
% Non observable
% The system maybe stabilisable and detectable
% This system is stabilisable and detectable