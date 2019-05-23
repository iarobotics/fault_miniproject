%% Use the system identification toolbox to generate the following state-space models
clear all
clc
close all

% Welding plant model
%load('models\plant_c2c.mat');
load('models\plant_c2cv.mat');
% figure()
% step(plant)

%% Check to see if the SS system is Observable and Controllable
[A, B, C, D] = ssdata(plant);
figure()
step(plant)

% % eigenvalues=poles of the system are all in RHP
% p1 = -0.9490 + 0.9920i;
% p2 = -0.9490 - 0.9920i;
p1 = -0.9490 + 1.7i;
p2 = -0.9490 - 1.7i;
p3 = -0.7325 + 0.0000i;
p4 = -0.9865 + 0.0000i;
p = [p1;p2;p3;p4];
%
%p = -1*eig(A);
%K = place(A,B,[p1 p2]);
K = place(A,B,p);
plant = ss(A-B*K,B,C,D);

[A, B, C, D] = ssdata(plant);
figure()
step(plant)

%E = ones(length(A),1);
E=[1;1;0;0];

Ob = obsv(A,C);
unob = length(A)-rank(Ob);

Co = ctrb(A,B);
unco = length(A) - rank(Co);

% Both the Observality and Controllability maytrix have full rank
if unob == 0 && unco == 0
    fprintf('System observable and detectable\n')
else
    error('System NOT observable/detectable\n')
end


%% Fault input matrices

Fx = [1;1;1;1];
Fy = [1;1];
 

%% Design of Luenberger Observer
p = 6*eig(A); % place eigenvalues 2-6 times  faster than the closed loop poles of the system
L = place(A',C',p)';


%% 1. Check that rank(CE) = rank(E) = 1:
if rank(C*E) == rank(E)
    fprintf('rank(CE) == rank(E)\n')
else
    error('rank(CE) != rank(E)\n')
end

%% 2. Compute H, T, A1

H = E/((C*E)'*(C*E)) * (C*E)';

T = eye(length(H)) - H*C;
A1 = T * A;

%%  Check the observability of (C,A1)
Ob = obsv(A1,C);
unob = length(A)-rank(Ob);
if unob == 0
    fprintf('(C,A1) Observable\n')
else
    error('(C,A1) NOT Observable\n')
end

% K1 can be computed by pole placement
K1=place(A1',C',p)';

%% 9. Compute F and K
F = A1 - K1*C;
K = K1 + F*H;
TB = T*B;
