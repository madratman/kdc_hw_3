%% Question 4.b
theta = [-0.8462    1.2025   -0.3563    1.2025];

% Give in the problem statement
global m I k1 k2 k3 D1 D2 D3 x_init Torques Jacob
Torques= [];
m = 80;
I = 2;
k1 = 1000;
k2 = 100;
k3 = 100;
D1 = 300;  
D2 = 75; 
D3 = 75;

% pg 133 equations for bipedal robots
L1 = 0.5;
L2 = 0.5;
A = -L1*cos(theta(1))-L2*cos(theta(1)+theta(2));
B = -L1*sin(theta(1))-L2*sin(theta(1)+theta(2));
C = -L1*cos(theta(3))-L2*cos(theta(3)+theta(4));
D = -L1*sin(theta(3))-L2*sin(theta(3)+theta(4));
Q = -L2*cos(theta(1)+theta(2));
R = -L2*sin(theta(1)+theta(2));
S = -L2*cos(theta(3)+theta(4));
T = -L2*sin(theta(3)+theta(4));

% pg 134 equations
E = C*B-A*D;
V = Q*B-R*A;
W = S*D-T*C;

% Eq (14)
Jacob = [C*V/E, D*V/E, (-V-Q*D+R*C)/(2*E)-0.5;
                  0          0           -0.5;
        -A*W/E, -B*W/E, (W+S*B-T*A)/(2*E)-0.5;
                 0          0            -0.5];
 
 
% [x x_dot z zdot t t_dot] given from problem
x_init = [0, 0.1, 0.8, -1, 0, 0.1];

% Use matlab ode to solve 
[t, x] = ode45(@(t, x) trunk(t, x), [0 10], x_init);
figure(1); hold on;
plot(t, x(:,1),t, x(:,3),t, x(:,5),'LineWidth',2.0)
legend('X', 'Z', '\theta');
xlabel('time')


%% Question 4.c
figure(2); hold on;
t_torques = linspace(t(1), t(end), length(Torques));

plot(t_torques, Torques(1,:),'LineWidth',2.0)
plot(t_torques, Torques(2,:),'LineWidth',2.0)
plot(t_torques, Torques(3,:),'LineWidth',2.0)
plot(t_torques, Torques(4,:),'LineWidth',2.0)
legend('L knee', 'L hip', 'R knee', 'R hip');
xlabel('time');
