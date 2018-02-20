function dx = trunk(t, x)

global m I k1 k2 k3 D1 D2 D3 x_init Jacob Torques
g = -9.81;

% Trunk
M = zeros(6);
M(2, 2) = 1/m;
M(4, 4) = 1/m; 
M(6, 6) = 1/I;

 % According to the problem
F = [0;
     -k2*x(1) - D2*x(2);
     0;
     k1*(x_init(3)-x(3))-D1*x(4);
     0;
     -k3*x(5) - D3*x(6)];
A = [0 1 0 0 0 0;
     0 0 0 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 0 0;
     0 0 0 0 0 1;
     0 0 0 0 0 0];

 dx = A*x + M*F;
 % Assigning torques over time for plotting
 Torques(1:4, end+1) = Jacob*[F(2); F(4)+m*g; F(6)];