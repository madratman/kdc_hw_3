% implements eqn 3.54 on pg 116 of book 
clear all;
clc;

theta = [0.1 0.2 0.3 0.4 0.5 0.6 0.7];

% J1, Z0
q1 = [0 0 0]';
omega_1 = [0 0 1]'; 
xi_1 = get_xi_from_q_and_omega(q1, omega_1);

% J2, Z1
q2 = [0 0 0]';
omega_2 = [0 1 0]'; 
xi_2 = get_xi_from_q_and_omega(q2, omega_2);

% J3, Z2
q3 = [0.045 0 0.550]';
omega_3 = [0 0 1]'; 
xi_3 = get_xi_from_q_and_omega(q3, omega_3);

% J4, Z3
q4 = [0.045 0 0.550]';
omega_4 = [0 1 0]'; 
xi_4 = get_xi_from_q_and_omega(q4, omega_4);

% J5, Z4
q5 = [0 0 0.850]';
omega_5 = [0 0 1]'; 
xi_5 = get_xi_from_q_and_omega(q5, omega_5);

% J6, Z5
q6 = [0 0 0.850]';
omega_6 = [0 1 0]'; 
xi_6 = get_xi_from_q_and_omega(q6, omega_6);

% J7, Z6
q7 = [0 0 0.910]';
omega_7 = [0 0 1]';
xi_7 = get_xi_from_q_and_omega(q7, omega_7);

% todo this is the marker tip. to include?
q8 = [0 0 1.03]';
omega_8 = [0 0 1]';
xi_8 = get_xi_from_q_and_omega(q8, omega_8);

manipulator_jacobian = xi_1; % eqn 3.54

all_xis = [xi_1 xi_2 xi_3 xi_4 xi_5 xi_6 xi_7 xi_8];

% following loop essentially implements eqn 3.54 for all xi's
% xi_n_hat = adjoint( exp(xi_1_skew*theta_1) * exp(xi_2_skew*theta_2) ... exp(xi_n-1_skew*theta_n-1))
				% * (xi_n-1)
prod_of_xi_exp = eye(4);
for i = 2:size(all_xis,2)
	curr_xi = all_xis(:,i-1);
	next_xi = all_xis(:,i);
	curr_xi_twist = get_skew_from_twist(curr_xi);
    prod_of_xi_exp = prod_of_xi_exp * expm( curr_xi_twist * theta(i-1) );
    manipulator_jacobian = [manipulator_jacobian, get_adjoint_tf(prod_of_xi_exp)*next_xi];
end

% print
manipulator_jacobian

% x y z qi qj qk q0
x_s = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122,-0.06664]';
x_d = [0.46320, 1.16402, 2.22058, -0.29301, 0.41901, 0.84979, 0.12817]';

g_s = [-0.81252, -0.15424, -0.56216, 0.44543;
       -0.37847, -0.59390,  0.70996, 1.12320;
       -0.44337,  0.78962,  0.42418, 2.22653;
        0.00000,  0.00000,  0.00000, 1.00000];
   
q_s = [x_s(7) x_s(4) x_s(5) x_s(6)];
q_d = [x_d(7) x_d(4) x_d(5) x_d(6)];

% https://www.mathworks.com/help/aerotbx/ug/quatinv.html
q_s_inv = [q_s(1), -q_s(2), -q_s(3), -q_s(4)];
q_s_inv = q_s_inv / (norm(q_s)^2);

% relative rotation = q_final * q_initial.inverse
% https://www.mathworks.com/help/aerotbx/ug/quatmultiply.html
delta_q_1 = [q_d(1)*q_s_inv(1) - q_d(2)*q_s_inv(2) - q_d(3)*q_s_inv(3) - q_d(4)*q_s_inv(4)];
delta_q_2 = [q_d(1)*q_s_inv(2) - q_d(2)*q_s_inv(1) - q_d(3)*q_s_inv(4) - q_d(4)*q_s_inv(3)];
delta_q_3 = [q_d(1)*q_s_inv(3) - q_d(2)*q_s_inv(3) - q_d(3)*q_s_inv(1) - q_d(4)*q_s_inv(2)];
delta_q_4 = [q_d(1)*q_s_inv(4) - q_d(2)*q_s_inv(2) - q_d(3)*q_s_inv(2) - q_d(4)*q_s_inv(1)];

delta_quat = [delta_q_1, delta_q_2, delta_q_3, delta_q_4];
delta_theta = 2*acos(delta_q_1);

delta_omega = delta_quat(2:4)' / sin(delta_theta/2);
delta_x = x_d(1:3) - x_s(1:3);

delta_vel_tip = [delta_x; delta_omega];
delta_theta_joints = pinv(manipulator_jacobian)*delta_vel_tip
