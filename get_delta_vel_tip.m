function delta_vel = get_delta_vel_tip(x_s, x_d)
% w,x,y,z
q_s = [x_s(4) x_s(5) x_s(6) x_s(7)];
q_d = [x_d(4) x_d(5) x_d(6) x_d(7)];
q_d = q_d/norm(q_d);
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
delta_quat = delta_quat / norm(delta_quat);
delta_theta = 2*acos(delta_q_1);

delta_omega = delta_quat(2:4)' / sin(delta_theta/2);
delta_x = x_d(1:3) - x_s(1:3);

delta_vel = [delta_x; delta_omega];
