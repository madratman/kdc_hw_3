clc;
clear all;

jaco = [ 0         0    0.0045   -0.4877   -0.0942   -0.4995   -0.4305;
         0         0   -0.0448   -0.2123    0.1977   -0.6498    0.4524;
         0         0         0    0.1465    0.0203    0.2139    0.1209;
         0   -0.0998    0.1977   -0.3836    0.5334   -0.6981    0.7100;
         0    0.9950    0.0198    0.9216    0.1692    0.6414    0.5622;
    1.0000         0    0.9801    0.0587    0.8288    0.3183    0.4242];

theta_s = [0.1 0.2 0.3 0.4 0.5 0.6 0.7];
x_s = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122,-0.06664]';
x_d = [0.46320, 1.16402, 2.22058, -0.29301, 0.41901, 0.84979, 0.12817]';
% x_d = [0.49796, 0.98500, 2.34041,-0.11698, 0.07755, 0.82524, 0.54706 ]';

x_s_twist = get_twist_from_pose(x_s);
x_s_fk_twist = get_twist_from_homo(forward_kinematics(theta_s));
x_d_twist = get_twist_from_pose(x_d);

update_factor = 0.05;

x_curr_twist = x_s_fk_twist;%x_s_twist;
x_curr_homo = forward_kinematics(theta_s)
theta_curr = theta_s';
error_twist = x_curr_twist - x_d_twist;
error_trans = error_twist(1:3) + cross(x_curr_homo(1:3,4), error_twist(4:6));
error_twist(1:3) = error_trans

lambda = 0.001
delete 'results_ques_3_4_trajectory.txt';
fileID = fopen('results_ques_3_4_trajectory.txt', 'a');
max_iter = 5000;
iter = 1;
% error_vec = zeros(max_iter);
error_vec = [];
fprintf(fileID, '%f %f %f %f %f %f %f \n', theta_curr);

while norm(error_twist) > 0.01
    delta_theta_joints = update_factor*inv(jaco'*jaco+lambda^2*eye(7))*jaco'*error_twist;
    theta_curr = theta_curr + delta_theta_joints;
    x_curr_homo = forward_kinematics(theta_curr);
    x_curr_twist = get_twist_from_homo(x_curr_homo);
    theta_curr';
    error_twist = x_curr_twist - x_d_twist;
    error_trans = error_twist(1:3) + cross(x_curr_homo(1:3,4), error_twist(4:6));
    error_twist(1:3) = error_trans; % lol code
    norm(error_twist);
    iter = iter + 1;
    error_vec(iter) = norm(error_twist);
    if iter > max_iter
        break
    end
end

plot(error_vec);
saveas(gcf,'results_ques_3_4_error_plot.png');
