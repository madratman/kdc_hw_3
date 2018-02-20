clc;
clear all;

jaco = [ 0         0    0.0045   -0.4877   -0.0942   -0.4995   -0.4305;
         0         0   -0.0448   -0.2123    0.1977   -0.6498    0.4524;
         0         0         0    0.1465    0.0203    0.2139    0.1209;
         0   -0.0998    0.1977   -0.3836    0.5334   -0.6981    0.7100;
         0    0.9950    0.0198    0.9216    0.1692    0.6414    0.5622;
    1.0000         0    0.9801    0.0587    0.8288    0.3183    0.4242]

% jaco = jaco(:,1:7);
jaco_pinv = pinv(jaco);

theta_s = [0.1 0.2 0.3 0.4 0.5 0.6 0.7];
x_s = [0.44543, 1.12320, 2.22653, -0.29883, 0.44566, 0.84122,-0.06664]';
x_d = [0.46320, 1.16402, 2.22058, -0.29301, 0.41901, 0.84979, 0.12817]';
% x_d = [0.49796, 0.98500, 2.34041,-0.11698, 0.07755, 0.82524, 0.54706 ]';

update_factor = 0.001;

x_curr = x_s;
theta_curr = theta_s';

while norm(x_d - x_curr) > 0.1
    delta_vel_tip = get_delta_vel_tip(x_curr, x_d);
    delta_theta_joints = update_factor*jaco'*delta_vel_tip;
    theta_curr = wrapToPi(theta_curr + delta_theta_joints);
    theta_curr'
    x_curr_homo = forward_kinematics(theta_curr);
    x_quat = rotm2quat(x_curr_homo(1:3, 1:3));
    x_curr =  [x_curr_homo(1:3,4)', x_quat]';
    norm(x_d - x_curr)
end