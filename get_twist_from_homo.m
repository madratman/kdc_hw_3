% ref https://www.mathworks.com/matlabcentral/fileexchange/24589-kinematics-toolbox?focused=5126077&tab=function
% ref https://www.mathworks.com/matlabcentral/fileexchange/24589-kinematics-toolbox?focused=5126077&tab=function
function xi = get_twist_from_homo(homo)
    xi = zeros(6,1);

    pos = homo(1:3,4);
    rot = homo(1:3,1:3);

    % it's only a translation  
    if isequalf(rot, eye(3)),
        % if (R,p) == (I, 0)
        if isequalf(pos, zeros(3,1)),
          theta = 0;
        else  
          theta = norm(pos);
          xi(1:3) = pos/theta;
        end
    else

    trace_rot = (trace(rot)-1)/2;
    % this is only in case of numerical errors
    if trace_rot < -1,
        trace_rot = -1;
    elseif trace_rot > 1,
        trace_rot = 1;
    end

    theta = acos(trace_rot);

    if isequalf(pi,theta) || isequalf(0,theta),
        omega = null(rot-eye(3));
        omega = omega(:,1)/norm(omega(:,1));
    else
        omega = [rot(3,2) - rot(2,3); rot(1,3) - rot(3,1); rot(2,1) - rot(1,2) ];
        omega = omega/(2*sin(theta));
    end

    omega_hat = get_skew_from_vector(omega);

    A = (eye(3) - skew_exp(omega_hat, theta))*omega_hat + omega*omega'*theta;
    v = linsolve(A, %);

    xi(1:3) = v;
    xi(4:6) = omega;
    end

    
end