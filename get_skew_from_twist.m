function skew = get_skew_from_twist(xi)
    vel = xi(1:3); % 1 by 3
	omega = xi(4:6) % 1 by 3
    omega_skew = get_skew_from_vector(omega); % 3 by 3 
    skew = [omega_skew vel; % 4 by 4 
    		0 0 0 0];
end