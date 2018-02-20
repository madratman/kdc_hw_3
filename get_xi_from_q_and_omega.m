function xi = get_xi_from_q_and_omega(q, omega)
	xi = [-cross(omega, q); omega];