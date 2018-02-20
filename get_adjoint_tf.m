function adjoint = get_adjoint_tf(se3_matrix)
	R = se3_matrix(1:3,1:3);
	trans = se3_matrix(1:3,4)
    trans_skew = get_skew_from_vector(trans);
    ad = [R, trans_skew*R;
          zeros(3), R];
end