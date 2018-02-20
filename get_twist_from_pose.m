% pose = x,y.z,q1,q2,q3,q0 not scalar at end
function twist = get_twist_from_pose(pose)
	pose_quat = [pose(7) pose(4) pose(5) pose(6)];
	pose_quat = pose_quat / norm(pose_quat);
	pose_rot = quat2rotm(pose_quat);
	pose_trans = pose(1:3);
	pose_homo = [pose_rot, pose_trans;
       			 0 0 0 1];
	twist = get_twist_from_homo(pose_homo);
