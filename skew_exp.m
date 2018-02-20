% ref https://www.mathworks.com/matlabcentral/fileexchange/24589-kinematics-toolbox?focused=5126077&tab=function
function g = skew_exp(s, theta)
	for i=1:size(theta,2),
		g(:,:,i) = eye(3) + s * sin(theta(i)) + s^2 * (1 - cos(theta(i)));
	end
end