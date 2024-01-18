function [ Ry ] = RotY( betaY )

% TODO: Fill with the proper rotation matrix

Ry=eye(3);
Ry(1,1) = cos(betaY);
Ry(1,3) = sin(betaY);
Ry(3,1) = -sin(betaY);
Ry(3,3) = cos(betaY);

end

