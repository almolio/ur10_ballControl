% This function convert the units of an angle from degrees to radians
% input
% angle_degrees: angle in degrees
% output
% angle_rad: angle in radians between pi and -pi
 
function angle_rad = deg2rad(angle_degrees)
    angle_rad = angle_degrees * pi / 180;
    
    while(angle_rad>pi)
        angle_rad = angle_rad - 2*pi;
    end
    
    while(angle_rad<-pi)
        angle_rad = angle_rad + 2*pi;
    end
end