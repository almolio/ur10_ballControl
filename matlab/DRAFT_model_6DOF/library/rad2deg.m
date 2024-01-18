% This function convert the units of an angle from degrees to radians
% input
% angle_rad: angle in radians
% output
% angle_degrees: angle in degrees between 180 and -180

function angle_degrees = rad2deg(angle_rad)
    angle_degrees = angle_rad * 180 / pi;
    
    while(angle_degrees>180)
        angle_degrees = angle_degrees - 360;
    end
    
    while(angle_degrees<-180)
        angle_degrees = angle_degrees + 360;
    end
end