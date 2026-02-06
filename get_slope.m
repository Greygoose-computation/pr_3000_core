function [h,v] = get_slope(slope_angle)
    v=sind(slope_angle);
    h=cosd(slope_angle);
end