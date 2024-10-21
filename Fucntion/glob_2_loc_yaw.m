function yaw = glob_2_loc_yaw(x1,x2,y1,y2)
    yaw = -pi + atan2(y1-y2, x1-x2);
end

