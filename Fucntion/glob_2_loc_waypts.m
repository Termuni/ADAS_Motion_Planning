function local_waypoints_ = glob_2_loc_waypts(glob_waypoints, ego_glob_pos, ego_yaw)
%GLOB_2_LOC_WAYPTS 이 함수의 요약 설명 위치
%   자세한 설명 위치

    num_waypoints = size(waypoints, 1);
    local_waypoints_ = zeros(num_waypoints, 2);

    % 헤딩 각에 대한 코사인 및 사인 계산
    cos_theta = cos(head);
    sin_theta = sin(head);

    for i = 1:num_waypoints
        % 글로벌 좌표에서 차량 위치를 빼서 상대적 위치 계산
        X_diff = waypoints(i, 1) - vehicle_position(1);
        Y_diff = waypoints(i, 2) - vehicle_position(2);
        
        % 2D 회전 변환을 통해 로컬 좌표로 변환
        local_waypoints_(i, 1) = X_diff * cos_theta + Y_diff * sin_theta;
        local_waypoints_(i, 2) = -X_diff * sin_theta + Y_diff * cos_theta;
    end

end
