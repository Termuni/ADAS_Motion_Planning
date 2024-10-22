function steering_angle = pure_pursuit_control(x, y, yaw, pathx, pathy,invalid, lookahead_distance)
    % 차량의 현재 위치에서 가장 가까운 경로 상의 점 찾기
    valid_indices = find(invalid > 0);
    path_x = pathx(valid_indices);
    path_y = pathy(valid_indices);

    distances = sqrt((path_x - x).^2 + (path_y - y).^2);
    [~, min_index] = min(distances);
    
    % 경로 상의 목표 지점 선택
    lookahead_index = min_index;
    while lookahead_index < length(path_x) && distances(lookahead_index) < lookahead_distance
        lookahead_index = lookahead_index + 1;
    end
    
    if lookahead_index >= length(path_x)
        lookahead_index = length(path_x);
    end
    
    target_x = path_x(lookahead_index);
    target_y = path_y(lookahead_index);
    
    % 차량 좌표계에서의 목표 지점 계산
    dx = target_x - x;
    dy = target_y - y;
    target_distance = sqrt(dx^2 + dy^2);
    
    target_local_x = dx * cos(yaw) + dy * sin(yaw);
    target_local_y = -dx * sin(yaw) + dy * cos(yaw);
    
    % 조향각 계산
    steering_angle = atan2(2 * target_local_y * lookahead_distance, target_distance^2);
end