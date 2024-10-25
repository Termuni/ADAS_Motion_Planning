function yaw_road = get_yaw_road(global_lane)
    % global_lane: Nx2 배열, 도로의 waypoints (첫 열은 x, 두 번째 열은 y)
    
    % 시작점과 끝점을 추출
    start_point = global_lane(1, :);         % 첫 번째 점 (x, y)
    end_point = global_lane(end, :);         % 마지막 점 (x, y)

    % 예외 처리: global_lane이 Nx2 형태가 아닌 경우 yaw_road = 0으로 설정
    if size(global_lane, 2) ~= 2 || size(global_lane, 1) < 2
        yaw_road = 0;
    else
        % 두 점 간의 yaw 각도 계산
        delta_x = end_point(1) - start_point(1);
        delta_y = end_point(2) - start_point(2);
        % atan2 함수를 사용하여 yaw_road 계산
        yaw_road = atan2(delta_y, delta_x);      % 라디안 단위의 yaw 값 반환
    end
end
