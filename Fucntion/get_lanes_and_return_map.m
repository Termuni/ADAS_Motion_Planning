function [mapx, mapy] = get_lanes_and_return_map(global_lane1, global_lane2, global_lane3, global_lane4)
    % 입력된 네 개의 차선을 하나의 x, y 좌표 배열로 합침
    % 각 lane은 Nx2 행렬로 들어온다고 가정 (N개의 점, 2차원 좌표)
    % 예외 처리: global_lane이 Nx2 형태가 아닌 경우 yaw_road = 0으로 설정
    if size(global_lane1, 2) ~= 2 || size(global_lane1, 1) < 2 || size(global_lane2, 2) ~= 2 || size(global_lane2, 1) < 2 || size(global_lane3, 2) ~= 2 || size(global_lane3, 1) < 2 || size(global_lane4, 2) ~= 2 || size(global_lane4, 1) < 2
        mapx = zeros(15, 1);
        mapy = zeros(15, 1);
        return;
    end

    % 각 차선의 x 좌표와 y 좌표를 따로 추출
    x_lane1 = global_lane1(:, 1); % lane 1의 x 좌표
    y_lane1 = global_lane1(:, 2); % lane 1의 y 좌표
    
    x_lane2 = global_lane2(:, 1); % lane 2의 x 좌표
    y_lane2 = global_lane2(:, 2); % lane 2의 y 좌표
    
    x_lane3 = global_lane3(:, 1); % lane 3의 x 좌표
    y_lane3 = global_lane3(:, 2); % lane 3의 y 좌표
    
    x_lane4 = global_lane4(:, 1); % lane 4의 x 좌표
    y_lane4 = global_lane4(:, 2); % lane 4의 y 좌표
    
    % x 좌표와 y 좌표를 차선별로 이어 붙여서 전체 좌표 구성
    mapx = [x_lane1; x_lane2; x_lane3; x_lane4]; % 모든 차선의 x 좌표
    mapy = [y_lane1; y_lane2; y_lane3; y_lane4]; % 모든 차선의 y 좌표
end
