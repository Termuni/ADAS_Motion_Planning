function [mapx, mapy] = get_lanes_and_return_map(global_lane1_x, global_lane1_y, global_lane2_x, global_lane2_y, global_lane3_x, global_lane3_y, global_lane4_x, global_lane4_y);
    % 입력된 네 개의 차선을 하나의 x, y 좌표 배열로 합침
    % 각 lane은 Nx2 행렬로 들어온다고 가정 (N개의 점, 2차원 좌표)
    % 예외 처리: global_lane이 Nx2 형태가 아닌 경우 yaw_road = 0으로 설정
    
    % x 좌표와 y 좌표를 차선별로 이어 붙여서 전체 좌표 구성
    mapx = [global_lane1_x; global_lane2_x; global_lane3_x; global_lane4_x]; % 모든 차선의 x 좌표
    mapy = [global_lane1_y; global_lane2_y; global_lane3_y; global_lane4_y]; % 모든 차선의 y 좌표
end
