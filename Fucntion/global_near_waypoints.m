function [global_points_lane1, global_points_lane2, global_points_lane3, global_points_lane4] = global_near_waypoints(global_lane1, global_lane2, global_lane3, global_lane4, ego_x, ego_y)
    % 결과 배열 초기화
    global_points_lane1 = zeros(15, 2);
    global_points_lane2 = zeros(15, 2);
    global_points_lane3 = zeros(15, 2);
    global_points_lane4 = zeros(15, 2);
    
    % 각 라인별 근처 waypoints 계산
    global_points_lane1 = find_near_waypoints(global_lane1, ego_x, ego_y);
    global_points_lane2 = find_near_waypoints(global_lane2, ego_x, ego_y);
    global_points_lane3 = find_near_waypoints(global_lane3, ego_x, ego_y);
    global_points_lane4 = find_near_waypoints(global_lane4, ego_x, ego_y);
end

function near_points = find_near_waypoints(global_lane, ego_x, ego_y)
    % 각 웨이포인트와 ego 위치 간의 거리 계산
    distances = sqrt((global_lane(:, 1) - ego_x).^2 + (global_lane(:, 2) - ego_y).^2);
    
    % 거리 기준으로 웨이포인트 정렬
    [~, sorted_indices] = sort(distances);
    sorted_lane = global_lane(sorted_indices, :);
    sorted_distances = distances(sorted_indices);
    
    % -50~0 사이에서 5개, 0~100 사이에서 10개 포인트 추출
    near_points = zeros(15, 2);
    count = 1;
    
    % -50 ~ 0m 구간에서 최대 5개 waypoints 추출
    for i = 1:length(sorted_distances)
        if sorted_distances(i) >= -50 && sorted_distances(i) < 0
            near_points(count, :) = sorted_lane(i, :);
            count = count + 1;
            if count > 5
                break;
            end
        end
    end
    
    % 0 ~ 100m 구간에서 최대 10개 waypoints 추가
    for i = 1:length(sorted_distances)
        if sorted_distances(i) >= 0 && sorted_distances(i) <= 100
            near_points(count, :) = sorted_lane(i, :);
            count = count + 1;
            if count > 15
                break;
            end
        end
    end
end
