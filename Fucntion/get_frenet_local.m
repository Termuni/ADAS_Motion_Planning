function [frenet_s, frenet_d] = get_frenet_local(ego_idx, local_mapx, local_mapy, prev_s)
    % 다음 경로점 계산
    next_wp = next_waypoint(ego_idx, local_mapx, local_mapy);
    
    prev_wp = max(next_wp - 1, 1);  % 경계 설정
    
    % 경로 벡터와 차량 위치 벡터 계산
    n_x = local_mapx(next_wp) - local_mapx(prev_wp);
    n_y = local_mapy(next_wp) - local_mapy(prev_wp);
    x_x = local_mapx(ego_idx) - local_mapx(prev_wp);
    x_y = local_mapy(ego_idx) - local_mapy(prev_wp);

    % 내적을 이용한 투영 벡터 계산
    denom = (n_x^2 + n_y^2);
    
    % 투영 계산 시 분모가 0이 되지 않도록 체크
    if denom == 0
        proj_norm = 0;
    else
        proj_norm = (x_x * n_x + x_y * n_y) / denom;
    end
    
    proj_x = local_mapx(prev_wp) + proj_norm * n_x;
    proj_y = local_mapy(prev_wp) + proj_norm * n_y;

    % Frenet d 값 계산
    frenet_d = get_dist(local_mapx(ego_idx), local_mapy(ego_idx), proj_x, proj_y);

    % 방향에 따른 부호 결정 (크로스 프로덕트)
    ego_vec = [x_x, x_y, 0];
    map_vec = [n_x, n_y, 0];
    d_cross = cross(ego_vec, map_vec);

    if d_cross(3) > 0
        frenet_d = -frenet_d;
    end

    % Frenet s 값 계산 
    % 이전 웨이포인트까지의 거리
    frenet_s = prev_s;
    % 현재 웨이포인트에서 투영 좌표까지의 거리 추가
    frenet_s = frenet_s + get_dist(local_mapx(prev_wp), local_mapy(prev_wp), proj_x, proj_y);
    
end

function next_wp = next_waypoint(ego_idx, mapx, mapy)
    closest_wp = get_closest_waypoints(ego_idx, mapx, mapy);
    
    % 경계 처리: 마지막 웨이포인트에 도달하면 마지막 웨이포인트 유지
    if closest_wp >= length(mapx)
        next_wp = length(mapx);  % 경로가 순환하지 않으므로 마지막 웨이포인트로 설정
        return;
    end

    % 다음 웨이포인트 계산
    map_vec = [mapx(closest_wp + 1) - mapx(closest_wp), mapy(closest_wp + 1) - mapy(closest_wp)];
    ego_vec = [mapx(ego_idx) - mapx(closest_wp), mapy(ego_idx) - mapy(closest_wp)];

    direction = sign(dot(map_vec, ego_vec));

    if direction >= 0
        next_wp = closest_wp + 1;
    else
        next_wp = closest_wp;
    end
end


function closest_wp = get_closest_waypoints(ego_idx, mapx, mapy)
    min_len = Inf;
    closest_wp = 0;  % 웨이포인트는 1에서 시작

    for i = 1:length(mapx)
        dist = get_dist(mapx(ego_idx), mapy(ego_idx), mapx(i), mapy(i));
        if dist < min_len
            min_len = dist;
            closest_wp = i;
        end
    end
end

function dist = get_dist(x1, y1, x2, y2)
    % 두 좌표 간의 거리 계산
    dist = sqrt((x1 - x2)^2 + (y1 - y2)^2);
end
