function [yaw_value, s_d_value] = calculate_frenet_and_paths(si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd, df_d, df_dd, opt_d, mapx, mapy, maps)

    % Define constants
    TARGET_SPEED = 1;
    LANE_WIDTH = 0.39;
    DF_SET = [LANE_WIDTH/2, -LANE_WIDTH/2];
    MIN_T = 1; 
    MAX_T = 2; 
    DT_T = 1; 
    DT = 0.1; 
    MAX_POINTS = (MAX_T / DT) + 1;  % 사전 계산된 최대 포인트 수
    max_paths = length(DF_SET) * ((MAX_T - MIN_T) / DT_T + 1);

    % Cost weights
    K_J = 0.1; 
    K_T = 0.1; 
    K_D = 1.0; 
    K_V = 1.0; 
    K_LAT = 1.0; 
    K_LON = 1.0; 
    V_MAX = 2;        % Maximum velocity (example)
    ACC_MAX = 2;       % Maximum acceleration (example)
    K_MAX = 4;       % Maximum curvature (example)

    % Pre-allocate fixed-size struct array
    frenet_paths = repmat(initialize_frenet_path(MAX_POINTS), 1, max_paths);  % 고정 크기의 구조체 배열
    valid_paths = repmat(initialize_frenet_path(MAX_POINTS), 1, max_paths);  % 고정 크기 배열로 지정
    
    path_count = 1;  % Path index tracker
    valid_count = 1;  % Valid path index tracker

    % 각 lateral 목표로 경로 생성
    for df = DF_SET
        for T = MIN_T:DT_T:MAX_T
            if path_count > max_paths
                break;  % 최대 경로 수 초과 시 종료
            end
            
            % Initialize Frenet path struct
            fp = initialize_frenet_path(MAX_POINTS);

            % Quintic polynomial for lateral trajectory
            lat_traj = QuinticPolynomial(di, di_d, di_dd, df, df_d, df_dd, T);

            % Calculate lateral trajectory
            for i = 1:MAX_POINTS
                t = (i-1) * DT;
                if t > T
                    break;
                end
                fp.t(i) = t;
                fp.d(i) = lat_traj.calc_pos(t);
                fp.d_d(i) = lat_traj.calc_vel(t);
                fp.d_dd(i) = lat_traj.calc_acc(t);
                fp.d_ddd(i) = lat_traj.calc_jerk(t);
            end

            % Longitudinal motion planning
            lon_traj = QuarticPolynomial(si, si_d, si_dd, sf_d, sf_dd, T);

            % Calculate longitudinal trajectory
            for i = 1:MAX_POINTS
                t = fp.t(i);
                if t > T
                    break;
                end
                fp.s(i) = lon_traj.calc_pos(t);
                fp.s_d(i) = lon_traj.calc_vel(t);
                fp.s_dd(i) = lon_traj.calc_acc(t);
                fp.s_ddd(i) = lon_traj.calc_jerk(t);
            end

            % Calculate costs
            J_lat = sum(fp.d_ddd .^ 2);  % Lateral jerk
            J_lon = sum(fp.s_ddd .^ 2);  % Longitudinal jerk

            % Consistency cost
            d_diff = (fp.d(end) - opt_d) ^ 2;
            v_diff = (TARGET_SPEED - fp.s_d(end)) ^ 2;

            % Update costs
            fp.c_lat = K_J * J_lat + K_T * T + K_D * d_diff;
            fp.c_lon = K_J * J_lon + K_T * T + K_V * v_diff;
            fp.c_tot = K_LAT * fp.c_lat + K_LON * fp.c_lon;

            % ---- Global Path Conversion ----
            % Frenet 좌표에서 Global 좌표로 변환
            for i = 1:length(fp.s)
                s_s = fp.s(i);
                d_d = fp.d(i);
                [fp.x(i), fp.y(i),~] = get_cartesian(s_s, d_d, mapx, mapy, maps);
            end

            % Yaw와 ds(거리 차이) 계산
            for i = 1:length(fp.x) - 1
                dx = fp.x(i + 1) - fp.x(i);
                dy = fp.y(i + 1) - fp.y(i);
                fp.yaw(i) = atan2(dy, dx);    % 방향 각도 계산
                fp.ds(i) = hypot(dx, dy);     % 두 점 사이의 거리 계산
            end
            
            % Yaw와 ds의 마지막 값을 복사
            fp.yaw(MAX_POINTS) = fp.yaw(MAX_POINTS-1);   % 마지막 yaw 값 복사
            fp.ds(MAX_POINTS) = fp.ds(MAX_POINTS-1);     % 마지막 ds 값 복사

            % 곡률(kappa) 계산
            for i = 1:length(fp.yaw) - 1
                yaw_diff = fp.yaw(i + 1) - fp.yaw(i);
                yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff));
                fp.kappa(i) = yaw_diff / fp.ds(i);    % 곡률 계산
            end

            % Store the Frenet path in the array
            frenet_paths(path_count) = fp;
            path_count = path_count + 1;
        end
    end
    
    % ---- Check Path Validity ----
    for i = 1:length(frenet_paths)
        fp = frenet_paths(i);
        acc_squared = abs(fp.s_dd .^ 2 + fp.d_dd .^ 2);
        
        if any(fp.s_d > V_MAX)  % Max speed check
            continue;
        elseif any(acc_squared > ACC_MAX ^ 2)  % Max acceleration check
            continue;
        elseif any(abs(fp.kappa) > K_MAX)  % Max curvature check
            continue;
        end
        
        % 유효한 경로를 valid_frenet_paths에 저장
        valid_paths(valid_count) = fp;
        valid_count = valid_count + 1;
    end
    
    % 유효한 경로까지만 반환
%     valid_paths = valid_paths(1:valid_count - 1);
    
    % ---- Preallocation ----
    max_valid_paths = length(valid_paths);  % valid_paths의 최대 크기


    % 경로 수만큼 opt_traj를 사전 할당
    opt_traj = repmat(initialize_frenet_path(MAX_POINTS), 1, max_valid_paths);
    
    opt_count = 0;  % 최적 경로를 저장할 인덱스
    
    min_cost = inf;  % 최소 비용을 무한대로 초기화
    
    % 유효한 경로 중 최소 비용 경로 찾기
    for i = 1:max_valid_paths
        if valid_paths(i).c_tot < min_cost
            min_cost = valid_paths(i).c_tot;
            opt_count = opt_count + 1;  % 최적 경로 카운터 증가
            opt_traj(opt_count) = valid_paths(i);  % 최적 경로 저장
        end
    end
    
    % 실제로 채워진 opt_traj만 반환
    opt_traj = opt_traj(1:opt_count);  % 사용한 크기만큼 자르기

   
    % ---- Extract Global Coordinate Values ----
    if ~isempty(opt_traj)
        % ---- Extract Global Coordinate Values ----
        max_length = max(arrayfun(@(p) length(p.x), opt_traj));
        
        % 고정 크기의 NaN 배열 생성 (각 경로마다 max_length에 맞춘다)
%         length(valid_paths)
        x_value = NaN(length(opt_traj), max_length);
        y_value = NaN(length(opt_traj), max_length);
        yaw_value = NaN(length(opt_traj), max_length);
        ds_value = NaN(length(opt_traj), max_length);
        kappa_value = NaN(length(opt_traj), max_length);
        s_d_value = NaN(length(opt_traj), max_length);
        
        path_len = length(opt_traj(1).x);
        
        % Global 좌표계 값들
        x_value(1, 1:path_len) = opt_traj(1).x;
        y_value(1, 1:path_len) = opt_traj(1).y;
        yaw_value(1, 1:path_len) = opt_traj(1).yaw;
        ds_value(1, 1:path_len) = opt_traj(1).ds;
        kappa_value(1, 1:path_len) = opt_traj(1).kappa;
        s_d_value(1, 1:path_len) = opt_traj(1).s_d;  % s_d 값 채우기
        
    else
        % 유효한 경로가 없는 경우 빈 배열 반환
        x_value = [];
        y_value = [];
        yaw_value = [];
        ds_value = [];
        kappa_value = [];
        s_d_value = [];  % s_d 값 반환
    end
end


% Helper function to initialize FrenetPath with fixed-size arrays
function fp = initialize_frenet_path(max_points)
    % Pre-allocate fixed-size arrays for Frenet path
    fp.t = zeros(1, max_points);
    fp.d = zeros(1, max_points);
    fp.d_d = zeros(1, max_points);
    fp.d_dd = zeros(1, max_points);
    fp.d_ddd = zeros(1, max_points);
    fp.s = zeros(1, max_points);
    fp.s_d = zeros(1, max_points);
    fp.s_dd = zeros(1, max_points);
    fp.s_ddd = zeros(1, max_points);
    fp.c_lat = 0;
    fp.c_lon = 0;
    fp.c_tot = 0;
    fp.x = zeros(1, max_points);
    fp.y = zeros(1, max_points);
    fp.yaw = zeros(1, max_points);
    fp.ds = zeros(1, max_points);
    fp.kappa = zeros(1, max_points);
end


function quintic = QuinticPolynomial(xi, vi, ai, xf, vf, af, T)
    % Quintic Polynomial coefficients
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;

    A = [T^3, T^4, T^5;
         3*T^2, 4*T^3, 5*T^4;
         6*T, 12*T^2, 20*T^3];

    b = [xf - a0 - a1*T - a2*T^2;
         vf - a1 - 2*a2*T;
         af - 2*a2];

    x = A\b;  % Solve linear system

    a3 = x(1);
    a4 = x(2);
    a5 = x(3);

    % Return handles for functions
    quintic.calc_pos = @(t) (a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4);
    quintic.calc_vel = @(t) (a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3);
    quintic.calc_acc = @(t) (2*a2 + 6*a3*t + 12*a4*t^2);
    quintic.calc_jerk = @(t) (6*a3 + 24*a4*t);
end

function quartic = QuarticPolynomial(xi, vi, ai, vf, af, T)
    % Quartic Polynomial coefficients
    a0 = xi;
    a1 = vi;
    a2 = 0.5 * ai;

    A = [3*T^2, 4*T^3;
         6*T, 12*T^2];

    b = [vf - a1 - 2*a2*T;
         af - 2*a2];

    x = A\b;  % Solve linear system

    a3 = x(1);
    a4 = x(2);

    % Return handles for functions
    quartic.calc_pos = @(t) (a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4);
    quartic.calc_vel = @(t) (a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3);
    quartic.calc_acc = @(t) (2*a2 + 6*a3*t + 12*a4*t^2);
    quartic.calc_jerk = @(t) (6*a3 + 24*a4*t);
end

function [x,y,heading]  = get_cartesian(s, d, mapx, mapy, maps)
    prev_wp = 1;

    % s를 maps 길이 내에서 모듈로 연산
    s = mod(s, maps(end-1));

    % s보다 작은 prev_wp 찾기
    while (s > maps(prev_wp + 1)) && (prev_wp < length(maps) - 1)
        prev_wp = prev_wp + 1;
    end

    % 다음 waypoint 찾기
    next_wp = mod(prev_wp + 1, length(mapx));

    dx = mapx(next_wp) - mapx(prev_wp);
    dy = mapy(next_wp) - mapy(prev_wp);

    % 방향(heading) 계산 [rad]
    heading = atan2(dy, dx);

    % s를 따라 segment에서 x, y 좌표 계산
    seg_s = s - maps(prev_wp);

    seg_x = mapx(prev_wp) + seg_s * cos(heading);
    seg_y = mapy(prev_wp) + seg_s * sin(heading);

    % 수직 방향으로 변환 (90도)
    perp_heading = heading + pi / 2;
    x = seg_x + d * cos(perp_heading);
    y = seg_y + d * sin(perp_heading);
end




