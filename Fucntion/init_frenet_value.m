function [si, si_d, si_dd, sf_d, sf_dd, di, di_d, di_dd,df_d,df_dd, opt_di] = init_frenet_value(s, d, target_speed, yaw_road, local_yaw, v, a)
    
    yawi = local_yaw - yaw_road;  % 로컬 좌표계에서의 yaw 차이

    % s 방향 초기조건
    si = s;
    si_d = v * cos(yawi);  % 종방향 속도 (로컬 yaw 차이 반영)
    si_dd = a * cos(yawi);  % 종방향 가속도
    sf_d = target_speed;  % 목표 속도
    sf_dd = 0;  % 목표 가속도

    % d 방향 초기조건
    di = d;
    di_d = v * sin(yawi);  % 횡방향 속도
    di_dd = a * sin(yawi);  % 횡방향 가속도
    df_d = 0;  % 목표 횡방향 속도
    df_dd = 0;  % 목표 횡방향 가속도

    opt_di = di;  % 최적 횡방향 위치
end
