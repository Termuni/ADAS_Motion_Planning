function yaw_local = glob_2_loc_yaw(yaw_global, yaw_ego)
    % 글로벌 좌표계의 yaw 값을 로컬 좌표계로 변환
    % yaw_global: 변환하고자 하는 객체의 글로벌 yaw 값 (rad)
    % yaw_ego: 자기 차량의 글로벌 yaw 값 (rad)

    % yaw 값 변환 (글로벌 yaw에서 차량의 yaw를 빼줌)
    yaw_local = yaw_global - yaw_ego;
    % yaw 값은 -pi ~ pi 범위로 정규화
    yaw_local = wrapToPi(yaw_local);
%     yaw = -pi + atan2(y1-y2, x1-x2);
end
