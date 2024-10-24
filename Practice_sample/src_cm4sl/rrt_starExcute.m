% 주차장 및 장애물 정보 가져오기
[start, goal, space, obstacles] = parking_map();  % 주차장 및 장애물 생성

% RRT* 설정 값 정의
config.eta = 5.0;  % 최대 확장 거리 (노드 간 거리)
config.gamma_rrt_star = 10;  % 근처 노드 탐색 거리 비율
config.goal_sample_rate = 0.05;  % 목표 지점을 샘플링할 확률
config.goal_range = 2.0;  % 목표 지점 근처에 도달했다고 판단하는 거리

% 장애물과 주차장 경계 시각화
figure;
hold on;
plot_parking_lot(space, obstacles);  % 주차장 및 장애물 그리기

% RRT* 알고리즘 실행
[G, goal_node_id] = rrt_star(start, goal, space, obstacles, config);

% 결과 시각화
plot_path(G, goal_node_id, start, goal);  % 경로 시각화
hold off;

path = [];
current_id = goal_node_id;

while current_id ~= -1
    path = [G.nodes(current_id, :); path];  % 경로에 현재 노드를 추가
    current_id = G.parents(current_id);    % 부모 노드로 이동
end
