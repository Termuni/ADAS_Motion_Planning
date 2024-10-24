function [G, goal_node_id] = rrt_star(start, goal, space, obstacles, config)
    % RRT* 알고리즘 초기화 및 실행 함수
    G.nodes = start;  % 첫 노드는 시작점
    G.parents = -1;   % 첫 노드의 부모는 없음
    G.costs = 0;      % 시작점의 비용은 0
    
    max_iter = 1;  % 최대 반복 횟수
    goal_node_id = -1; % 목표 노드의 ID
    
    for i = 1:max_iter
        % 1. 랜덤 노드 생성
        rand_node = sample_free(space, config.goal_sample_rate, goal);
        
        % 2. 가장 가까운 노드 찾기
        nearest_node_id = get_nearest(G, rand_node);
        nearest_node = G.nodes(nearest_node_id, :);
        
        [a, b] = size(obstacles);
        disp(a);
        disp(b);

        for j = 1:length(obstacles)
            obs = obstacles(i);
            disp(obs);
        end

        % 3. 새로운 노드 생성
        new_node = steer(nearest_node, rand_node, config.eta);
        
        % 4. 주차장 경계 체크 및 충돌 체크
        if is_within_boundary(new_node, space) && is_collision_free(nearest_node, new_node, obstacles)
            % 5. 인근 노드 찾기
            near_node_ids = get_near_nodes(G, new_node, config);
            
            % 6. 새로운 노드를 트리에 추가
            G.nodes = [G.nodes; new_node];
            G.parents = [G.parents; nearest_node_id];
            G.costs = [G.costs; G.costs(nearest_node_id) + distance(nearest_node, new_node)];
            
            % 7. 비용 최소화 과정
            min_node_id = nearest_node_id;
            min_cost = G.costs(nearest_node_id) + distance(nearest_node, new_node);
            
            for j = 1:length(near_node_ids)
                near_node_id = near_node_ids(j);
                near_node = G.nodes(near_node_id, :);
                if is_collision_free(near_node, new_node, obstacles)
                    cost = G.costs(near_node_id) + distance(near_node, new_node);
                    if cost < min_cost
                        min_node_id = near_node_id;
                        min_cost = cost;
                    end
                end
            end
            
            G.parents(end) = min_node_id;
            G.costs(end) = min_cost;
            
            % 8. 트리 재연결
            for j = 1:length(near_node_ids)
                near_node_id = near_node_ids(j);
                near_node = G.nodes(near_node_id, :);
                if is_collision_free(new_node, near_node, obstacles)
                    cost = G.costs(end) + distance(new_node, near_node);
                    if cost < G.costs(near_node_id)
                        G.parents(near_node_id) = size(G.nodes, 1);
                        G.costs(near_node_id) = cost;
                    end
                end
            end
            
            % 9. 목표에 도달했는지 확인
            if distance(new_node, goal) < config.goal_range
                goal_node_id = size(G.nodes, 1);
                break;
            end
        end
    end
end

function result = is_within_boundary(node, space)
    % 주차장 경계 내에 있는지 확인
    result = (node(1) >= space(1) && node(1) <= space(2)) && (node(2) >= space(3) && node(2) <= space(4));
end


function rand_node = sample_free(space, goal_sample_rate, goal)
    if rand() < goal_sample_rate
        rand_node = goal;
    else
        rand_node = [rand_range(space(1), space(2)), rand_range(space(3), space(4))];
    end
end

function val = rand_range(min_val, max_val)
    val = (max_val - min_val) * rand() + min_val;
end

function nearest_node_id = get_nearest(G, rand_node)
    dists = vecnorm(G.nodes - rand_node, 2, 2);
    [~, nearest_node_id] = min(dists);
end

function new_node = steer(nearest_node, rand_node, eta)
    dir_vec = rand_node - nearest_node;
    dist = norm(dir_vec);
    if dist > eta
        dir_vec = dir_vec / dist * eta;
    end
    new_node = nearest_node + dir_vec;
end

function near_node_ids = get_near_nodes(G, new_node, config)
    n_nodes = size(G.nodes, 1);
    r = config.gamma_rrt_star * sqrt(log(n_nodes) / n_nodes);
    dists = vecnorm(G.nodes - new_node, 2, 2);
    near_node_ids = find(dists <= r);
end

function result = is_collision_free(node_from, node_to, obstacles)
    result = true;
    for i = 1:length(obstacles)
        obs = obstacles(i);
        if check_collision(node_from, node_to, obs)
            result = false;
            break;
        end
    end
end

function result = check_collision(node_from, node_to, obs)
    t = linspace(0, 1, 20);
    points = (1-t)' * node_from + t' * node_to;
    dists = vecnorm(points - [obs.x, obs.y], 2, 2);
    result = any(dists <= obs.r);
end

function d = distance(node1, node2)
    d = norm(node1 - node2);
end

