function plot_path(G, goal_node_id, start, goal)
    % RRT*로 탐색된 경로를 그리는 함수
    
    % 시작점과 목표점 그리기
    plot(start(1), start(2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');  % 시작점
    plot(goal(1), goal(2), 'rx', 'MarkerSize', 10, 'LineWidth', 2);  % 목표점
    
    % 트리 그리기 (RRT* 탐색 트리)
    for i = 2:size(G.nodes, 1)
        parent_id = G.parents(i);
        plot([G.nodes(parent_id, 1), G.nodes(i, 1)], ...
             [G.nodes(parent_id, 2), G.nodes(i, 2)], 'k-', 'LineWidth', 0.5);  % 트리는 얇은 검은색 선으로
    end
    
    % 목표에 도달한 경로를 강조해서 그리기
    if goal_node_id ~= -1
        current_id = goal_node_id;
        while current_id ~= -1
            parent_id = G.parents(current_id);
            if parent_id == -1, break; end
            plot([G.nodes(parent_id, 1), G.nodes(current_id, 1)], ...
                 [G.nodes(parent_id, 2), G.nodes(current_id, 2)], 'g-', 'LineWidth', 2);  % 경로는 두꺼운 빨간색 선으로
            current_id = parent_id;
        end
    end
    
    axis equal;
    grid on;
    title('RRT* Path from Start to Goal');
end
