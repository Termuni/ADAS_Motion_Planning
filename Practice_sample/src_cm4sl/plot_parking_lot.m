function plot_parking_lot(space, obstacles)
    % 주차장 경계와 장애물을 그리는 함수
    
    % 주차장 경계선 그리기
    rectangle('Position', [space(1), space(3), space(2) - space(1), space(4) - space(3)], 'EdgeColor', 'r', 'LineWidth', 2);
    
    % 주차장 내부의 점 그리기 (점으로 주차장 영역 표현)
    plot(5.5, -40:44.4, 'r.', 'MarkerSize', 10);  % 주차장의 입구를 빨간 점으로 표현
    
    % 장애물 그리기
    for i = 1:length(obstacles)
        plot_obstacle(obstacles(i));
    end
    
    % 축 설정
    xlabel('X Position');
    ylabel('Y Position');
    title('Parking Lot with Obstacles');
    axis equal;
    grid on;
end

function plot_obstacle(obstacle)
    % 개별 장애물을 그리는 함수 (타원 형태)
    theta = linspace(0, 2*pi, 100);
    x = obstacle.x + obstacle.r * cos(theta);
    y = obstacle.y + obstacle.r * sin(theta);
    plot(x, y, 'k-', 'LineWidth', 1.5);  % 장애물을 검은색 타원으로 표시
end
