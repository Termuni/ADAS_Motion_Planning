function [err, x_y,local_map] = get_lat_err(local_points, FLAG, all_lane, path_list, y_flag)
persistent target

if isempty(target)
    target = [0,0];
    i_set1 = 1:10;
    i_set2 = 11:20;
    i_set3 = 21:30;
    i_set4 = 31:33;
    path_lane = zeros(1,length(path_list));
    for i = path_list
        if ismember(i_set1,i)
            path_lane(i) = 1;
        end
        if ismember(i_set2,i)
            path_lane(i) = 2;
        end
        if ismember(i_set3,i)
            path_lane(i) = 3;
        end
        if ismember(i_set4,i)
            path_lane(i) = 4;
        end
    end
end

local_map = local_points;


% local_map 행렬에서 x가 양수인 점들만 추출
positive_x_points = local_map(local_map(:,1) > 0, :); 

closest_lane = 1;
second_closest_lane = 1;

temp = [0 0 0 0];
if y_flag == 1
    for i = 1:4
        size(positive_x_points)
        if ismember(positive_x_points, all_lane(:,i:2*i),'rows')
            closest_lane = i;
            temp(i) = i;
        end
        if ismember(positive_x_points, all_lane(:,i:2*i),'rows')
            second_closest_lane = i;
            temp(i) = i;
            break;
        end
    end

end



if closest_lane == second_closest_lane
    ;
else
    positive_x_points = positive_x_points(ismember(positive_x_points, all_lane(:,closest_lane:closest_lane*2),'rows'),:);

end

% 원점과의 거리 계산
distances = sqrt(sum(positive_x_points.^2, 2));

% 거리를 오름차순으로 정렬하고 인덱스 얻기
[sorted_distances, sorted_idx] = sort(distances);


if isempty(sorted_idx) || length(sorted_idx) < 4
    err = 0;
    x_y = [10000,10000];
    
    return;
end

% 2번째로 가까운 점과 네 번째로 가까운 점의 인덱스
if FLAG == 100
    err = 0;
    x_y = [10000,10000];
   
    return;
end

if FLAG == 10
    closest_idx = sorted_idx(2);
    second_closest_idx = sorted_idx(2);    
else
    closest_idx = sorted_idx(2);
    second_closest_idx = sorted_idx(4);

    
end
% 가장 가까운 점과 두 번째로 가까운 점의 좌표
closest_point = positive_x_points(closest_idx, :);
second_closest_point = positive_x_points(second_closest_idx, :);

% 두 점의 평균 좌표 계산
average_point = (closest_point + second_closest_point) / 2;



target = average_point;

err = -target(1,2) ;
x_y = target;

end