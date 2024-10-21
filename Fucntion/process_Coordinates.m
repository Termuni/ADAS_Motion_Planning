function out = process_Coordinates(coordinates)
    % coordinates 배열에서 x와 y 좌표를 분리하여 처리
    out = zeros(length(coordinates)/2,2);
    out(:,1) = coordinates(1:length(coordinates)/2);
    out(:,2) = coordinates(length(coordinates)/2+1:length(coordinates));
end
