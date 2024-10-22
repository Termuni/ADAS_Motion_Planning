function points = polynomial_value(num_degree, num_point, coeff, x)
    np = num_point; % 데이터 포인트의 개수
    nd = num_degree; % 다항식 차수
    fn_x = zeros(1, nd+1); % A 행렬 생성
    fn_y = zeros(np, 1); % B 벡터 생성
    points = zeros(np, 2); % 계수 벡터 생성

    for i = 1:np
        for j = 1:(nd + 1)
            fn_x(1, j) = x(i, 1)^(nd - (j - 1)); % x의 값을 계산
        end

        fn_y(i, 1) = fn_x * coeff; % 계수와 x를 곱하여 y값을 계산
        points(i, 1) = points(i, 1); % points 배열의 x 값 그대로 사용
        points(i, 2) = fn_y(i, 1); % 계산된 y 값을 points 배열에 저장
    end


end
