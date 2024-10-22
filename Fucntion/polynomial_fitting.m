function coeff = polynomial_fitting(num_degree, num_point, points)
    np = num_point; % 데이터 포인트의 개수
    nd = num_degree; % 다항식 차수
    A = zeros(np, nd+1); % A 행렬 생성
    B = zeros(np, 1); % B 벡터 생성
    coeff = zeros(nd+1, 1); % 계수 벡터 생성

    % 피팅을 위한 행렬 A와 벡터 B 계산
    for i = 1:np
        for j = 1:(nd + 1)
            A(i, j) = points(i, 1)^(nd - (j - 1)); % MATLAB에서는 1-based 인덱스
        end
        B(i) = points(i, 2); % 두 번째 열의 y값 저장
    end

    % 계수 계산 (최소자승법)
    coeff = (A' * A) \ (A' * B); % @는 함수 핸들이므로 '로 변경, '\'는 역행렬 구하는 방식
end
