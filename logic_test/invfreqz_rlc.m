function [b, a] = invfreqz_rlc(Hd, w)
    % Hd: 目标频响（复数向量）
    % w:  频率点（单位为 rad/sample）

    N = length(w);
    z = exp(1j * w(:));  % z = e^(jw)

    A = zeros(N, 7);     % 4个b系数 + 3个a系数
    y = Hd(:);

    for k = 1:N
        zk = z(k);
        A(k,1) = 1;                     % b0
        A(k,2) = 1/zk;                  % b1
        A(k,3) = 1/zk^2;                % b2
        A(k,4) = 1/zk^3;                % b3
        A(k,5) = -y(k)/zk;              % -a1 * H
        A(k,6) = -y(k)/zk^2;            % -a2 * H
        A(k,7) = -y(k)/zk^3;            % -a3 * H
    end

    x = A \ y;

    b = real(x(1:4)).';                % 分子系数 b0~b3
    a = [1, real(x(5:7)).'];           % 分母系数 a0=1, a1~a3
end
