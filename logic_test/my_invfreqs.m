function [b_coeffs, a_coeffs] = my_invfreqs(H, w, nb)
% my_invfreqs_2nd_order_fixed 实现一个简单的2阶invfreqs（连续时间）。
%   [b_coeffs, a_coeffs] = my_invfreqs_2nd_order_fixed(H, w, nb)
%   恢复一个连续时间2阶传递函数 H(s) = N(s)/D(s) 的分子(b)和分母(a)系数，
%   使其最符合给定频率 w 下的复数频率响应 H。
%   本版本将分母最高阶系数 a_na 固定为1，与MATLAB标准invfreqs行为一致。
%
%   输入:
%     H        : 复数频率响应向量。
%     w        : 频率向量，单位为弧度/秒。
%     nb       : 分子多项式的阶数。分母阶数在本函数中固定为2 (na=2)。
%
%   输出:
%     b_coeffs : 分子系数向量，格式为 [b_nb, ..., b_1, b_0]。
%     a_coeffs : 分母系数向量，格式为 [a_2, a_1, a_0]，其中 a_2 固定为1。
%
%   传递函数形式为 H(s) = (b_nb*s^nb + ... + b_1*s + b_0) /
%                            (a_2*s^2 + a_1*s + a_0)
%   系数通过最小二乘法求解线性方程组来估计，假设 a_2 = 1。

    % 分母阶数固定为2
    na = 2;

    n_freq = length(w);

    % 未知系数的总数量 (不包括固定为1的 a_na)
    % 分子: (nb + 1) 个系数 (b_0 到 b_nb)
    % 分母: (na) 个系数 (a_0 和 a_1，因为 a_2=1)
    n_unknowns = (nb + 1) + na; 

    if n_freq < n_unknowns
        error('频率点不足。对于指定的分子阶数 (%d) 和2阶分母，至少需要 %d 个频率点。', n_unknowns, nb);
    end

    % 初始化用于线性方程组 Ax = B 的矩阵
    % x 向量将包含 [b_0, b_1, ..., b_nb, a_0, a_1] (在翻转之前)
    A = zeros(n_freq, n_unknowns);
    B = zeros(n_freq, 1);

    for k = 1:n_freq
        wk = w(k);
        Hk = H(k);
        jwk = 1j * wk; % s = j*omega

        % 构建矩阵 A 和向量 B 的行:
        % sum(b_j * s^j) - Hk * sum(a_i * s^i) = Hk * s^na  (i 从 0 到 na-1)
        % (其中 a_na = 1 的项被移到右侧)

        % 分子系数部分 (A 的第 1 列到第 nb+1 列): 用于 b_j 项
        % 这些对应于 x 求解后得到的 [b_0, b_1, ..., b_nb]
        for j_idx = 0:nb
            A(k, j_idx + 1) = jwk^j_idx; % s^0, s^1, ..., s^nb
        end

        % 分母系数部分 (A 的第 nb+2 列到第 n_unknowns 列): 用于 a_i 项
        % 这些对应于 x 求解后得到的 [a_0, a_1]
        % 注意：这里的 i_idx 对应的是 s^i 的系数，所以 i_idx=0 对应 a_0, i_idx=1 对应 a_1
        for i_idx = 0:(na-1) % 系数 a_0, a_1 (因为 a_2 已经固定为1)
            A(k, (nb + 1) + i_idx + 1) = -Hk * (jwk^i_idx); % -Hk*s^0, -Hk*s^1
        end

        % 方程的右侧： Hk * s^na (where s^na is (jwk)^na)
        B(k) = Hk * (jwk^na);
    end

    % 求解线性方程组，获取未知系数
    x = A \ B;

    % 从解向量 x 中提取 b 和 a 系数
    % b_coeffs_temp 包含 [b_0; b_1; ...; b_nb] (按升幂排列)
    b_coeffs_temp = x(1 : (nb + 1));
    
    % a_coeffs_remaining 包含 [a_0; a_1] (按升幂排列)
    a_coeffs_remaining = x((nb + 1) + 1 : end);
    
    % 构建完整的分母系数，并将其按升幂排列：[a_0, a_1, a_2=1]
    a_coeffs_ascending = [a_coeffs_remaining; 1]; 
    
    % 翻转系数以符合MATLAB的常规表示（降幂排列）
    b_coeffs = flipud(b_coeffs_temp); % 结果为 [b_nb; ...; b_1; b_0]
    a_coeffs = flipud(a_coeffs_ascending); % 结果为 [a_2; a_1; a_0]

end