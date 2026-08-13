function [r, l] = measurements_2D(p, R, env, D_B)

persistent N theta l_;

if isempty(l_)
    N     = 360;
    theta = linspace(0, 2*pi, N+1);
    theta = theta(1:end-1);

    l_ = [cos(theta); sin(theta)];
end

l = l_;
yaw = atan2(R(2, 1), R(1, 1));

l_w = [cos(theta + yaw); sin(theta + yaw)];

r = D_B * ones(1, N);

p_hw = ([0, -1; 1, 0] * (p - env.p_e)) * env.N;

[h_obs, w_obs] = size(env.obs);

is_p_hw_in_obs = is_in_rec(p_hw, [1; 1], [h_obs; w_obs]);

for i = 1:N
    l_pix = [-l_w(2, i); l_w(1, i)];

    p_check = check_range(p_hw, l_pix, [h_obs; w_obs], is_p_hw_in_obs, D_B*env.N);

    if isempty(p_check)
        r(i) = D_B;
        continue;
    else
        index = find(env.obs(p_check) == true);
        if isempty(index)
            r(i) = D_B;
        else
            [row, col] = ind2sub([h_obs, w_obs], p_check(index(1)));
            r(i) = min(norm(([row; col] - p_hw)) / env.N, D_B);
        end
    end
end

end

function flag = is_in_rec(p, range_min, range_max)
flag = all(p >= range_min & p <= range_max);
end

function p_check = check_range(p_hw, l_pix, size_hw, is_p_hw_in_obs, max_pixels_len)

A = [1, 0; 1, 0; 0, 1; 0, 1];
b = [1; size_hw(1); 1; size_hw(2)];

t = ((b - A * p_hw) ./ round(A * l_pix*10e10) * 10e10)';
p = round(p_hw + l_pix .* t);

mask = is_in_rec(p, [1; 1], size_hw);
mask = mask & t >= 0 & isfinite(t);
p    = p(:, mask);

[~, idx] = sort(vecnorm(p - p_hw));

if is_p_hw_in_obs
    start_hw = p_hw;
    end_hw   = p(:, idx(1));
elseif ~isempty(p)
    start_hw = p(:, idx(1));
    end_hw   = p(:, idx(end));
else
    p_check = zeros(1, 0);
    return;
end

if(norm(start_hw-p_hw) > max_pixels_len)
    p_check = zeros(1, 0);
    return;
end

start_hw = p_hw + (start_hw-p_hw) * min(1, max_pixels_len/norm(start_hw-p_hw));
end_hw   = p_hw + (  end_hw-p_hw) * min(1, max_pixels_len/norm(  end_hw-p_hw));
len = ceil(norm(end_hw - start_hw));
h = round(linspace(start_hw(1), end_hw(1), len));
w = round(linspace(start_hw(2), end_hw(2), len));


p_check = sub2ind(size_hw, h, w);
end
