function H = half_plane(H, A, b, bias, varargin)
if(isempty(H))
    H.p_limit   = varargin{1};
    H.P         = [H.p_limit(2) H.p_limit(1) H.p_limit(1) H.p_limit(2); H.p_limit(4) H.p_limit(4) H.p_limit(3) H.p_limit(3)]*2;
    H.clr       = varargin{2};
    H.name      = varargin{3};
    H.bias      = bias;

    if(~isequal(H.name, ''))
        H.name_visibility = 'on';
    else
        H.name_visibility = 'off';
    end

    [x, ~] = calcu_polyhedron(A, b, H.P);
    if(isequal(size(H.clr), [1, 3]))
        hold on;
        H.handle_space = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(:), 'facecolor', H.clr(:), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
    else
        hold on;
        H.handle_space = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(1), 'facecolor', H.clr(1), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
    end

else
    H.bias      = bias;
    [x, ~] = calcu_polyhedron(A, b, H.P);
    H.handle_space.XData = x(1,:)+H.bias(1);
    H.handle_space.YData = x(2,:)+H.bias(2);
end
end


function [x, flag] = calcu_polyhedron(A, b, P)
flag = true;
m = size(A, 1);
x = [];

for k = 1:m
    [~, mall] = calcu_halfspace(A(k, :), b(k, :), P);
    x = [x, mall];
end
for k = 1:m
    for j = (k+1):m
        if(abs(det([A(k, :); A(j, :)])) > 1e-6)
            x = [x, [A(k, :); A(j, :)]\[b(k);b(j)] ];
        end
    end
end

x_right  = x(:, all(A*x-b <= 1e-6, 1));
x_right  = unique(x_right', 'rows')';

try
    k = convhull(x_right(1, :),x_right(2, :));
    x = x_right(:, k);
catch
    x    = zeros(2, 1);
    flag = false;
    fprintf('[half_space_2D] empty polyhedron \n');
end
end

function [x, xall] = calcu_halfspace(A, b, P)
b  = b/norm(A);
A  = A/norm(A);

nP = size(P, 2);
A_P = [ones(nP, 1), zeros(nP, 1); zeros(nP, 1), ones(nP, 1)];
b_P = [P(1, :), P(2, :)]';

xall = P;
for k = 1:size(A_P, 1)
    if(abs(det([A_P(k, :); A])) > 1e-6)
        xall = [xall, [A_P(k, :); A]\[b_P(k);b]];
    end
end

xall  = xall(:, A*xall-b <= 1e-6);

try
    k = convhull(xall(1, :),xall(2, :));
    x = xall(:, k);
catch
    x    = zeros(2, 1);
    fprintf('[half_plane] empty\n');
end

end
