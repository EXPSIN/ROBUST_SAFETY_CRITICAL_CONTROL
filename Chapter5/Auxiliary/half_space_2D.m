function H = half_space_2D(H, A, b, varargin)
if(isempty(H))
    H.p_limit   = varargin{1};
    H.P         = [H.p_limit(2) H.p_limit(1) H.p_limit(1) H.p_limit(2); H.p_limit(4) H.p_limit(4) H.p_limit(3) H.p_limit(3)];
    H.clr       = varargin{2};
    H.mode      = varargin{3};
    try
        H.name      = varargin{4};
    catch
        H.name      = 'domain';
    end
    H.n         = size(A, 1);
    v = linspace(min(H.p_limit), max(H.p_limit), 200);
    [H.x, H.y] = meshgrid(v);
    if(isequal(H.mode, 'condition'))
        for i = 1:H.n
            [x, ~] = calcu_halfspace(A(i, :), b(i, :), H.P);
            H.handle_space(i) = patch(x(1, :), x(2, :), H.clr(i), 'facecolor', H.clr(i), 'facealpha', 0.5, 'displayname', H.name);
        end
    elseif(isequal(H.mode, 'domain'))
        [x, ~] = calcu_polyhedron(A, b, H.P);
        H.handle_space = patch(x(1, :), x(2, :), H.clr(1), 'facecolor', H.clr(1), 'facealpha', 0.5, 'displayname', H.name);
    else
        error('Error mode, ''condition'' or ''domain''. ');
    end

    H.handle_bound = plot(H.P(1, mod(0:4,4)+1), H.P(2, mod(0:4,4)+1), 'k.--', 'HandleVisibility', 'off');
    axis(H.p_limit);
else
    if(H.n ~= size(A, 1))
        error('Error size A');
    end

    if(isequal(H.mode, 'condition'))
        for i = 1:H.n
            [x, ~] = calcu_halfspace(A(i, :), b(i, :), H.P);
            H.handle_space(i).XData = x(1,:);
            H.handle_space(i).YData = x(2,:);
        end
    elseif(isequal(H.mode, 'domain'))
        [x, ~] = calcu_polyhedron(A, b, H.P);
        H.handle_space.XData = x(1,:);
        H.handle_space.YData = x(2,:);
    end
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
    fprintf('[half_space_2D] empty halfspace \n');
end

end
