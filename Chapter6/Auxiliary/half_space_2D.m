function H = half_space_2D(H, A, b, varargin)
if(isempty(H))
    H.p_limit   = varargin{1};
    H.P         = [H.p_limit(2) H.p_limit(1) H.p_limit(1) H.p_limit(2); H.p_limit(4) H.p_limit(4) H.p_limit(3) H.p_limit(3)];
    H.clr       = varargin{2};
    H.mode      = varargin{3};
    H.name      = varargin{4};
    if(~isequal(H.name, ''))
        H.name_visibility = 'on';
    else
        H.name_visibility = 'off';
    end
    H.bias      = varargin{5};
    H.n         = size(A, 1);
    v = linspace(min(H.p_limit), max(H.p_limit), 200);
    [H.x, H.y] = meshgrid(v);
    if(isequal(H.mode, 'condition'))
        for i = 1:H.n
            [x, ~] = calcu_halfspace(A(i, :), b(i, :), H.P);
            if(isequal(size(H.clr), [1, 3]))
                H.handle_space(i) = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(:), 'facecolor', H.clr(:), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
            else
                H.handle_space(i) = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(i), 'facecolor', H.clr(i), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
            end

        end
    elseif(isequal(H.mode, 'domain'))
        if(H.n == 0)
            A = [1,0; 0, 1; -1, 0; 0, -1];
            b = [H.p_limit(2), H.p_limit(4), H.p_limit(1), H.p_limit(3)]'*2;
            H.handle_space = patch([H.p_limit(1) H.p_limit(1) H.p_limit(2), H.p_limit(2) H.p_limit(1)], ....
                [ H.p_limit(4) H.p_limit(3) H.p_limit(3) H.p_limit(4) H.p_limit(4)], H.clr(1), 'facecolor', H.clr(1), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
        else
            [x, ~] = calcu_polyhedron(A, b, H.P);
            if(isequal(size(H.clr), [1, 3]))
                hold on;
                H.handle_space = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(:), 'facecolor', H.clr(:), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
            else
                hold on;
                H.handle_space = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(1), 'facecolor', H.clr(1), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility);
            end

        end


    else
        error('Error mode, ''condition'' or ''domain''. ');
    end
    axis(H.p_limit);
else
    if(isequal(H.mode, 'condition'))

        for i = 1:H.n
            [x, ~] = calcu_halfspace(A(i, :), b(i, :), H.P);
            H.handle_space(i).XData = x(1,:);
            H.handle_space(i).YData = x(2,:);
        end
    elseif(isequal(H.mode, 'domain'))
        H.bias      = varargin{5};
        if(size(A, 1) ~= 0)
            [x, ~] = calcu_polyhedron(A, b, H.P);
            H.handle_space.XData = x(1,:)+H.bias(1);
            H.handle_space.YData = x(2,:)+H.bias(2);
        else
            H.handle_space.XData = [H.p_limit(1) H.p_limit(1) H.p_limit(2), H.p_limit(2) H.p_limit(1)];
            H.handle_space.YData = [ H.p_limit(4) H.p_limit(3) H.p_limit(3) H.p_limit(4) H.p_limit(4)];
        end

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
