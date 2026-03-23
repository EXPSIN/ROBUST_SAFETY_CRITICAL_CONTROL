%{
%   Drawing a set with a cone constraint:
%      S := {x\in R^{2*1}: Ax+c_\delta|x| <= b} , with
%      A \in R^{n*2}, each row of A is unit vector, b \in R^{n\times 1}, 
%      and c_\delta \in [0, 1).
%   Note: the colored region is the feasible set.
%   written by Wu Si, email: wusixstx@163.com
%}
function H = cone_constraint_2D(H, A, b, c_delta, bias, axis_range, clr, displayname)
H.p_limit = axis_range;       % 图形边界
H.clr     = clr;              % 可行域颜色
H.name    = displayname;      %
H.bias    = bias;

if(~isequal(H.name, ''))
    H.name_visibility = 'on';
else
    H.name_visibility = 'off';
end


% if(~all(abs(vecnorm(A, 2, 2)-1) <= 1e-10)  )
%     error('Each row of A should be an unit vector.');
% elseif(c_delta < 0 || c_delta > 1)
%     error('c_delta should in [0, 1].');
% end

count_constraints = size(A, 1);

% Add the corner points.
cor = [axis_range(1), axis_range(3); axis_range(1), axis_range(4); 
       axis_range(2), axis_range(3); axis_range(2), axis_range(4)]';
x = cor(:, cone(cor(1,:), cor(2,:), A, b, c_delta) < 1e-10);

% Draw the implict function
for i = 1:count_constraints
    try
        delete(H.fp_line{i});
    catch

    end
    H.fp{i} = fimplicit(@(x,y)cone(x, y, A(i,:), b(i), c_delta), ...
                     axis_range, 'Color', H.clr);
    mask = cone(H.fp{i}.XData, H.fp{i}.YData, A, b, c_delta) < 1e-10;
    x    = [[H.fp{i}.XData(mask); H.fp{i}.YData(mask)], x];
    H.fp_line{i} = plot(H.fp{i}.XData + H.bias(1), H.fp{i}.YData + H.bias(2), 'Color', H.clr, 'HandleVisibility', 'off');
    delete(H.fp{i});
end

% Obtain convex hull.
try
    k = convhull(x(1, :),x(2, :));
    x = x(:, k);
catch
    x    = zeros(2, 1);
    fprintf('[cone_constraint_implict] empty polyhedron \n');
end

% Draw the feasible set.
try
    delete(H.handle_space);
catch

end
H.handle_space = patch(x(1, :)+H.bias(1), x(2, :)+H.bias(2), H.clr(:), 'facecolor', H.clr(:), 'facealpha', 0.3, 'displayname', H.name, 'HandleVisibility', H.name_visibility, 'EdgeColor', H.clr(:));

end

function res = cone(x, y, A, b, c_delta)
res    = zeros(size(x));
[m, n] = size(x);
for i = 1:m
    for j = 1:n
        vec      = [x(i,j);y(i,j)];
        res(i,j) = max(A*vec+c_delta*norm(vec) - b);
    end
end
end 