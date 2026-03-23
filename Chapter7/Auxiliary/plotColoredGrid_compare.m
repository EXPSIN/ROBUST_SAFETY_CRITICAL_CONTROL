function [h, h_colorbar] = plotColoredGrid_compare(X, Y, values, clr, grid_size, face_alpha, enable_colorbar)
    numRows = size(values, 1);
    numCols = size(values, 2);

    % values(values==-1) = nan;   % 不可行
    values(values==-2) = nan;   % 非安全集合

    h = surf(X, Y, values, 'LineStyle', 'none'); % 使用imagesc绘制颜色图
    h_colorbar = colorbar; % 添加颜色条
    axis equal; % 设置坐标轴比例
    set(gca, 'YDir', 'normal'); % 设置Y轴方向正常
    
    % 可选: 设置NaN值的显示方式
    colormap('sky'); % 使用jet颜色映射
    % 
    % 
    % % 整理栅格的顶点坐标和颜色数据
    % X = [X ,X(:, end)+grid_size(1)];
    % X = [X; X(end, :)] - grid_size(1)/2;
    % Y = [Y, Y(:, end)];
    % Y = [Y; Y(end, :)+grid_size(2)] - grid_size(2)/2;
    % vertices = [X(:), Y(:)];
    % 
    % % plot(X(:), Y(:), 'r.', 'LineStyle', 'none'); hold on;
    % % plot(X(:), Y(:), 'ko', 'LineStyle', 'none');
    % clr_3d(1, 1, 1:3) = 1-clr;
    % % faceColors = ones(numRows, numCols, 3) .* sign(values) .* values.^2 .* clr_3d;
    % % faceColors = ones(numRows, numCols, 3) .* values.^3 .* clr_3d;
    % faceColors = ones(numRows, numCols, 3) .* values .* clr_3d;
    % % faceColors = ones(numRows, numCols, 3);
    % % for i = 1:numRows
    % %     for j = 1:numCols
    % %         if(values(i, j) == -1)
    % %             faceColors(i, j, :) = [1,1,1];
    % %         else
    % %             faceColors(i, j, :) = values(i,j) .* clr;
    % %         end
    % %     end
    % % end
    % 
    % % 创建栅格的连接信息
    % faceColors  = reshape(faceColors, [], 3);
    % % faceColors  = 1-faceColors;
    % 
    % numFaces    = numCols * numRows;
    % faces       = zeros(numFaces, 4);    
    % [row, col]  = ind2sub([numRows, numCols], 1:numFaces);
    % faces(:, 1) = sub2ind([numRows+1, numCols+1],   row,   col);
    % faces(:, 2) = sub2ind([numRows+1, numCols+1], row+1,   col);
    % faces(:, 3) = sub2ind([numRows+1, numCols+1], row+1, col+1);
    % faces(:, 4) = sub2ind([numRows+1, numCols+1],   row, col+1);
    % 
    % % 绘制整个平面栅格
    % indices = ~all(faceColors == [0,0,1], 2); % 排除部分栅格
    % line_clr = faceColors(find(indices == 1, 1), :);
    % if(isempty(line_clr))
    %     line_clr = [1,1,1];
    % end
    % 
    % h = patch('Vertices', vertices, ...
    %     'Faces', faces(indices, :), ...
    %     'FaceVertexCData', faceColors(indices, :), ...
    %     'FaceColor', 'flat', ...
    %     ... 'EdgeColor', line_clr, ...
    %     'linestyle', 'none', ...
    %     'facealpha', face_alpha);
    % if(enable_colorbar)
    %     colormap(1-linspace(0,1,100)'.*(1-clr));
    %     % colormap('sky');
    %     colorbar;
    % end
    % 
    % % h = patch('Vertices', vertices, ...
    % %     'Faces', faces, ...
    % %     'FaceVertexCData', faceColors, ...
    % %     'FaceColor','flat', ...
    % %     ... 'EdgeColor', line_clr, ...
    % %     'linestyle', 'none', ...
    % %     'facealpha', face_alpha);
end
