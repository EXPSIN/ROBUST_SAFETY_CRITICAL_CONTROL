function [h, h_colorbar] = plotColoredGrid_compare(X, Y, values, clr, grid_size, face_alpha, enable_colorbar)
    numRows = size(values, 1);
    numCols = size(values, 2);

    values(values==-2) = nan;

    h = surf(X, Y, values, 'LineStyle', 'none');
    h_colorbar = colorbar;
    axis equal;
    set(gca, 'YDir', 'normal');

    colormap('sky');
end
