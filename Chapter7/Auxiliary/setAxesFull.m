function setAxesFull(figHandle, leftMargin, rightMargin, bottomMargin, topMargin, verticalSpacing)
    % fullScreenSubplotsWithMarginsAndSpacing - 调整子图布局，支持设置上下左右留白和图间间距
    %
    % 参数：
    % figHandle       - figure 的句柄
    % leftMargin      - 左边的留白（相对于 figure 宽度，0 到 1 之间）
    % rightMargin     - 右边的留白（相对于 figure 宽度，0 到 1 之间）
    % topMargin       - 上边的留白（相对于 figure 高度，0 到 1 之间）
    % bottomMargin    - 底部的留白（相对于 figure 高度，0 到 1 之间）
    % verticalSpacing - 子图之间的间距（相对于 figure 高度，0 到 1 之间）
    
    if nargin < 1
        figHandle = gcf; % 默认使用当前 figure
    end
    if nargin < 2
        leftMargin = 0.1; % 默认左边留白
    end
    if nargin < 3
        rightMargin = 0.1; % 默认右边留白
    end
    if nargin < 4
        topMargin = 0.1; % 默认上边留白
    end
    if nargin < 5
        bottomMargin = 0.1; % 默认底部留白
    end
    if nargin < 6
        verticalSpacing = 0.05; % 默认子图之间的间距
    end
    
    % 获取当前 figure 的子图
    axHandles = findall(figHandle, 'Type', 'axes');
    
    % 按行从上到下排序
    axHandles = flipud(axHandles);
    
    % 子图总数
    numSubplots = numel(axHandles);
    
    % 计算每个子图的宽度和高度
    totalVerticalSpace = topMargin + bottomMargin + (numSubplots - 1) * verticalSpacing;
    subplotHeight = (1 - totalVerticalSpace) / numSubplots;
    subplotWidth = 1 - leftMargin - rightMargin;
    
    % 设置每个子图的位置
    for i = 1:numSubplots
        % 计算新的位置
        newPosition = [leftMargin, ...
                       bottomMargin + (numSubplots - i) * (subplotHeight + verticalSpacing), ...
                       subplotWidth, ...
                       subplotHeight];
        set(axHandles(i), 'Position', newPosition);
    end
end
