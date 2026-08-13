function setAxesFull(figHandle, leftMargin, rightMargin, bottomMargin, topMargin, verticalSpacing)

    if nargin < 1
        figHandle = gcf;
    end
    if nargin < 2
        leftMargin = 0.1;
    end
    if nargin < 3
        rightMargin = 0.1;
    end
    if nargin < 4
        topMargin = 0.1;
    end
    if nargin < 5
        bottomMargin = 0.1;
    end
    if nargin < 6
        verticalSpacing = 0.05;
    end

    axHandles = findall(figHandle, 'Type', 'axes');

    axHandles = flipud(axHandles);

    numSubplots = numel(axHandles);

    totalVerticalSpace = topMargin + bottomMargin + (numSubplots - 1) * verticalSpacing;
    subplotHeight = (1 - totalVerticalSpace) / numSubplots;
    subplotWidth = 1 - leftMargin - rightMargin;

    for i = 1:numSubplots
        newPosition = [leftMargin, ...
                       bottomMargin + (numSubplots - i) * (subplotHeight + verticalSpacing), ...
                       subplotWidth, ...
                       subplotHeight];
        set(axHandles(i), 'Position', newPosition);
    end
end
