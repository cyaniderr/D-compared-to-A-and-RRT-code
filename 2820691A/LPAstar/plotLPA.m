function plotLPA(LPAbase, showDynamicObst)
    % Plot Map and Rs start and target nodes

    if nargin < 2
        showDynamicObst = false;
    end

    newObstColor = 'k';

    xMin = LPAbase.Map.xMin;
    xMax = LPAbase.Map.xMax;
    yMin = LPAbase.Map.yMin;
    yMax = LPAbase.Map.yMax;
    obstX = LPAbase.Obsts.xaxis;
    obstY = LPAbase.Obsts.yaxis;
    xs = LPAbase.R.xstart;
    ys = LPAbase.R.ystart;
    xt = LPAbase.R.xtarget;
    yt = LPAbase.R.ytarget;

    figure(1)
    grid on;
    axis equal
    axis([xMin - 1, xMax + 1, yMin - 1, yMax + 1])
    box on
    hold on

    % g=gca;
    % g.XTick=0:xMax;
    % g.YTick=0:yMax;

    % start
    plot(xs, ys, 'g*') 
    % target
    plot(xt, yt, 'b*')
  

    % Obstacles
    plot(obstX, obstY, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');

    % new (dynamic) obstacles
    if showDynamicObst && isfield(LPAbase, 'NewObsts') && LPAbase.NewObsts.count > 0
        plot(LPAbase.NewObsts.x, LPAbase.NewObsts.y, 'o', 'MarkerSize', 5, ...
            'MarkerFaceColor', newObstColor, 'MarkerEdgeColor', newObstColor);
    end

   

    % walls
    rectangle('Position', [xMin - 0.5 yMin - 0.5 (xMax - xMin + 1) (yMax - yMin + 1)], 'LineWidth', 5)
    xlabel('Y(meters)'); ylabel('X(meters)');

end
