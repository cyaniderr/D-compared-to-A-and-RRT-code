function plotAnimation2(LPAbase, pathCoords)
    

    x = pathCoords(:, 1);
    y = pathCoords(:, 2);

    
    newObstColor = 'k';

    h1 = plot(x(1), y(1), 'bsquare');
  
    pause(0.25)
    for i = 2:numel(x)

        set(h1, 'XData', x(i))
        set(h1, 'YData', y(i))

       

        % % New Obstacles
        % new (dynamic) obstacles
        if isfield(LPAbase, 'NewObsts')

            for iNewObst = 1:LPAbase.NewObsts.count

                if LPAbase.NewObsts.t(iNewObst) == i
                    plot(LPAbase.NewObsts.x(iNewObst), LPAbase.NewObsts.y(iNewObst), 'o', 'MarkerSize', 5, ...
                        'MarkerFaceColor', newObstColor, 'MarkerEdgeColor', newObstColor);
                end

            end

        end

        drawnow
        % pause(0.2) uncomment for animation

 
    end

end
