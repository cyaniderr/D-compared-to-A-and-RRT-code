function plotSolution(pathCoords)
    % Plot Path and mPath on the Map

    x = pathCoords(:, 1);
    y = pathCoords(:, 2);

    % path
    plot(x, y, 'b', 'LineWidth', 2);
    plot(x(2:end - 1), y(2:end - 1),  'b', 'LineWidth', 2);

   

end
