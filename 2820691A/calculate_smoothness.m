function smoothness = calculate_smoothness(best_path)

    X = best_path(:, 1);
    Y = best_path(:, 2);

    dX = diff(X);
    dY = diff(Y);

    a = dX == 0 & dY == 0;
    dX(a) = [];
    dY(a) = [];
    X(a) = [];
    Y(a) = [];
    
    len = length(dX)-1;
    if numel(X) > 2
        sd = zeros(1, len);

        for i = 1:len
            x = sqrt(dX(i + 1) ^ 2 + dY(i + 1) ^ 2);
            y = sqrt(dX(i) ^ 2 + dY(i) ^ 2);
            z = (X(i) - X(i + 2)) ^ 2 + (Y(i) - Y(i + 2)) ^ 2;
            b = abs(acosd((x ^ 2 + y ^ 2 - z) / (2 * x * y)));
            sd(i) = 180 - b;
        end

        smooth = sum(sd);
        smoothness = round(smooth, 3);
    else
        smoothness = 0;
    end

end