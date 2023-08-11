function feasible = newnode( x,y,dist,path_dist )

    feasible = 0;
    diff = dist/2;
    a = path_dist / 2;
    c = sqrt(2) * (95-2) / 2;
    if a>c
        b = sqrt(a*a - c*c);
    else
        fprintf("sMaj<sMin.....adjust c\n");
    end
 
    dist1 = (((x-diff)*cos(1/4*pi) + (y-diff)*sin(1/4*pi))^2) / (a^2) + ((-(x-diff)*sin(1/4*pi) + (y-diff)*cos(1/4*pi))^2 ) / (b^2);

    if dist1 <= 1
         feasible = 1;
    else 
         feasible = 0;
    end
end

