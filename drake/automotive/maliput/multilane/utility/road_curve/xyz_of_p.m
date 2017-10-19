function [x] =  xyz_of_p(p)
    x = ones(length(p), 1) * [10, 10, 0];
    z = (pi/2) .* elevation_of_p(p);
    x = x + 10 .* [cos(pi/4 + pi/2 .* p') sin(pi/4 +  pi/2 .* p') z'];