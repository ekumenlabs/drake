function [z] =  elevation_of_p(p)
    scale = 10 * pi / 2;
    z = (10 / scale) + (15 / scale) .* p;