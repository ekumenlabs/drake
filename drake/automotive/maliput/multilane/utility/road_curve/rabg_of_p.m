function [roll, pitch, yaw] = rabg_of_p(p)
    roll = zeros(length(p), 1);
    pitch = -atan(15 / (10 * pi/2)) .* ones(length(p), 1);
    yaw = (pi/4 + pi/2) + pi/2 .* p';