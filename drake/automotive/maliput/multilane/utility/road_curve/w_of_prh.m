function [v] = w_of_prh(p, r, h)
  v = zeros(length(p), 3);
  [roll, pitch, yaw] =  rabg_of_p(p);
  xyz = xyz_of_p(p);
  for ii=1:length(p)
      v(ii,:) = xyz(ii,:) + [0 r h] * angle2dcm(yaw(ii), pitch(ii), roll(ii));
  end