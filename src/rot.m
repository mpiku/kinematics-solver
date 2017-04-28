function R = rot( fi )
    % Returns 2-D rotation matrix.
    % Input:
    %  * fi - angle in rads
    R = [cos(fi) -sin(fi); sin(fi) cos(fi)];
end

