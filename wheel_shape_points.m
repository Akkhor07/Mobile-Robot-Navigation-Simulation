function [wx, wy] = wheel_shape_points(pose, L, W)
    wl = 0.09; ww = 0.045;
    offsets = [ L/2-wl/2,  W/2+ww/2; ...
                L/2-wl/2, -W/2-ww/2; ...
               -L/2+wl/2,  W/2+ww/2; ...
               -L/2+wl/2, -W/2-ww/2];
    corners = [-wl/2 -ww/2; wl/2 -ww/2; wl/2 ww/2; -wl/2 ww/2]';
    R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
    wx = cell(1,4); wy = cell(1,4);
    for i = 1:4
        c = R*corners + R*offsets(i,:)';
        wx{i} = c(1,:) + pose(1);
        wy{i} = c(2,:) + pose(2);
    end
end
