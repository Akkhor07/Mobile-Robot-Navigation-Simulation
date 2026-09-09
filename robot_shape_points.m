function [bx, by] = robot_shape_points(pose, L, W)
    corners = [-L/2 -W/2; L/2 -W/2; L/2 W/2; -L/2 W/2]';
    R = [cos(pose(3)) -sin(pose(3)); sin(pose(3)) cos(pose(3))];
    rot = R*corners;
    bx = rot(1,:) + pose(1);
    by = rot(2,:) + pose(2);
end

