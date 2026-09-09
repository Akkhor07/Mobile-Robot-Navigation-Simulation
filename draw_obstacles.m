function draw_obstacles(ax, obstacles)
    for i = 1:numel(obstacles)
        o = obstacles(i);
        switch o.type
            case 'circle'
                th = linspace(0,2*pi,40);
                patch(ax, o.center(1)+o.radius*cos(th), o.center(2)+o.radius*sin(th), ...
                      'k', 'EdgeColor','none');
            case 'rect'
                patch(ax, o.center(1)+o.w/2*[-1 1 1 -1], o.center(2)+o.h/2*[-1 -1 1 1], ...
                      'k', 'EdgeColor','none');
            case 'poly'
                patch(ax, o.verts(:,1), o.verts(:,2), 'k', 'EdgeColor','none');
        end
    end
end

