function occ = build_environment(nx, ny, res, obstacles)
    [Xc, Yc] = meshgrid(1:nx, 1:ny);
    xw = (Xc - 0.5) * res;
    yw = (Yc - 0.5) * res;
    occ = false(ny, nx);
    for i = 1:numel(obstacles)
        o = obstacles(i);
        switch o.type
            case 'circle'
                occ = occ | ((xw - o.center(1)).^2 + (yw - o.center(2)).^2 <= o.radius^2);
            case 'rect'
                occ = occ | (xw >= o.center(1)-o.w/2 & xw <= o.center(1)+o.w/2 & ...
                             yw >= o.center(2)-o.h/2 & yw <= o.center(2)+o.h/2);
            case 'poly'
                occ = occ | inpolygon(xw, yw, o.verts(:,1), o.verts(:,2));
        end
    end
end

