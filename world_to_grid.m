function cell = world_to_grid(pt, res, nx, ny)
    gx = min(max(floor(pt(1)/res) + 1, 1), nx);
    gy = min(max(floor(pt(2)/res) + 1, 1), ny);
    cell = [gx, gy];
end

