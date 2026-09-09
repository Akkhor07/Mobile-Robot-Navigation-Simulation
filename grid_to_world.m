function pts = grid_to_world(cells, res)
    pts = [(cells(:,1) - 0.5) * res, (cells(:,2) - 0.5) * res];
end

