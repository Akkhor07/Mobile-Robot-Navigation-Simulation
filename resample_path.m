function smoothPath = resample_path(pathWorld, spacing)
    if size(pathWorld,1) < 2
        smoothPath = pathWorld;
        return;
    end
    d = [0; cumsum(hypot(diff(pathWorld(:,1)), diff(pathWorld(:,2))))];
    % remove any duplicate cumulative-distance points (can happen if two
    % consecutive waypoints coincide) so interp1 doesn't error out
    [d, uidx] = unique(d, 'stable');
    pathWorld = pathWorld(uidx, :);
    totalLen = d(end);
    if totalLen < 1e-6
        smoothPath = pathWorld;
        return;
    end
    sSamples = 0:spacing:totalLen;
    if sSamples(end) < totalLen
        sSamples(end+1) = totalLen;
    end
    xs = interp1(d, pathWorld(:,1), sSamples, 'pchip');
    ys = interp1(d, pathWorld(:,2), sSamples, 'pchip');
    smoothPath = [xs(:) ys(:)];
end

