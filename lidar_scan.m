function [ranges, angles, hitPts] = lidar_scan(pose, occGrid, res, nx, ny, maxRange, numBeams, fov)
    angles = pose(3) + linspace(-fov/2, fov/2*(1-2/numBeams), numBeams); % avoid duplicate 0/360 beam
    ranges = maxRange * ones(1, numBeams);
    hitPts = nan(numBeams, 2);
    step = res/2;
    maxSteps = floor(maxRange/step);
    for b = 1:numBeams
        ang = angles(b);
        dvec = [cos(ang), sin(ang)];
        hitFound = false;
        for s = 1:maxSteps
            px = pose(1) + dvec(1)*s*step;
            py = pose(2) + dvec(2)*s*step;
            if px <= 0 || px >= nx*res || py <= 0 || py >= ny*res
                ranges(b) = s*step; hitPts(b,:) = [px py]; hitFound = true; break;
            end
            gx = min(max(floor(px/res)+1,1), nx);
            gy = min(max(floor(py/res)+1,1), ny);
            if occGrid(gy, gx)
                ranges(b) = s*step; hitPts(b,:) = [px py]; hitFound = true; break;
            end
        end
        if ~hitFound
            ranges(b) = maxRange;
        end
    end
end

