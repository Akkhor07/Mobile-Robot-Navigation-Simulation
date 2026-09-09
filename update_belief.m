function belief = update_belief(belief, pose, ranges, angles, maxRange, res, nx, ny, lOcc, lFree, lMax, lMin)
    step = res/2;
    for b = 1:numel(angles)
        ang = angles(b); rng = ranges(b);
        hit = rng < (maxRange - 1e-6);
        dvec = [cos(ang), sin(ang)];
        nSteps = max(0, floor(rng/step) - 1); % stop just before the hit cell
        for s = 0:nSteps
            px = pose(1) + dvec(1)*s*step;
            py = pose(2) + dvec(2)*s*step;
            gx = min(max(floor(px/res)+1,1), nx);
            gy = min(max(floor(py/res)+1,1), ny);
            belief(gy, gx) = max(lMin, belief(gy, gx) - lFree);
        end
        if hit
            hx = pose(1) + dvec(1)*rng; hy = pose(2) + dvec(2)*rng;
            gx = min(max(floor(hx/res)+1,1), nx);
            gy = min(max(floor(hy/res)+1,1), ny);
            belief(gy, gx) = min(lMax, belief(gy, gx) + lOcc);
        end
    end
end

