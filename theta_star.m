function path = theta_star(occGrid, startCell, goalCell)
    [ny, nx] = size(occGrid);
    gScore = inf(ny, nx);
    fScore = inf(ny, nx);
    cameFromX = zeros(ny, nx);
    cameFromY = zeros(ny, nx);
    openMask = false(ny, nx);
    closedMask = false(ny, nx);

    gScore(startCell(2), startCell(1)) = 0;
    fScore(startCell(2), startCell(1)) = heuristic(startCell, goalCell);
    openMask(startCell(2), startCell(1)) = true;

    neigh = [-1 -1;-1 0;-1 1;0 -1;0 1;1 -1;1 0;1 1];
    maxIter = nx*ny*4;
    found = false;
    cx = startCell(1); cy = startCell(2);

    for iter = 1:maxIter
        openIdx = find(openMask);
        if isempty(openIdx)
            break;
        end
        fVals = fScore(openIdx);
        [~, mpos] = min(fVals);
        curLin = openIdx(mpos);
        [cy, cx] = ind2sub([ny nx], curLin);

        if cx == goalCell(1) && cy == goalCell(2)
            found = true;
            break;
        end

        openMask(cy, cx) = false;
        closedMask(cy, cx) = true;

        px = cameFromX(cy, cx); py = cameFromY(cy, cx);
        if px == 0 && py == 0
            parent = [cx, cy];
        else
            parent = [px, py];
        end

        for k = 1:8
            nxp = cx + neigh(k,1);
            nyp = cy + neigh(k,2);
            if nxp < 1 || nxp > nx || nyp < 1 || nyp > ny
                continue;
            end
            if occGrid(nyp, nxp) || closedMask(nyp, nxp)
                continue;
            end

            if line_of_sight(occGrid, parent, [nxp nyp])
                candParent = parent;
                tentG = gScore(candParent(2), candParent(1)) + heuristic(candParent, [nxp nyp]);
            else
                candParent = [cx cy];
                tentG = gScore(cy, cx) + heuristic([cx cy], [nxp nyp]);
            end

            if tentG < gScore(nyp, nxp)
                gScore(nyp, nxp) = tentG;
                cameFromX(nyp, nxp) = candParent(1);
                cameFromY(nyp, nxp) = candParent(2);
                fScore(nyp, nxp) = tentG + heuristic([nxp nyp], goalCell);
                openMask(nyp, nxp) = true;
            end
        end
    end

    if ~found
        path = [];
        return;
    end

    path = [cx, cy];
    while ~(cx == startCell(1) && cy == startCell(2))
        px = cameFromX(cy, cx); py = cameFromY(cy, cx);
        cx = px; cy = py;
        path = [[cx, cy]; path]; %#ok<AGROW>
    end
end

