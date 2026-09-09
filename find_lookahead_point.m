function [lookPt, idxOut] = find_lookahead_point(path, pose, lookaheadDist, idxHint)
    N = size(path,1);
    searchStart = max(1, idxHint);
    searchEnd = min(N, searchStart+60);
    seg = searchStart:searchEnd;
    d2 = (path(seg,1)-pose(1)).^2 + (path(seg,2)-pose(2)).^2;
    [~, rel] = min(d2);
    nearestIdx = seg(rel);

    idx = nearestIdx; accDist = 0; lookPt = path(end,:);
    for i = nearestIdx:N-1
        segLen = hypot(path(i+1,1)-path(i,1), path(i+1,2)-path(i,2));
        if accDist + segLen >= lookaheadDist
            remain = lookaheadDist - accDist;
            tt = remain / max(segLen, 1e-6);
            lookPt = path(i,:) + tt*(path(i+1,:)-path(i,:));
            idx = i;
            break;
        end
        accDist = accDist + segLen;
        idx = i+1;
    end
    idxOut = nearestIdx;
end

