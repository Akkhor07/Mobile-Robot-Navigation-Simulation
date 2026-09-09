function Frep = apf_repulsion(pose, hitPts, ranges, influenceRadius, kRep)
    Frep = [0 0];
    for i = 1:size(hitPts,1)
        if isnan(hitPts(i,1))
            continue;
        end
        d = ranges(i);
        if d < influenceRadius && d > 1e-3
            dirVec = (pose(1:2) - hitPts(i,:)) / d;
            mag = kRep * (1/d - 1/influenceRadius) * (1/d^2);
            Frep = Frep + mag*dirVec;
        end
    end
end

