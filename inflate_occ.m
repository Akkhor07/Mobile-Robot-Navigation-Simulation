function inflated = inflate_occ(occGrid, res, radius)
    rCells = max(1, round(radius/res));
    [dxk, dyk] = meshgrid(-rCells:rCells, -rCells:rCells);
    kernel = double((dxk.^2 + dyk.^2) <= rCells^2);
    inflated = conv2(double(occGrid), kernel, 'same') > 0;
end

