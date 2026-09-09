function ok = line_of_sight(occGrid, c1, c2)
    x0 = c1(1); y0 = c1(2); x1 = c2(1); y1 = c2(2);
    dx = abs(x1-x0); dy = abs(y1-y0);
    sx = sign(x1-x0); sy = sign(y1-y0);
    err = dx - dy;
    x = x0; y = y0;
    ok = true;
    while true
        if occGrid(y, x)
            ok = false; return;
        end
        if x==x1 && y==y1
            break;
        end
        e2 = 2*err;
        movedX = false; movedY = false;
        if e2 > -dy
            err = err - dy; x = x + sx; movedX = true;
        end
        if e2 < dx
            err = err + dx; y = y + sy; movedY = true;
        end
        if movedX && movedY
            if occGrid(y, x-sx) && occGrid(y-sy, x)
                ok = false; return;
            end
        end
    end
end

