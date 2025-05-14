%% lineIntersectsRectangle.m - Check if line intersects with a rectangle
function intersects = lineIntersectsRectangle(p1, p2, rect_min, rect_max)
    % Simplified check if line segment from p1 to p2 intersects rectangle
    
    % Line parametric equation: p1 + t*(p2-p1), t in [0,1]
    dir = p2 - p1;
    
    % X planes intersection
    if dir(1) ~= 0
        tx1 = (rect_min(1) - p1(1)) / dir(1);
        tx2 = (rect_max(1) - p1(1)) / dir(1);
        tx_min = min(tx1, tx2);
        tx_max = max(tx1, tx2);
    else
        % Line is parallel to Y axis
        if p1(1) >= rect_min(1) && p1(1) <= rect_max(1)
            tx_min = -inf;
            tx_max = inf;
        else
            tx_min = inf;
            tx_max = -inf;
        end
    end
    
    % Y planes intersection
    if dir(2) ~= 0
        ty1 = (rect_min(2) - p1(2)) / dir(2);
        ty2 = (rect_max(2) - p1(2)) / dir(2);
        ty_min = min(ty1, ty2);
        ty_max = max(ty1, ty2);
    else
        % Line is parallel to X axis
        if p1(2) >= rect_min(2) && p1(2) <= rect_max(2)
            ty_min = -inf;
            ty_max = inf;
        else
            ty_min = inf;
            ty_max = -inf;
        end
    end
    
    % Intersection exists if intervals overlap
    t_min = max(tx_min, ty_min);
    t_max = min(tx_max, ty_max);
    
    intersects = (t_min <= t_max) && (t_max >= 0) && (t_min <= 1);
end