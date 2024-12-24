function [x, y] = calcVehPos_onLine(line, laneDist, dev)
    % line: n x 2 matrix representing the polyline [(x1, y1); (x2, y2); ...]
    % laneDist: distance travelled along the polyline
    % dev: lateral deviation from the polyline (negative is right, positive is left)

    % Compute the cumulative distance along the polyline
    distances = sqrt(sum(diff(line).^2, 2));
    cumulativeDistances = [0; cumsum(distances)];

    % Find the segment where the laneDist falls
    idx = find(cumulativeDistances >= laneDist, 1);
    if isempty(idx)
        idx = length(cumulativeDistances);
    end
    if idx > 1
        % Interpolate to find the exact point on the segment
        d1 = cumulativeDistances(idx-1);
        d2 = cumulativeDistances(idx);
        t = (laneDist - d1) / (d2 - d1);
        
        % Get the direction vector of the segment
        direction = line(idx, :) - line(idx-1, :);
        direction = direction / norm(direction);  % Normalize
        
        % Calculate the point on the polyline
        pointOnLine = (1 - t) * line(idx-1, :) + t * line(idx, :);
        
        % Compute the normal vector (perpendicular to the direction)
        normal = [-direction(2), direction(1)];
        
        % Calculate the final position with the deviation
        finalPosition = pointOnLine + dev * normal;
        
        x = finalPosition(1);
        y = finalPosition(2);
    else
        % If the laneDist is 0, the vehicle is at the start of the line
        p1 = line(1,:);
        p2 = line(2,:);
        vec = p2 - p1;
        vec = vec/norm(vec);
        x = p1(1) + dev * -vec(2);
        y = p1(2) + dev * vec(1);
        
    end
end