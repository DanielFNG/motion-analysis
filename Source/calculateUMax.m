function [u_max, multiplier] = calculateUMax(polygons, x, z)
% Calculate the closest distance between a point and a polygon. 
%
% Input:
%   polygon - a Polygon, or cell array of polygons
%   x - a value or array of values giving the x trajectory of a point
%   z - a value of array of values giving the z trajectory of a point
%
% Note: if any of the above are cell/numeric arrays, all must be
%       cell/numeric arrays as appropriate.
%
% Output: 
%   u_max - a struct with fields 'x' and 'z' giving the distances in these
%           directions between the point and polygon
%   multiplier - +1 or -1 depending on whether or not the point is outside
%                of the polygon
%
% Note: the lowest fields of u_max and multiplier will be arrays matching
%       the length of the input data.

    n_frames = length(x);
    
    % Handle the case where we have a single polygon rather than a cell
    % array of polygons.
    if n_frames == 1
        polygons = {polygons};
    end

    % Make initial assignments.
    u_max.x.x = 1000*ones(n_frames, 1);
    u_max.x.z = z;
    u_max.z.x = x;
    u_max.z.z = u_max.x.x;
    multiplier.x = zeros(n_frames, 1);
    multiplier.z = multiplier.x;
    
    for frame = 1:n_frames
        
        % Get the lines.
        line_set = polygons{frame}.LineSet;
        n_lines = polygons{frame}.LineSet.NLines;
        
        % Get the polygon.
        polyx = polygons{frame}.getXValues();
        polyz = polygons{frame}.getZValues();
        
        % Check whether the point is inside the polygon.
        in = inpolygon(z(frame), x(frame), polyz, polyx);
        
        % Change behaviour based on in.
        if in
            limits = true;
            multiplier.x(frame) = 1;
            multiplier.z(frame) = 1;
            
            % Check each line.
            for line = 1:n_lines
                
                current_line = line_set.Lines(line);
                z_range = current_line.z(2) - current_line.z(1);
                if z_range ~= 0
                    m = (current_line.x(2) - current_line.x(1))/z_range;
                    c = current_line.x(1) - m*current_line.z(1);
                    proposed_x = m*z(frame) + c;
                    proposed_z = (x(frame) - c)/m;
                else
                    proposed_x = 1000;
                    proposed_z = current_line.z(1);
                end
                
                if abs(proposed_x - x(frame)) < abs(u_max.x.x(frame) - x(frame))
                    if limits && ...
                            ~inpolygon(u_max.x.z(frame), proposed_x, polyz, polyx)
                        continue
                    end
                    u_max.x.x(frame) = proposed_x;
                end
                
                if abs(proposed_z - z(frame)) < abs(u_max.z.z(frame) - z(frame))
                    if limits && ...
                            ~inpolygon(proposed_z, u_max.z.x(frame), polyz, polyx)
                        continue
                    end
                    u_max.z.z(frame) = proposed_z;
                end
            end
        else
            if x(frame) < min(polyx) || x(frame) > max(polyx)
                multiplier.x(frame) = -1;
            else
                multiplier.x(frame) = 1;
            end
            if z(frame) < min(polyz) || z(frame) > max(polyz)
                multiplier.z(frame) = -1;
            else
                multiplier.z(frame) = 1;
            end
            if abs(x(frame) - min(polyx)) < abs(x(frame) - max(polyx))
                u_max.x.x(frame) = min(polyx);
            else
                u_max.x.x(frame) = max(polyx);
            end
            if abs(z(frame) - min(polyz)) < abs(z(frame) - max(polyz))
                u_max.z.z(frame) = min(polyz);
            else
                u_max.z.z(frame) = max(polyz);
            end
        end
    end
end