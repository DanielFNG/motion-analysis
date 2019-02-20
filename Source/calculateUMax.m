function [u_max, multiplier] = calculateUMax(line_map, x, z)

    % Annoying IK bug.
    top_level = line_map.Count - 1;

    % Make initial assignments.
    u_max.x.x = 1000*ones(top_level, 1);
    u_max.x.z = z;
    u_max.z.x = x;
    u_max.z.z = 1000*ones(top_level, 1);
    multiplier.x = zeros(top_level, 1);
    multiplier.z = multiplier.x;

    for frame = 1:top_level
        
        % Get the lines.
        lines = line_map(frame);
        
        % Get the polygon.
        polyx = [];
        polyz = [];
        for line = 1:length(lines)
            polyx = [polyx; lines{line}.x];  %#ok<*AGROW>
            polyz = [polyz; lines{line}.z];
        end
        
        % Check whether the point is inside the polygon.
        in = inpolygon(z(frame), x(frame), polyz, polyx);
        
        % Change behaviour based on in.
        if in
            suitable_lines = lines;
            limits = true;
            multiplier.x(frame) = 1;
            multiplier.z(frame) = 1;
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
        
        % Check each suitable line.
        n_suitable_lines = length(suitable_lines);
        for line = 1:n_suitable_lines
            
            current_line = suitable_lines{line};
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
    end
end