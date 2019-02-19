function [x, z] = calculateUMax(bos_map, xcom)

    n_frames = length(xcom.x);
    for i=1:n_frames
        
        lines = bos_map(i);
        n_points = length(lines{1}.x);
        
        % Group the line values.
        x_vals = [];
        z_vals = [];
        for k=1:length(lines)
            x_vals = [x_vals; lines{k}.x];
            z_vals = [z_vals; lines{k}.z];
        end
        
        % Find maximal points.
        [xlim_min, xind_min] = min(x_vals);
        [xlim_max, xind_max] = max(x_vals);
        [zlim_min, zind_min] = min(z_vals);
        [zlim_max, zind_max] = max(z_vals);
        
        if xcom.x(i) >= xlim_min && xcom.x(i) <= xlim_max
            % Calculate suitable lines in x direction.
            suitable_lines = {};
            for l=1:length(lines)
                data = lines{l}.z;
                if xcom.z(i) >= data(1) && xcom.z(i) <= data(end)
                    suitable_lines = [suitable_lines lines{l}];
                end
            end
            n_suitable = length(suitable_lines);
            dist = zeros(n_suitable, 1);
            
            % Compute dist from suitable lines.
            for l=1:n_suitable
                xdata = suitable_lines{l}.x;
                zdata = suitable_lines{l}.z;
                m = (xdata(2) - xdata(1))/(zdata(2) - zdata(1));
                c = data(1) - m*xdata(1);
                dist(l) = m*xcom.z(i) + c;
            end
            
            % Find minimum dist.
            x.x(i) = min(dist);
            x.z(i) = xcom.z(i);
        else
            d = [abs(xcom.x(i) - xlim_min), abs(xcom.x(i) - xlim_max)];
            if d(1) < d(2)
                x.x(i) = xlim_min;
                ind = xind_min;
            else
                x.x(i) = xlim_max;
                ind = xind_max;
            end
            rem = mod(ind, n_points);
            line = (ind - rem)/n_points + 1;
            x.z(i) = lines{line}.z(rem);
        end
        
        if xcom.z(i) >= zlim_min && xcom.z(i) <= zlim_max
            % Calculate suitable lines in x direction.
            suitable_lines = {};
            for l=1:length(lines)
                data = lines{l}.x;
                if xcom.x(i) >= data(1) && xcom.x(i) <= data(end)
                    suitable_lines = [suitable_lines lines{l}];
                end
            end
            n_suitable = length(suitable_lines);
            dist = zeros(n_suitable, 1);
            
            % Compute dist from suitable lines.
            for l=1:n_suitable
                xdata = suitable_lines{l}.x;
                zdata = suitable_lines{l}.z;
                m = (xdata(2) - xdata(1))/(zdata(2) - zdata(1));
                c = data(1) - m*xdata(1);
                dist(l) = (xcom.x(i) - c)/m;
            end
            
            % Find minimum dist.
            z.z(i) = min(dist);
            z.x(i) = xcom.x(i);
        else
            d = [abs(xcom.z(i) - zlim_min), abs(xcom.z(i) - zlim_max)];
            if d(1) < d(2)
                z.z(i) = zlim_min;
                ind = zind_min;
            else
                z.z(i) = zlim_max;
                ind = zind_max;
            end
            rem = mod(ind, n_points);
            line = (ind - rem)/n_points;
            z.x(i) = lines{line}.x(rem);
        end
        
    end

end