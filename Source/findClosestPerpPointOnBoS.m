function result = findClosestPerpPointOnBoS(bos_map, xcom, direction)

    n_frames = length(xcom);
    key_set = zeros(1, n_frames);
    value_set = {};
    for i=1:n_frames
        cx = xcom(i);
        lines = bos_map(i);
        suitable_lines = {};
        for l=1:length(lines)
            data = lines{l}.(direction);
            if data(1) <= cx && data(end) >= cx
                suitable_lines = [suitable_lines lines{l}];
            end
        end
        n_suitable = length(suitable_lines);
        dist = zeros(n_suitable, 1);
        for l=1:n_suitable
            xdata = suitable_lines{l}.x;
            zdata = suitable_lines{l}.z;
            m = (xdata(2) - xdata(1))/(zdata(2) - zdata(1));
            c = data(1) - m*xdata(1);
            switch direction
                case 'x'
                    dist(l) = m*cx + c;
                case 'z'
                    dist(l) = (cx - c)/m;
            end
        end
        [min, minloc] = min(dist);
        switch direction
            case 'x'
                result.z = cx;
                result.x = min;
            case 'y'
                result.x = cx;
                result.z = min;
        end
    end

end