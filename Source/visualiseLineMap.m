function visualiseLineMap(line_map, bound_map)

    h = animatedline('LineWidth', 1.5, 'LineStyle', '-.');
    b = animatedline('LineWidth', 1.5, 'LineStyle', '--');
    hold on;
    for i=1:line_map.Count
        lines = line_map(i);
        bounds = bound_map(i);
        n_lines = length(lines);
        for j=1:n_lines
            addpoints(h, lines{j}.z(:), lines{j}.x(:));
        end
        for j=1:4
            addpoints(b, bounds{j}.z(:), bounds{j}.x(:));
        end
        drawnow;
        legend('Lines', 'Boundary');
        pause(0.2);
        clearpoints(h);
        clearpoints(b);
    end
    

end