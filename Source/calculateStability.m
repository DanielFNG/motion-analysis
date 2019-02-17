function result = calculateStability(u_max, ex_com)

measure = calculateMeasureOfStability(u_max, ex_com);
result = max(measure, 0);

end


