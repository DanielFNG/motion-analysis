function corrected_positions = accountForMovingReferenceFrame(...
    positions, time, speed)

    n_frames = length(positions);
    travel = speed*time;
    tx = linspace(0, travel, n_frames);
    corrected_positions = reshape(positions, n_frames, 1) + ...
        reshape(tx, n_frames, 1);

end
