classdef GaitCycle < Motion
% Class for using OpenSim analysis data to perform calculations.

    properties (GetAccess = private, SetAccess = private)
        GRFRightFoot = 'ground_force1_' 
        GRFLeftFoot = 'ground_force2_'
        MTP1Marker = '_MTP1_'
        MTP5Marker = '_MTP5_'
        HeelMarker = '_Heel_'
        AnkleMarker = '_Ankle_Lat_'
        Sideways = 'Z'
    end
        
    methods
        
        function result = calculateStepFrequency(obj)
            
            % Analysis requirements.
            obj.require('GRF');
            
            % Frequency calculation.
            result = 1/obj.getTotalTime();
            
        end
        
        function result = calculateStepWidth(obj, adjacent)
            
            % Analysis requirements.
            obj.require('IK');
            adjacent.require('IK');
            
            % Identify the leading foot.
            [side, other_side] = obj.identifyLeadingFootIK();
            
            % Construct labels.
            first = [side obj.HeelMarker obj.Sideways];
            second = [other_side obj.HeelMarker obj.Sideways];
            
            % Get data.
            first = obj.Trial.data.IK.InputMarkers.getColumn(first);
            second = adjacent.Trial.data.IK.InputMarkers.getColumn(second);
            
            % Compute step width.
            result = abs(second(1) - first(1));
        
        end
        
        function result = calculateCoMD(obj, direction, speed)
        % Calculate CoM displacement.
        
            if nargin == 1 || any(strcmp({'y', 'z'}, direction))
                result = Motion.calculateCoMD(obj, direction);
            else
                obj.require('BK');
                label = [obj.CoM direction];
                data = obj.accountForTreadmill(...
                    obj.Trial.data.BK.Positions.getColumn(label), speed);
                result = peak2peak(data);
            end
        end
        
        function result = calculateCoPD(obj, cutoff, direction)
        % Calculate CoP displacement at the leading foot. 
        
            % Analysis requirements.
            obj.require('GRF');
            
            % CoPD calculation.  
            directions = {'x', 'z'};
            foot = obj.identifyLeadingFootGRF();
            stance = obj.isolateStancePhase(foot, cutoff);
            for i=1:length(directions)
                label = directions{i};
                cp = ['_p' label];
                cop = obj.Trial.data.GRF.Forces.getColumn([foot cp]);
                result.(directions{i}) = peak2peak(cop(stance));
            end
            
            % Optionally, return only one direction.
            if nargin == 3
                result = result.(direction);
            end
        end
        
        function result = calculateMoSNew(...
                obj, cutoff, speed, leg_length, direction)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
            
            % Directional behaviour.
            if nargin < 5
                directions = {'X', 'Z'};
            else
                directions = {upper(direction)};
            end
            
            for i=1:length(directions)
                
                % Acquire CoM data, & get it to same frequency as marker data.
                com_data = ...
                    copy(obj.Trial.data.BK.Positions.getColumn(com_label));
                com_data.spline(obj.Trial.data.IK.InputMarkers.Frequency);
                com_label = [obj.CoM directions{i}];
                com_pos = com_data.getColumn(com_label);
                com_vel = com_data.getColumn(com_label);
                
                % Time of gait cycle.
                time = obj.Trial.data.BK.Positions.getTotalTime();
                
                % Compute the BoS map.
                bos_map = obj.computeBoS(cutoff, speed);
                
                % If necessary account for speed in the CoM.
                if strcmp(directions{i}, 'x')
                    com_pos = GaitCycle.accountForTreadmill(com_pos, time, speed);
                    com_vel = com_vel + speed;
                end
                
                % Compute extrapolated centre of mass.
                xcom = calculateXCoM(com_pos, com_vel, leg_length);
                
                % Compute umax.
                u_max = findClosestPerpPointOnBoS(bos_map, xcom);
            end
            
        end
        
        function result = calculateMoS(...
                obj, stance_cutoff, speed, leg_length, direction)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
            
            % Get the start and end time of stance.
            [foot, side] = obj.identifyLeadingFootGRF();
            timesteps = obj.Trial.data.GRF.Forces.getColumn('time');
            stance_indices = obj.isolateStancePhase(foot, stance_cutoff);
            start_time = timesteps(stance_indices(1));
            end_time = timesteps(stance_indices(end));
            
            % MoS calculations.
            if nargin < 5
                directions = {'x', 'z'};
            else
                directions = {direction};
            end
            
            for i=1:length(directions)
                
                pos = obj.Trial.data.BK.Positions.slice(start_time, end_time);
                vel = obj.Trial.data.BK.Velocities.slice(start_time, end_time);
                com_label = [obj.CoM directions{i}];
                com_pos = pos.getColumn(com_label);
                com_vel = vel.getColumn(com_label);
                
                switch directions{i}
                    case 'x'
                        marker = obj.HeelMarker;
                        test = obj.AnkleMarker;
                    case 'z'
                        marker = obj.AnkleMarker;
                end
                
                markers = ...
                    obj.Trial.data.IK.InputMarkers.slice(start_time, end_time);
                test_label = [side test directions{i}];
                bos_label = [side marker directions{i}];
                bos = markers.getColumn(bos_label)/1000;  % Convert to m
                test = markers.getColumn(test_label)/1000;
                
                switch directions{i}
                    case 'x'
                        com_pos = GaitCycle.accountForTreadmill(com_pos, speed);
                        com_vel = com_vel + speed;
                        bos = GaitCycle.accountForTreadmill(bos, speed);
                        test = GaitCycle.accountForTreadmill(test, speed);
                end
                
                xcom = com_pos + com_vel*sqrt(leg_length/obj.Gravity);
                mos = min(bos - xcom);
                result.(directions{i}) = max(0, mos); 
            end
        end
        
        function visualiseBoS(obj, cutoff, speed, t)
            
            markers = obj.Trial.data.IK.InputMarkers;
            line_map = obj.computeBoS(cutoff, speed);
            n_timesteps = markers.NFrames;
            
            if nargin == 4
                frame = markers.getFrame(t);
                figure;
                xlim([-300 600]);
                ylim([-600 600]);
                hold on;
                lines = line_map(frame);
                n_lines = length(lines);
                for i=1:n_lines
                    plot(lines{i}.z(:), lines{i}.x(:), 'color', 'k');
                end
            else
                h = animatedline;
                hold on;
                axis([-1, 1, -1, 4]);
                for frame=1:n_timesteps
                    lines = line_map(frame);
                    n_lines = length(lines);
                    for i=1:n_lines
                        addpoints(h, lines{i}.z(:), lines{i}.x(:));
                    end
                    drawnow;
                    pause(0.05);
                    clearpoints(h);
                end
            end
            
        end
    end
    
    methods (Access = private)
        
        function [side, other_side] = identifyLeadingFootIK(obj)
            
            right = obj.Trial.data.IK.Kinematics.getColumn('hip_flexion_r');
            left = obj.Trial.data.IK.Kinematics.getColumn('hip_flexion_l');
            
            if right(1) > left(1)
                side = 'R';
                other_side = 'L';
            else
                side = 'L';
                other_side = 'R';
            end
        end
        
        function [foot, side, other_foot, other_side] = ...
                identifyLeadingFootGRF(obj)
        
            % Isolate vertical force data for each foot.
            vert = 'vy';
            right = obj.Trial.data.GRF.Forces.getColumn([obj.GRFRightFoot vert]);
            left = obj.Trial.data.GRF.Forces.getColumn([obj.GRFLeftFoot vert]);
            
            % The index at which each peak occurs.
            right_zeros = find(right == 0);
            left_zeros = find(left == 0);
            
            % Check which peaks sooner. 
            if right_zeros(1) > left_zeros(1)
                foot = obj.GRFRightFoot;
                other_foot = obj.GRFLeftFoot;
                side = 'R';
                other_side = 'L';
            else
                foot = obj.GRFLeftFoot;
                other_foot = obj.GRFRightFoot;
                side = 'L';
                other_side = 'R';
            end
        end
        
        function indices = isolateStancePhase(obj, foot, cutoff)
        % Get the indices corresponding to stance phase using GRF data.
        
            vert = 'vy';
            indices = ...
                find(obj.Trial.data.GRF.Forces.getColumn([foot vert]) > cutoff);
            if sum(diff(indices) ~= 1)
                error('Multiple stance phases detected.');
            end
            
        end
        
        function indices = isolateOffStancePhase(obj, foot, cutoff)
        % Get the indices corresponding to stance phase using GRF data.
        
            vert = 'vy';
            indices = ...
                find(obj.Trial.data.GRF.Forces.getColumn([foot vert]) > cutoff);
            
            diff_f = diff(indices);
            if sum(diff_f ~= 1) ~= 1
                error('Detected 1 or 3+ stance phases.');
            else
                index = find(diff_f ~= 1);
                indices = indices(index+1:end);
            end
            
        end
        
        function lines = computeBoS(obj, cutoff, speed)
        % Compute & return the BoS trajectory.
        %
        % Returns a map which takes a frame and returns a structure of
        % length L, each of which corresponds to a line (with x & z
        % points). These lines make up the BoS at this frame. 
            
            % Identify the leading foot and off foot.
            [lead, lead_side, off, off_side] = obj.identifyLeadingFootGRF();
            
            % Identify the support phase times.
            times = obj.identifySupportPhases(cutoff, lead, off);
            
            % Find the support phase pattern.
            if length(times) == 5
                n_phases = 4;
            else
                n_phases = 3;
            end
            phase = cell(n_phases, 1);
            
            % Get the timestep associated with marker data.
            markers = copy(obj.Trial.data.IK.InputMarkers);
            
            % Convert to m.
            markers.scaleColumns(0.001);  % Conversion to m.
            
            % Account for speed.
            state_labels = markers.Labels(3:end);
            time = markers.getTotalTime();
            for i=1:3:length(state_labels)-2
                input = markers.getColumn(state_labels{i});
                adjusted_input = ...
                    GaitCycle.accountForTreadmill(input, time, speed);
                markers.setColumn(state_labels{i}, adjusted_input);
            end
            
            % Take the appropriate slices.
            for i=1:n_phases
                phase{i} = markers.slice(times{i}, times{i+1});
            end
            
            % Compute the line segments.
            switch n_phases
                case 4
                    line{1} = ...
                        obj.computeBoSDoubleSupport(phase{1}, lead_side, off_side);
                    line{2} = obj.computeBoSSingleSupport(phase{2}, lead_side);
                    line{3} = ...
                        obj.computeBoSDoubleSupport(phase{3}, off_side, lead_side);
                    line{4} = obj.computeBoSSingleSupport(phase{4}, off_side);
                case 3
                    line{1} = obj.computeBoSSingleSupport(phase{1}, lead_side);
                    line{2} = ...
                        obj.computeBoSDoubleSupport(phase{2}, off_side, lead_side);
                    line{3} = obj.computeBoSSingleSupport(phase{3}, off_side);
            end
            
            % Create a line map.
            key_set = [];
            value_set = {};
            offset = 0;
            
            for i=1:length(line)
                n_frames = size(line{i}.x, 3);
                n_lines = size(line{i}.x, 1);
                n_points = size(line{i}.x, 2);
                
                for j=1:n_frames
                    key_set = [key_set j + offset];
                    lines = {};
                    for k=1:n_lines
                        lines{k}.x = reshape(line{i}.x(k,:,j), n_points, 1);
                        lines{k}.z = reshape(line{i}.z(k,:,j), n_points, 1);
                    end
                    value_set{j + offset} = lines;
                end
                offset = offset + n_frames;
            end
                    
            lines = containers.Map(key_set, value_set);
            
        end
        
        
        
        function times = identifySupportPhases(obj, cutoff, foot, off_foot)
            
            % Identify the stance phases for each.
            leading_stance = obj.isolateStancePhase(foot, cutoff);
            off_stance = obj.isolateOffStancePhase(off_foot, cutoff);
            
            % Find the double support sections as the intersection of the 
            % stance phases.
            double_support = intersect(leading_stance, off_stance);
            
            % Get the timesteps.
            time = obj.Trial.data.GRF.Forces.getColumn('time');
            
            % Split in to first and second double support phase.
            diff_ds = diff(double_support);
            if sum(diff_ds ~= 1) == 0
                ds1 = [];
                ds2 = double_support;
            elseif sum(diff_ds ~= 1) == 1
                index = find(diff_ds ~= 1);
                pre_split = double_support(index);
                post_split = double_support(index+1);
                ds1 = 1:pre_split;
                ds2 = post_split:double_support(end);
            else
                error('Found more than 2 double support phases.');
            end
            
            % Find the single support phases using set difference. 
            ss1 = setdiff(setdiff(leading_stance, ds1), ds2);
            ss2 = setdiff(off_stance, ds2);
            
            if isempty(ds1)
                times{1} = time(1);
                n = 2;
            else
                times{1} = time(1);
                times{2} = time(ss1(1));
                n = 3;
            end
            times{n} = time(ds2(1));
            times{n+1} = time(ss2(1));
            times{n+2} = time(end);
        end
        
        function lines = computeBoSDoubleSupport(obj, markers, side, other_side)
        % Computes boundary of support for data in single support.
        %
        % Returns a structure with fields 'x' and 'z', each of which is an n by 
        % t by m array giving the x & z positions at each time index (t), 
        % for each line (n), over the points in (m). 
        
            % Get required marker trajectories.
            lead_big_toe = [side obj.MTP1Marker];
            lead_small_toe = [side obj.MTP5Marker];
            lead_heel = [side obj.HeelMarker];
            off_ankle = [other_side obj.AnkleMarker];
            off_heel = [other_side obj.HeelMarker];
            
            switch side
                case 'R'
                    labels = {lead_big_toe, lead_small_toe, lead_heel, ...
                        off_heel, off_ankle};
                case 'L'
                    labels = {lead_big_toe, off_ankle, off_heel, lead_heel, ...
                        lead_small_toe};
            end
            
            % Compute line segment info. 
            lines = obj.computeCyclicLineSegments(markers, labels);
            
        end
        
        function lines = computeBoSSingleSupport(obj, markers, side)
        % Computes boundary of support for data in single support.
        %
        % Returns a structure with fields 'x' and 'z', each of which is an n by 
        % t by m array giving the x & z positions at each time index (t), 
        % for each line (n), over the points in (m).  
        
            % Get required marker trajectories.
            big_toe_label = [side obj.MTP1Marker];
            small_toe_label = [side obj.MTP5Marker];
            heel_label = [side obj.HeelMarker];
            labels = {big_toe_label, small_toe_label, heel_label};
            
            % Compute line segment info. 
            lines = obj.computeCyclicLineSegments(markers, labels);
            
        end
        
    end
    
    methods (Static)
        
        function corrected_positions = accountForTreadmill(...
                positions, time, speed)
            
            n_frames = length(positions);
            travel = speed*time;
            tx = linspace(0, travel, n_frames);
            corrected_positions = reshape(positions, n_frames, 1) + ...
                reshape(tx, n_frames, 1);
            
        end
        
        function lines = computeCyclicLineSegments(markers, labels)
            
            n_markers = length(labels);
            lines.x = zeros(n_markers, markers.Frequency, markers.NFrames);
            lines.z = lines.x;
            
            for i=1:n_markers
                next_marker = mod(i, n_markers) + 1;
                [x_s, z_s, x_e, z_e] = GaitCycle.getBoundaryPoints(...
                    markers, labels{i}, labels{next_marker});
                for j=1:markers.NFrames
                    lines.x(i, :, j) = ...
                        x_s(j):(x_e(j)-x_s(j))/(markers.Frequency-1):x_e(j);
                    lines.z(i, :, j) = ...
                        z_s(j):(z_e(j)-z_s(j))/(markers.Frequency-1):z_e(j);
                end
            end
            
        end
        
        function [x_s, z_s, x_e, z_e] = ...
                getBoundaryPoints(markers, start_label, end_label)
            
            z_s = markers.getColumn([start_label 'Z']);
            z_e = markers.getColumn([end_label 'Z']);
            x_s = markers.getColumn([start_label 'X']);
            x_e = markers.getColumn([end_label 'X']);
            
        end
    
    end

end