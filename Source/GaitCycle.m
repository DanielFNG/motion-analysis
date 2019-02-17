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
            if ~all(diff(indices) == 1)
                error('Multiple stance phases detected.');
            end
            
        end
        
        function result = computeBoS(obj, cutoff)
            
            % Identify the leading foot and off foot.
            [lead, lead_side, off, off_side] = obj.identifyLeadingFootGRF();
            
            % Identify the support phase times.
            [ds1, ss1, ds2, ss2, fin] = ...
                obj.identifySupportPhases(cutoff, lead, off);
            
            % Get the timestep associated with marker data.
            markers = obj.Trial.data.IK.InputMarkers;
            timestep = 1/markers.Frequency;
            
            % Take the appropriate slices.
            first_ds = markers.slice(ds1, ss1 - timestep);
            first_ss = markers.slice(ss1, ds2 - timestep);
            second_ds = markers.slice(ds2, ss2 - timestep);
            second_ss = markers.slice(ss2, fin);
            
            % Compute the line segments.
            first_ds_lines = obj.computeBoSDoubleSupport(first_ds, lead_side);
            second_ds_lines = obj.computeBoSDoubleSupport(second_ds, off_side);
            first_ss_lines = obj.computeBoSSingleSupport(first_ss, lead_side);
            second_ss_lines = obj.computeBoSSingleSupport(second_ss, off_side);
            
            % Concatenate the cell arrays in the right order.
            result = [first_ds_lines, first_ss_lines, ...
                second_ds_lines, second_ss_lines];
            
        end
        
        function [ds1, ss1, ds2, ss2, fin] = ...
                identifySupportPhases(obj, cutoff, foot, off_foot)
            
            % Identify the stance phases for each.
            leading_stance = obj.isolateStancePhase(foot, cutoff);
            off_stance = obj.isolateStancePhase(off_foot, cutoff);
            
            % Find the double support sections as the intersection of the 
            % stance phases.
            double_support = intersect(leading_stance, off_stance);
            
            % Split in to first and second double support phase.
            diff_ds = diff(double_support);
            if sum(diff_ds ~= 1) ~= 1
                error('Error in double support split.');
            else
                index = find(diff_ds ~= 1);
                pre_split = double_support(index);
                post_split = double_support(index+1);
            end
            ds1 = 1:pre_split;
            ds2 = post_split:double_support(end);
            
            % Find the single support phases using set difference. 
            ss1 = setdiff(leading_stance, ds1);
            ss2 = setdiff(off_stance, ds2);
            
            % Get the timesteps.
            time = obj.Trial.data.GRF.Force.getColumn('time');
            ds1 = time(1);
            ss1 = time(ss1(1));
            ds2 = time(ds2(1));
            ss2 = time(ss2(1));
            fin = time(end);
        end
        
        function [m, c, x_s, x_e] = computeBoSDoubleSupport(...
                obj, markers, side, other_side)
        % Computes boundary of support for data in single support.
        %
        % Returns arrays of m, c, x_s, x_e, where y_s = m(1)*x_s + c(1).
        % The size of each array is 3 by t, where t is the number of
        % timesteps and there are 3 lines composing the BoS for single support. 
   
        
            % Get required marker trajectories.
            lead_big_toe = [side obj.MTP1Marker];
            lead_small_toe = [side obj.MTP5Marker];
            lead_heel = [side obj.HeelMarker];
            off_ankle = [other_side obj.AnkleMarker];
            off_heel = [otherside obj.HeelMarker];
            
            switch side
                case 'R'
                    labels = {lead_big_toe, lead_small_toe, lead_heel, ...
                        off_heel, off_ankle};
                case 'L'
                    labels = {lead_big_toe, off_ankle, off_heel, lead_heel, ...
                        lead_small_toe};
            end
            
            % Compute line segment info. 
            [m, c, x_s, x_e] = obj.computeCyclicLineSegments(markers, labels);
            
        end
        
        function [m, c, x_s, x_e] = computeBoSSingleSupport(obj, markers, side)
        % Computes boundary of support for data in single support.
        %
        % Returns arrays of m, c, x_s, x_e, where y_s = m(1)*x_s + c(1).
        % The size of each array is 5 by t, where t is the number of
        % timesteps and there are 5 lines composing the BoS for single
        % support.
        
            % Get required marker trajectories.
            big_toe_label = [side obj.MTP1Marker];
            small_toe_label = [side obj.MTP5Marker];
            heel_label = [side obj.HeelMarker];
            labels = {big_toe_label, small_toe_label, heel_label};
            
            % Compute line segment info. 
            [m, c, x_s, x_e] = obj.computeCyclicLineSegments(markers, labels);
            
        end
        
    end
    
    methods (Static)
        
        function corrected_positions = accountForTreadmill(positions, speed)
            
            n_frames = length(positions);
            dx = speed/n_frames;
            travel = (0:n_frames - 1)*dx;
            corrected_positions = positions + travel';
            
        end
        
        function [m, c, x_s, x_e] = computeCyclicLineSegments(markers, labels)
            
            n_markers = length(labels);
            m = zeros(n_markers, markers.NRows);
            c = m;
            x_s = m;
            y_s = m;
            x_e = m;
            y_e = m;
            
            for i=1:n_markers
                next_marker = mod(i, n_markers) + 1;
                [x_s(i,:), y_s(i,:), x_e(i,:), y_e(i,:)] = ...
                    getBoundaryPoints(markers, labels{i}, labels{next_marker});
                m(i,:) = calculateLineGradient(x_s, y_s, x_e, y_e);
                c(i,:) = calculateLineIntercept(m(i,:), x_s, y_s);
            end
            
        end
        
        function [x_s, y_s, x_e, y_e] = ...
                getBoundaryPoints(markers, start_label, end_label)
            
            x_s = markers.getColumn([start_label 'Z']);
            x_e = markers.getColumn([end_label 'Z']);
            y_s = markers.getColumn([start_label 'X']);
            y_e = markers.getColumn([end_label 'Z']);
            
        end
        
        function gradient = calculateLineGradient(x_s, y_s, x_e, y_e)
           
            gradient = (y_e - y_s)./(x_e - x_s);
            
        end
        
        function intercept = calculateLineIntercept(m, x, y)
           
            intercept = y - m.*x;
            
        end
    
    end

end