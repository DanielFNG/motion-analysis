classdef GaitCycle < SidedMotion & DynamicMotion
% Class for using OpenSim analysis data to perform calculations.

    properties (Access = private)
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
        
        function [p, v] = calculateCoMPositionAndVelocity(obj)
            
            % Directional behaviour.
            directions = {'x', 'z'};
            for i=1:length(directions)
                
                % Acquire CoM data, & get it to same frequency as marker data.
                com_label = [obj.CoM directions{i}];
                pos_data = copy(obj.BK.Positions);
                vel_data = obj.BK.Velocities;
                pos_data.spline(obj.Markers.Trajectories.Frequency);
                com_pos = pos_data.getColumn(com_label);
                com_vel = vel_data.getColumn(com_label);
                
                % Apply a filter - TEMPORARILY WHILE BK DOESN'T DO THIS!!
                dt = 1/100;
                freq = 6;
                p.(directions{i}) = ZeroLagButtFiltfilt(dt, freq, 4, 'lp', com_pos);
                v.(directions{i}) = ZeroLagButtFiltfilt(dt, freq, 4, 'lp', com_vel);
            end
            
        end
        
        function result = calculateMoS(...
                obj, cutoff, speed, leg_length, direction, mode)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
            
            % Compute CoM position & velocity.
            [com_pos, com_vel] = obj.calculateCoMPositionAndVelocity(speed);
                
            % Compute extrapolated centre of mass.
            xcom.x = calculateXCoM(com_pos.x, com_vel.x, leg_length);
            xcom.z = calculateXCoM(com_pos.z, com_vel.z, leg_length);
            
            % Compute the BoS map.
            bos_map = obj.computeBoS(cutoff, speed);
                
            % Compute umax.
            [u_max, multiplier] = calculateUMax(bos_map, xcom.x, xcom.z);
            
            % Compute MoS.
            result.x = multiplier.x.*abs(u_max.x.x - xcom.x);
            result.z = multiplier.z.*abs(u_max.z.z - xcom.z);
            
            % If requested provide MoS in one direction only.
            if nargin >= 5
                switch mode
                    case 'full'
                        result = result.(direction);
                    case 'mean'
                        result = mean(result.(direction));
                    case 'min'
                        result = min(result.(direction));
                end
            end
            
        end
        
        function result = calculateMoSCoM(obj, speed, direction, mode)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
            
            % Compute CoM position & velocity.
            com_pos = obj.calculateCoMPositionAndVelocity(speed);
            
            % Compute the BoS map.
            bos_map = obj.computeProjectedBoS(speed);
            
            % Compute umax.
            [u_max, multiplier] = calculateUMax(bos_map, com_pos.x, com_pos.z);
            
            % Compute MoS.
            result.x = multiplier.x.*abs(u_max.x.x - com_pos.x);
            result.z = multiplier.z.*abs(u_max.z.z - com_pos.z);
            
            % If requested provide MoS in one direction only.
            if nargin >= 3
                switch mode
                    case 'full'
                        result = result.(direction);
                    case 'mean'
                        result = mean(result.(direction));
                    case 'min'
                        result = min(result.(direction));
                end
            end
        end
        
        function result = calculateMoSAX(obj, speed, leg_length, direction, mode)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
            
            % Compute CoM position & velocity.
            [com_pos, com_vel] = obj.calculateCoMPositionAndVelocity(speed);
                
            % Compute extrapolated centre of mass.
            xcom.x = calculateXCoM(com_pos.x, com_vel.x, leg_length);
            xcom.z = calculateXCoM(com_pos.z, com_vel.z, leg_length);
            
            % Compute the BoS map.
            bos_map = obj.computeAXProjectedBoS(speed, leg_length);
            
            % Compute umax.
            [u_max, multiplier] = calculateUMax(bos_map, xcom.x, xcom.z);
            
            % Compute MoSAX.
            result.x = multiplier.x.*abs(u_max.x.x - xcom.x(1:end-1));  % IK bug
            result.z = multiplier.z.*abs(u_max.z.z - xcom.z(1:end-1));
            
            % If requested provide MoS in one direction only.
            if nargin >= 4
                switch mode
                    case 'full'
                        result = result.(direction);
                    case 'mean'
                        result = mean(result.(direction));
                    case 'min'
                        result = min(result.(direction));
                end
            end
        end
        
        function visualise(obj, bos_params, frame, varargin)
            
            leg_length = [];
            cutoff = [];
            if ~isempty(bos_params)
                leg_length = bos_params{1};
                if length(bos_params) > 1
                    cutoff = bos_params{2};
                end
            end
            
            [points, polygons, labels] = ...
                obj.parseVisualiseArgs(leg_length, cutoff, varargin{:});
            
            n_points = length(points);
            n_polygons = length(polygons);
            
            switch frame
                case 'all'
                    start_frame = 1;
                    n_frames = obj.Markers.Trajectories.NFrames;
                otherwise
                    start_frame = frame;
                    n_frames = frame;
            end
            
            % Create plot axis properties etc.
            figure;
            title('Stability', 'FontSize', 20);
            xlabel('z (m)', 'FontSize', 15);
            ylabel('x (m)', 'FontSize', 15);
            hold on;
            axis([-0.3, 0.4, -0.1, 2.3]);
            pbaspect([1 2 1]);
            ax = gca;
            ax.FontSize = 15;
            
            % Assign some params.
            point_style = {'bx', 'rx'};
            polygon_lines = {'-', '--', ':'};
            
            % Create an animated line for every polygon.
            animated_lines = cell(1, n_polygons);
            for i=1:n_polygons
                animated_lines{i} = animatedline(...
                    'LineWidth', 1.5, 'LineStyle', polygon_lines{i}); 
            end
            
            for frame = start_frame:n_frames
                
                % Wait so we can see it.
                pause(0.05);
                
                % Clear the plot.
                if frame ~= start_frame
                    delete(findobj(gca, 'type', 'line'));
                    for p=1:n_polygons
                        clearpoints(animated_lines{p});
                    end
                end
                
                % Get the lines for this frame.
                for p = 1:n_polygons
                    lines = polygons{p}(frame);
                    
                    % Draw each line.
                    for l = 1:length(lines)
                        addpoints(...
                            animated_lines{p}, lines{l}.z(:), lines{l}.x(:));
                    end
                end
                
                % Draw each point.
                for p = 1:n_points
                    plot(points{p}.z(frame), points{p}.x(frame), ...
                        point_style{p}, 'LineWidth', 1.5, 'MarkerSize', 12);
                end
                
                % Add legend.
                legend(labels{:}, 'Location', 'SouthEast', 'FontSize', 15);
                
                % Update the plot.
                drawnow;
            end
            
        end
        
        function [points, polygons, labels] = ...
                parseVisualiseArgs(obj, leg_length, cutoff, varargin)
            
            points = {};
            polygons = {};
            labels = {};
            
            if any(strcmp('BoS', varargin))
                polygons = [polygons {obj.computeBoS(cutoff)}];
                labels = [labels 'BoS'];
            end
            
            if any(strcmp('PBoS', varargin))
                polygons = [polygons {obj.computeProjectedBoS()}];
                labels = [labels 'PBoS'];
            end
            
            if any(strcmp('XPBoS', varargin))
                polygons = [polygons {obj.computeXProjectedBoS(leg_length)}];
                labels = [labels 'XPBoS'];
            end
            
            if any(strcmp('CoM', varargin))
                [p, ~] = obj.calculateCoMPositionAndVelocity();
                com.x = p.x;
                com.z = p.z;
                points = [points com];
                labels = [labels 'CoM'];
            end
            
            if any(strcmp('XCoM', varargin))
                [p, v] = obj.calculateCoMPositionAndVelocity();
                xcom.x = calculateXCoM(p.x, v.x, leg_length);
                xcom.z = calculateXCoM(p.z, v.z, leg_length);
                points = [points xcom];
                labels = [labels 'XCoM'];
            end
            
        end
        
        function visualiseBoS(obj, cutoff, speed, leg_length, t)
            
            markers = obj.Trial.data.IK.InputMarkers;
            line_map = obj.computeBoS(cutoff, speed);
            [projected_line_map, ~] = obj.computeProjectedBoS(speed);
            %xprojected_line_map = obj.computeXProjectedBoS(speed, leg_length);
            %xprojected_line_map = obj.computeAXProjectedBoS(speed, leg_length);
            n_timesteps = markers.NFrames;
             
            
            [p, v] = calculateCoMPositionAndVelocity(obj, speed);
            xcom.x = calculateXCoM(p.x, v.x, leg_length);
            xcom.z = calculateXCoM(p.z, v.z, leg_length);
            
            u_max = calculateUMax(projected_line_map, p.x, p.z);
            %u_max = calculateUMax(axprojected_line_map, xcom.x, xcom.z);
            %u_max = calculateUMax(line_map, xcom.x, xcom.z);
            
            %xlim = [0.2, 1.8];
            %zlim = [-0.3, 0.4];
            xlim = [-0.1, 2.3];
            zlim = [-0.3, 0.4];
            
            figure('units','normalized','outerposition',[0 0 1 1])
            
            if nargin == 5
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
                h = animatedline('LineWidth', 1.5);
                title('Stability Metrics', 'FontSize', 20);
                xlabel('z (m)', 'FontSize', 15);
                ylabel('x (m)', 'FontSize', 15);
                proj = animatedline('LineWidth', 1.5, 'LineStyle', '--');
                %xproj = animatedline('LineWidth', 1.5, 'LineStyle', ':');
                %axproj = animatedline('LineWidth', 1.5, 'LineStyle', '-.');
                hold on;
                axis([zlim(1), zlim(2), xlim(1), xlim(2)]);
                pbaspect([1 2 1]);
                for frame=1:n_timesteps - 2  % adjustment for IKTool bug
                    lines = line_map(frame);
                    projected_lines = projected_line_map(frame);
                    %xprojected_lines = xprojected_line_map(frame);
                    %axprojected_lines = axprojected_line_map(frame);
                    n_lines = length(lines);
                    for i=1:n_lines
                        addpoints(h, lines{i}.z(:), lines{i}.x(:));
                    end
                    n_projected_lines = length(projected_lines);
                    %n_xprojected_lines = length(xprojected_lines);
                    %n_axprojected_lines = length(axprojected_lines);
                    for i=1:n_projected_lines
                        addpoints(proj, projected_lines{i}.z(:), projected_lines{i}.x(:));
                    end
%                     for i=1:n_xprojected_lines
%                         addpoints(xproj, xprojected_lines{i}.z(:), xprojected_lines{i}.x(:));
%                     end
%                     for i=1:n_axprojected_lines
%                         addpoints(axproj, axprojected_lines{i}.z(:), axprojected_lines{i}.x(:));
%                     end
%                     mos_x_z = linspace(xcom.z(frame), u_max.x.z(frame), 100);
%                     mos_x_x = linspace(xcom.x(frame), u_max.x.x(frame), 100);
%                     mos_z_z = linspace(xcom.z(frame), u_max.z.z(frame), 100);
%                     mos_z_x = linspace(xcom.x(frame), u_max.z.x(frame), 100);
                    mos_x_z = linspace(p.z(frame), u_max.x.z(frame), 100);
                    mos_x_x = linspace(p.x(frame), u_max.x.x(frame), 100);
                    mos_z_z = linspace(p.z(frame), u_max.z.z(frame), 100);
                    mos_z_x = linspace(p.x(frame), u_max.z.x(frame), 100);
                    
                    %plot(u_max.z.z(frame), u_max.z.x(frame), 'gx', 'LineWidth', 1.5, 'MarkerSize', 12);
                    plot(p.z(frame), p.x(frame), 'bx', 'LineWidth', 1.5, 'MarkerSize', 12);
                    plot(xcom.z(frame), xcom.x(frame), 'rx', 'LineWidth', 1.5, 'MarkerSize', 12);
                    plot(mos_x_z, mos_x_x, 'LineWidth', 1.5, 'color', 'g');
                    plot(mos_z_z, mos_z_x, 'LineWidth', 1.5, 'color', 'm');
                    ax = gca;
                    ax.FontSize = 15;
                    legend('BoS', 'PBoS', 'CoM', 'XCoM', 'CoM-AP', 'CoM-ML', 'Location', 'SouthEast', 'FontSize', 15);
                    drawnow;
                    %pause(0.01);
                    
                    % Capture the plot as an image
                    fr = getframe(gcf);
                    im = frame2im(fr);
                    [imind,cm] = rgb2ind(im,256);
                    % Write to the GIF File
                    if frame == 1
                        imwrite(imind,cm,'test2.gif','gif', 'Loopcount',inf);
                    else
                        imwrite(imind,cm,'test2.gif','gif','DelayTime',0,'WriteMode','append');
                    end
                    
                    if frame ~= n_timesteps - 1
                        delete(findobj(gca, 'type', 'line'));
                        clearpoints(h);
                        clearpoints(proj);
                        %clearpoints(xproj);
                        %clearpoints(axproj);
                    else
                        for i=1:10
                            imwrite(imind,cm,'test2.gif','gif','DelayTime',0,'WriteMode','append');
                        end
                    end
                    
                    
                end
            end
            
        end
    end
    
    methods %(Access = private)
        
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
            right = obj.GRF.Forces.getColumn([obj.GRFRightFoot vert]);
            left = obj.GRF.Forces.getColumn([obj.GRFLeftFoot vert]);
            
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
                find(obj.GRF.Forces.getColumn([foot vert]) > cutoff);
            if sum(diff(indices) ~= 1)
                error('Multiple stance phases detected.');
            end
            
        end
        
        function indices = isolateOffStancePhase(obj, foot, cutoff)
        % Get the indices corresponding to stance phase using GRF data.
        
            vert = 'vy';
            indices = ...
                find(obj.GRF.Forces.getColumn([foot vert]) > cutoff);
            
            diff_f = diff(indices);
            if sum(diff_f ~= 1) ~= 1
                error('Detected 1 or 3+ stance phases.');
            else
                index = find(diff_f ~= 1);
                indices = indices(index+1:end);
            end
            
        end
        
        function [lines, bounds] = computeXProjectedBoS(obj, leg_length)
            
            % Get the timestep associated with marker data.
            markers = copy(obj.Markers.Trajectories);
            
            % Compute velocities.
            velocity = markers.computeGradients();
            
            % Get the BK data.
            positions = copy(obj.BK.Positions);
            velocities = copy(obj.BK.Velocities);
            
            % Spline it down.
            positions.spline(markers.Frequency);
            velocities.spline(markers.Frequency);
            
            % Get com data.
            com_pos = positions.getColumn([obj.CoM 'X']);
            com_vel = velocities.getColumn([obj.CoM 'X']);
            
            % Access the big toe data.
            right_toe = markers.getColumn(['R' obj.MTP1Marker 'X']);
            left_toe = markers.getColumn(['L' obj.MTP1Marker 'X']);
            right_heel = markers.getColumn(['R' obj.HeelMarker 'X']);
            left_heel = markers.getColumn(['L' obj.HeelMarker 'X']);
            right_heel_vel = velocity.getColumn(['R' obj.HeelMarker 'X']);
            left_heel_vel = velocity.getColumn(['L' obj.HeelMarker 'X']);
            
            % Extrapolate it.
            right_toe = calculateAXBoS(right_toe, right_heel_vel, com_pos, com_vel, leg_length);
            left_toe = calculateAXBoS(left_toe, left_heel_vel, com_pos, com_vel, leg_length);
            
            % 
            right_leads = right_toe > left_toe;
            left_leads = left_toe > right_toe;
            
            if right_leads(1)
                ind = find(right_leads == 0);
                labels = {'R', 'L'};
            else
                ind = find(left_leads == 0, 1);
                labels = {'L', 'R'};
            end
            first_lead = 1:(ind(1)-1);
            off = ind(1):ind(end);
            second_lead = (ind(end)+1):length(right_toe);
            
            % Extrapolate every marker by the correct ankle marker velocity.
            for i=3:markers.NCols
                label = markers.Labels{i};
                if strcmp(label(1), 'R')
                    vel = velocity.getColumn(['R_Ankle_Lat_' label(end)]);
                else
                    vel = velocity.getColumn(['L_Ankle_Lat_' label(end)]);
                end
                
                % Get CoM data.
                com_pos = positions.getColumn([obj.CoM label(end)]);
                com_vel = velocities.getColumn([obj.CoM label(end)]);
                
                % Apply a filter - TEMPORARILY WHILE BK DOESN'T DO THIS!!
                dt = 1/100;
                freq = 6;
                com_pos = ZeroLagButtFiltfilt(dt, freq, 4, 'lp', com_pos);
                com_vel = ZeroLagButtFiltfilt(dt, freq, 4, 'lp', com_vel);
                
                col = markers.getColumn(i);
                new = calculateAXBoS(col, vel, com_pos, com_vel, leg_length);
                markers.setColumn(i, new);
            end
            
            % Chop in to 3 segments.
            first_lead = markers.slice(first_lead);
            off = markers.slice(off);
            second_lead = markers.slice(second_lead);
            
            % Compute double support lines for entire gait cycle.
            line{1} = obj.computeBoSDoubleSupport(first_lead, labels{1}, labels{2});
            line{2} = obj.computeBoSDoubleSupport(off, labels{2}, labels{1});
            line{3} = obj.computeBoSDoubleSupport(second_lead, labels{1}, labels{2});
            
            % Create a line map.
            key_set = [];
            value_set = {};
            bound_set = {};
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
                    bounds = {};
                    for k=1:4
                        bounds{k}.x = reshape(line{i}.bounds.x(k,:,j), n_points, 1);
                        bounds{k}.z = reshape(line{i}.bounds.z(k,:,j), n_points, 1);
                    end
                    value_set{j + offset} = lines;
                    bound_set{j + offset} = bounds;
                end
                offset = offset + n_frames;
            end
                    
            lines = containers.Map(key_set, value_set);
            bounds = containers.Map(key_set, bound_set);
            
        end
        
        function [lines, bounds] = computeProjectedBoS(obj)
            
            % Get the timestep associated with marker data.
            markers = copy(obj.Markers.Trajectories);
            
            % Access the big toe data.
            right_toe = markers.getColumn(['R' obj.MTP1Marker 'X']);
            left_toe = markers.getColumn(['L' obj.MTP1Marker 'X']);
            
            % 
            right_leads = right_toe > left_toe;
            left_leads = left_toe > right_toe;
            
            if right_leads(1) 
                ind = find(right_leads == 0);
                labels = {'R', 'L'};
            else
                ind = find(left_leads == 0);
                labels = {'L', 'R'};
            end
            first_lead = 1:ind(1)-1;
            off = ind(1):ind(end);
            second_lead = ind(end)+1:length(right_toe);
            
            % Chop in to 3 segments.
            first_lead = markers.slice(first_lead);
            off = markers.slice(off);
            second_lead = markers.slice(second_lead);
            
            % Compute double support lines for entire gait cycle.
            line{1} = obj.computeBoSDoubleSupport(first_lead, labels{1}, labels{2});
            line{2} = obj.computeBoSDoubleSupport(off, labels{2}, labels{1});
            line{3} = obj.computeBoSDoubleSupport(second_lead, labels{1}, labels{2});
            
            % Create a line map.
            key_set = [];
            value_set = {};
            bound_set = {};
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
                    bounds = {};
                    for k=1:4
                        bounds{k}.x = reshape(line{i}.bounds.x(k,:,j), n_points, 1);
                        bounds{k}.z = reshape(line{i}.bounds.z(k,:,j), n_points, 1);
                    end
                    value_set{j + offset} = lines;
                    bound_set{j + offset} = bounds;
                end
                offset = offset + n_frames;
            end
                    
            lines = containers.Map(key_set, value_set);
            bounds = containers.Map(key_set, bound_set);
            
        end
        
        function [lines, bounds] = computeBoS(obj, cutoff)
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
            markers = copy(obj.Markers.Trajectories);
            
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
            bound_set = {};
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
                    bounds = {};
                    for k=1:4
                        bounds{k}.x = reshape(line{i}.bounds.x(k,:,j), n_points, 1);
                        bounds{k}.z = reshape(line{i}.bounds.z(k,:,j), n_points, 1);
                    end
                    value_set{j + offset} = lines;
                    bound_set{j + offset} = bounds;
                end
                offset = offset + n_frames;
            end
                    
            lines = containers.Map(key_set, value_set);
            bounds = containers.Map(key_set, bound_set);
            
        end
        
        function times = identifySupportPhases(obj, cutoff, foot, off_foot)
            
            % Identify the stance phases for each.
            leading_stance = obj.isolateStancePhase(foot, cutoff);
            off_stance = obj.isolateOffStancePhase(off_foot, cutoff);
            
            % Find the double support sections as the intersection of the 
            % stance phases.
            double_support = intersect(leading_stance, off_stance);
            
            % Get the timesteps.
            time = obj.GRF.Forces.getColumn('time');
            
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
            lead_big_toe_x = markers.getColumn([side obj.MTP1Marker 'X']);
            lead_heel_x = markers.getColumn([side obj.HeelMarker 'X']);
            
            lead_big_toe_z = markers.getColumn([side obj.MTP1Marker 'Z']);
            lead_small_toe_z = markers.getColumn([side obj.MTP5Marker 'Z']);
            
            off_big_toe_x = markers.getColumn([other_side obj.MTP1Marker 'X']);
            off_heel_x = markers.getColumn([other_side obj.HeelMarker 'X']);
            
            off_big_toe_z = markers.getColumn([other_side obj.MTP1Marker 'Z']);
            off_small_toe_z = markers.getColumn([other_side obj.MTP5Marker 'Z']);
            
            lead_top_left.x = lead_big_toe_x;
            lead_top_left.z = lead_small_toe_z;
            
            lead_top_right.x = lead_big_toe_x;
            lead_top_right.z = lead_big_toe_z;
            
            off_top.x = off_big_toe_x;
            off_top.z = off_small_toe_z;
            
            off_bottom_right.x = off_heel_x;
            off_bottom_right.z = off_small_toe_z;
            
            off_bottom_left.x = off_heel_x;
            off_bottom_left.z = off_big_toe_z;
            
            lead_bottom.x = lead_heel_x;
            lead_bottom.z = lead_small_toe_z;
            
            lead_bottom_right.x = lead_heel_x;
            lead_bottom_right.z = lead_big_toe_z;
            
            corner_top_right.x = lead_big_toe_x;
            
            corner_bottom_right.x = off_heel_x;
            
            corner_bottom_left.x = off_heel_x;
            
            off_corner_bottom_right.x = lead_heel_x;
            
            off_corner_bottom_left.x = lead_heel_x;
            
            n_lines = 6;
            lines.x = zeros(n_lines, markers.Frequency, markers.NFrames);
            lines.z = lines.x;
            lines.bounds.x = zeros(4, markers.Frequency, markers.NFrames);
            lines.bounds.z = lines.bounds.x;
            directions = {'x', 'z'};
            
            for d=1:length(directions)
                for i=1:markers.NFrames
                    if lead_heel_x(i) >= off_heel_x(i)
                        lines.(directions{d})(1, :, i) = linspace(lead_top_left.(directions{d})(i), ...
                            lead_top_right.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(2, :, i) = linspace(lead_top_right.(directions{d})(i), ...
                            off_top.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(3, :, i) = linspace(off_top.(directions{d})(i),...
                            off_bottom_right.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(4, :, i) = linspace(off_bottom_right.(directions{d})(i), ...
                            off_bottom_left.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(5, :, i) = linspace(off_bottom_left.(directions{d})(i), ...
                            lead_bottom.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(6, :, i) = linspace(lead_bottom.(directions{d})(i), ...
                            lead_top_left.(directions{d})(i), markers.Frequency);
                        
                    elseif lead_heel_x(i) < off_heel_x(i)
                        lines.(directions{d})(1, :, i) = linspace(lead_top_left.(directions{d})(i), ...
                            lead_top_right.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(2, :, i) = linspace(lead_top_right.(directions{d})(i), ...
                            off_top.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(3, :, i) = linspace(off_top.(directions{d})(i),...
                            off_bottom_right.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(4, :, i) = linspace(off_bottom_right.(directions{d})(i), ...
                            lead_bottom_right.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(5, :, i) = linspace(lead_bottom_right.(directions{d})(i), ...
                            lead_bottom.(directions{d})(i), markers.Frequency);
                        lines.(directions{d})(6, :, i) = linspace(lead_bottom.(directions{d})(i), ...
                            lead_top_left.(directions{d})(i), markers.Frequency);
                        
                        
                    end
                    
                end
            end
            
        end
        
        function lines = computeBoSSingleSupport(obj, markers, side)
        % Computes boundary of support for data in single support.
        %
        % Returns a structure with fields 'x' and 'z', each of which is an n by 
        % t by m array giving the x & z positions at each time index (t), 
        % for each line (n), over the points in (m).  
        
            % Get required marker trajectories.
            big_toe_x = markers.getColumn([side obj.MTP1Marker 'X']);
            heel_x = markers.getColumn([side obj.HeelMarker 'X']);
            
            big_toe_z = markers.getColumn([side obj.MTP1Marker 'Z']);
            small_toe_z = markers.getColumn([side obj.MTP5Marker 'Z']);
            
            top_left.x = big_toe_x;
            top_left.z = big_toe_z;
            
            top_right.x = big_toe_x;
            top_right.z = small_toe_z;
            
            bottom_right.x = heel_x;
            bottom_right.z = small_toe_z;
            
            bottom_left.x = heel_x;
            bottom_left.z = big_toe_z;
            
            n_lines = 4;
            lines.x = zeros(n_lines, markers.Frequency, markers.NFrames);
            lines.z = lines.x;
            lines.bounds.x = zeros(4, markers.Frequency, markers.NFrames);
            lines.bounds.z = lines.bounds.x;
            directions = {'x', 'z'};
            
            for d=1:length(directions)
                for i=1:markers.NFrames
                    lines.(directions{d})(1, :, i) = linspace(top_left.(directions{d})(i), ...
                        top_right.(directions{d})(i), markers.Frequency);
                    lines.(directions{d})(2, :, i) = linspace(top_right.(directions{d})(i), ...
                        bottom_right.(directions{d})(i), markers.Frequency);
                    lines.(directions{d})(3, :, i) = linspace(bottom_right.(directions{d})(i),...
                        bottom_left.(directions{d})(i), markers.Frequency);
                    lines.(directions{d})(4, :, i) = linspace(bottom_left.(directions{d})(i), ...
                        top_left.(directions{d})(i), markers.Frequency);
                end
            end
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