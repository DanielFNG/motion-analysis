classdef Gait < Motion
% Class for using OpenSim analysis data to perform calculations.

    properties (Access = private)
        GRFRightFoot = 'ground_force1_' 
        GRFLeftFoot = 'ground_force2_'
        MTP1Marker = '_MTP1_'
        MTP5Marker = '_MTP5_'
        HeelMarker = '_Heel_'
        AnkleMarker = '_Ankle_Lat_'
        Forward = 'X'
        Sideways = 'Z'
    end
        
    methods
        
        function result = calculateMoS(obj, direction, mode)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
                
            % Compute extrapolated centre of mass.
            xcom = obj.calculateXCoM();
            
            % Compute the BoS map.
            bos_polygons = obj.computeBoS();
                
            % Compute umax.
            [u_max, multiplier] = calculateUMax(bos_polygons, xcom.x, xcom.z);
            
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
        
        function result = calculateMoSCoM(obj, direction, mode)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
            
            % Compute CoM position & velocity.
            com_pos = obj.getCoMPositionAndVelocity();
            
            % Compute the BoS map.
            pbos_polygons = obj.computePBoS();
            
            % Compute umax.
            [u_max, multiplier] = ...
                calculateUMax(pbos_polygons, com_pos.x, com_pos.z);
            
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
        
        function result = calculateXPMoS(obj, direction, mode)
            
            % Analysis requirements.
            obj.require({'GRF', 'IK', 'BK'});
                
            % Compute extrapolated centre of mass.
            xcom = obj.calculateXCoM();
            
            % Compute the BoS map.
            bos_polygons = obj.computeXPBoS();
            
            % Compute umax.
            [u_max, multiplier] = calculateUMax(bos_polygons, xcom.x, xcom.z);
            
            % Compute MoSAX.
            result.x = multiplier.x.*abs(u_max.x.x - xcom.x);
            result.z = multiplier.z.*abs(u_max.z.z - xcom.z);
            
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
        
        function visualise(obj, frame, filename, varargin)
            
            [points, polygons, lines, labels] = ...
                obj.parseVisualiseArgs(varargin{:});
            
            n_points = length(points);
            n_lines = length(lines);
            n_polygons = length(polygons);
            
            switch frame
                case 'all'
                    start_frame = 1;
                    n_frames = obj.MotionData.Markers.Trajectories.NFrames;
                otherwise
                    start_frame = frame;
                    n_frames = frame;
            end
            
            % Create plot axis properties etc.
            figure('units','normalized','outerposition',[0 0 1 1]);
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
            line_colors = {'r', 'g', 'b', 'm', 'c', 'y'};
            
            % Create an animated line for every polygon.
            animated_lines = cell(1, n_polygons);
            for i=1:n_polygons
                animated_lines{i} = animatedline(...
                    'LineWidth', 1.5, 'LineStyle', polygon_lines{i}); 
            end
            
            for frame = start_frame:10:n_frames
                
                % Clear the plot.
                if frame ~= start_frame
                    delete(findobj(gca, 'type', 'line'));
                    for p=1:n_polygons
                        clearpoints(animated_lines{p});
                    end
                end
                
                % Get the lines for this frame.
                for p = 1:n_polygons
                    line_set = polygons{p}{frame};
                    
                    % Draw each line.
                    for l = 1:length(line_set)
                        addpoints(animated_lines{p}, ...
                            line_set{l}.z(:), line_set{l}.x(:));
                    end
                end
                
                % Draw each point.
                for p = 1:n_points
                    plot(points{p}.z(frame), points{p}.x(frame), ...
                        point_style{p}, 'LineWidth', 1.5, 'MarkerSize', 12);
                end
                
                % Draw each line.
                for l=1:n_lines
                    plot(lines{l}{frame}.z, lines{l}{frame}.x, ...
                        'LineWidth', 1.5, 'color', line_colors{l});
                end
                
                % Add legend.
                legend(labels{:}, 'Location', 'bestoutside', 'FontSize', 15);
                
                % Update the plot.
                drawnow;
                
                % If requested save a gif.
                if ~isempty(filename)
                    fr = getframe(gcf);
                    im = frame2im(fr);
                    [imind, cm] = rgb2ind(im, 256);
                    if frame == 1
                        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
                    else
                        imwrite(imind, cm, filename, 'gif', 'DelayTime', 0, ...
                            'WriteMode', 'append');
                    end
                end
            end
            
            % Stop the gif from ending so suddenly.
            if ~isempty(filename)
                fr = getframe(gcf);
                im = frame2im(fr);
                [imind, cm] = rgb2ind(im, 256);
                for i=1:10
                    imwrite(imind, cm, filename, 'gif', 'DelayTime', 0, ...
                        'WriteMode', 'append');
                end
            end
            
        end
    end
    
    methods (Access = private)
        
        function [p, v] = getCoMPositionAndVelocity(obj)
        % Provide simpler access to CoM position & velocity.
        %
        % Outputs structs p and v which are indexed by x/y/z.
            
            % Directional behaviour.
            directions = {'x', 'y', 'z'};
            for i=1:length(directions)
                % Acquire CoM data.
                com_label = [obj.CoM directions{i}];
                p.(directions{i}) = ...
                    obj.MotionData.BK.Positions.getColumn(com_label);
                v.(directions{i}) = ...
                    obj.MotionData.BK.Velocities.getColumn(com_label);
            end
            
        end
        
        function xcom = calculateXCoM(obj)
            
            [p, v] = obj.getCoMPositionAndVelocity();
            
            xcom.x = extrapolatePendulum(p.x, v.x, obj.MotionData.LegLength);
            xcom.z = extrapolatePendulum(p.z, v.z, obj.MotionData.LegLength);
            
        end
        
        function [points, polygons, lines, labels] = ...
                parseVisualiseArgs(obj, varargin)
            
            points = {};
            polygons = {};
            lines = {};
            labels = {};
            
            if any(strcmp('BoS', varargin))
                polygons = [polygons {obj.computeBoS()}];
                labels = [labels 'BoS'];
            end
            
            if any(strcmp('PBoS', varargin))
                polygons = [polygons {obj.computePBoS()}];
                labels = [labels 'PBoS'];
            end
            
            if any(strcmp('XPBoS', varargin))
                polygons = [polygons {obj.computeXPBoS()}];
                labels = [labels 'XPBoS'];
            end
            
            if any(strcmp('CoM', varargin))
                [p, ~] = obj.getCoMPositionAndVelocity();
                points = [points p];
                labels = [labels 'CoM'];
            end
            
            if any(strcmp('XCoM', varargin))
                xcom = obj.calculateXCoM();
                points = [points xcom];
                labels = [labels 'XCoM'];
            end
            
            if any(strcmp('MoS', varargin))
                if ~any(strcmp(labels, 'XCoM')) || ~any(strcmp(labels, 'BoS'))
                    error('Need XCoM and BoS for MoS.');
                end
                u_max = calculateUMax(obj.computeBoS(), xcom.x, xcom.z);
                lines = [lines {obj.constructLine(u_max.x, xcom)} ...
                    {obj.constructLine(u_max.z, xcom)}];
                labels = [labels 'MoS-x' 'MoS-z'];
            end
            
            if any(strcmp('MoSCoM', varargin))
                if ~any(strcmp(labels, 'CoM')) || ~any(strcmp(labels, 'PBoS'))
                    error('Need CoM and PBoS for MoSCoM.');
                end
                u_max = calculateUMax(obj.computePBoS(), p.x, p.z);
                lines = [lines {obj.constructLine(u_max.x, p)} ...
                    {obj.constructLine(u_max.z, p)}];
                labels = [labels 'MoSCoM-x' 'MoSCoM-z'];
            end
            
            if any(strcmp('XPMoS', varargin))
                if ~any(strcmp(labels, 'XCoM')) || ~any(strcmp(labels, 'XPBoS'))
                    error('Need XCoM and XPBoS for XPMoS.');
                end
                u_max = calculateUMax(...
                    obj.computeXPBoS(), xcom.x, xcom.z);
                lines = [lines {obj.constructLine(u_max.x, xcom)} ...
                    {obj.constructLine(u_max.z, xcom)}];
                labels = [labels 'XPMoS' 'XPMoS-z'];
            end
            
        end
        
        function [side, other_side] = identifyFrontFoot(obj, frame, markers)
           
            right = markers.getColumn(['R' obj.MTP1Marker obj.Forward]);
            left = markers.getColumn(['L' obj.MTP1Marker obj.Forward]);
            
            if right(frame) > left(frame)
                side = 'R';
                other_side = 'L';
            else
                side = 'L';
                other_side = 'R';
            end
            
        end
        
        function [side, other_side] = identifyLeadingFootIK(obj)
            
            right = obj.MotionData.IK.Kinematics.getColumn('hip_flexion_r');
            left = obj.MotionData.IK.Kinematics.getColumn('hip_flexion_l');
            
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
            right = obj.MotionData.GRF.Forces.getColumn([obj.GRFRightFoot vert]);
            left = obj.MotionData.GRF.Forces.getColumn([obj.GRFLeftFoot vert]);
            
            % The index at which each the vertical force drops off.
            right_zeros = find(right < obj.MotionData.GRFCutoff);
            left_zeros = find(left < obj.MotionData.GRFCutoff);
            
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
        
        function indices = isolateStancePhase(obj, foot)
        % Get the indices corresponding to stance phase using GRF data.
        
            vert = 'vy';
            indices = find(obj.MotionData.GRF.Forces.getColumn([foot vert]) ...
                > obj.MotionData.GRFCutoff);
                
        end
        
        function indices = isolateOffStancePhase(obj, foot, cutoff)
        % Get the indices corresponding to the stance phases of the off foot.
        %
        % Similar to isolateStancePhase but does not require that only one
        % stance phase is detected. 
        
            vert = 'vy';
            indices = find(obj.MotionData.GRF.Forces.getColumn([foot vert]) ...
                > Obj.MotionData.GRFCutoff);
            
        end
        
        function marker_data = extrapolateMarkers(obj)
        % Create an extrapolated copy of the marker data.
            
            % Copy marker data & computer marker velocities.
            marker_data = copy(obj.MotionData.Markers.Trajectories);
            velocity_data = marker_data.computeGradients();
            
            % Select markers of interest.
            markers = {obj.MTP1Marker, obj.MTP5Marker, obj.HeelMarker};
            
            % Extrapolate the position of each marker.
            for side = 'LR'
                for direction = 'XZ'
                    com_label = [obj.CoM direction];
                    com_v = obj.MotionData.BK.Velocities.getColumn(com_label);
                    ankle_label = [side obj.AnkleMarker direction];
                    ankle_v = velocity_data.getColumn(ankle_label);
                    for marker = 1:length(markers)
                        marker_label = [side markers{marker} direction];
                        x = marker_data.getColumn(marker_label);
                        adjusted = extrapolateMovingPendulum(...
                            x, ankle_v, com_v, obj.MotionData.LegLength);
                        marker_data.setColumn(marker_label, adjusted);
                    end
                end
            end
            
        end
        
        function polygons = computeXPBoS(obj)
           
            % Initialise.
            n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            polygons = cell(n_frames, 1);
            
            % Create some extrapolated markers.
            markers = obj.extrapolateMarkers();
            
            for frame = 1:n_frames
                
                % Compute as if we were in double support.
                point_set = obj.computeDoubleSupportPoints(frame, markers);
                
                % Create the polygons for this frame.
                polygons{frame} = obj.constructPolygon(point_set);
                
            end
            
        end
        
        function polygons = computePBoS(obj)
           
            % Initialise.
            n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            polygons = cell(n_frames, 1);
            
            for frame = 1:n_frames
                
                % Compute as if we were in double support.
                point_set = obj.computeDoubleSupportPoints(frame);
                
                % Create the polygons for this frame.
                polygons{frame} = obj.constructPolygon(point_set);
                
            end
            
        end
        
        function polygons = computeBoS(obj)
            
            % Initialise.
            n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            polygons = cell(n_frames, 1);
            
            % Identify the leading foot and off foot.
            [lead, side, off, off_side] = obj.identifyLeadingFootGRF();
            
            % Get the frames at which we're in single & double support.
            lss_frames = obj.isolateStancePhase(lead);
            oss_frames = obj.isolateOffStancePhase(off);
            ds_frames = intersect(lss_frames, oss_frames);
            
            for frame = 1:n_frames
                
                % Check if we're in double support.
                if any(ds_frames == frame)
                    point_set = obj.computeDoubleSupportPoints(frame);
                elseif any(lss_frames == frame)
                    point_set = obj.computeSingleSupportPoints(frame, side);
                else
                    point_set = obj.computeSingleSupportPoints(frame, off_side);
                end
                
                % Create the polygon for this frame.
                polygons{frame} = obj.constructPolygon(point_set);
            end
            
        end
        
        function point_set = computeSingleSupportPoints(obj, frame, side)
            
            % Get required marker trajectories.
            markers = obj.MotionData.Markers.Trajectories;
            big_toe_x = markers.getColumn([side obj.MTP1Marker 'X']);
            heel_x = markers.getColumn([side obj.HeelMarker 'X']);
            big_toe_z = markers.getColumn([side obj.MTP1Marker 'Z']);
            small_toe_z = markers.getColumn([side obj.MTP5Marker 'Z']);
            
            % Create points.
            top_left.x = big_toe_x(frame);
            top_left.z = big_toe_z(frame);
            
            top_right.x = top_left.x;
            top_right.z = small_toe_z(frame);
            
            bottom_right.x = heel_x(frame);
            bottom_right.z = top_right.z;
            
            bottom_left.x = bottom_right.x;
            bottom_left.z = top_left.z;
            
            % Create point set.
            point_set = {top_left, top_right, bottom_right, bottom_left};
            
        end
        
        function point_set = computeDoubleSupportPoints(obj, frame, markers)
            
            % Handle input arguments.
            if nargin < 3
                markers = obj.MotionData.Markers.Trajectories;
            end
            
            % Check which side leads.
            [side, other_side] = obj.identifyFrontFoot(frame, markers);
            
            % Get required marker trajectories.
            lead_big_toe_x = markers.getColumn([side obj.MTP1Marker 'X']);
            lead_heel_x = markers.getColumn([side obj.HeelMarker 'X']);
            
            lead_big_toe_z = markers.getColumn([side obj.MTP1Marker 'Z']);
            lead_small_toe_z = markers.getColumn([side obj.MTP5Marker 'Z']);
            
            off_big_toe_x = markers.getColumn([other_side obj.MTP1Marker 'X']);
            off_heel_x = markers.getColumn([other_side obj.HeelMarker 'X']);
            
            off_big_toe_z = markers.getColumn([other_side obj.MTP1Marker 'Z']);
            off_small_toe_z = markers.getColumn([other_side obj.MTP5Marker 'Z']);
            
            % Create points.
            lead_top_left.x = lead_big_toe_x(frame);
            lead_top_left.z = lead_small_toe_z(frame);
            
            lead_top_right.x = lead_top_left.x;
            lead_top_right.z = lead_big_toe_z(frame);
            
            off_top.x = off_big_toe_x(frame);
            off_top.z = off_small_toe_z(frame);
            
            off_bottom_right.x = off_heel_x(frame);
            off_bottom_right.z = off_top.z;
            
            off_bottom_left.x = off_bottom_right.x;
            off_bottom_left.z = off_big_toe_z(frame);
            
            lead_bottom.x = lead_heel_x(frame);
            lead_bottom.z = lead_top_left.z;
            
            lead_bottom_right.x = lead_bottom.x;
            lead_bottom_right.z = lead_top_right.z;
            
            % Create point set.
            if lead_bottom.x >= off_bottom_right.x
                point_set = {lead_top_left, lead_top_right, off_top, ...
                    off_bottom_right, off_bottom_left, lead_bottom};
            else
                point_set = {lead_top_left, lead_top_right, off_top, ...
                    off_bottom_right, lead_bottom_right, lead_bottom};
            end
            
        end
        
    end
    
    methods (Static, Access = private)
        
        function line = constructLine(start, finish)
            
            n_points = 100;
            n_frames = length(start.x);
            switch n_frames
                case 1
                    line.x = linspace(start.x, finish.x, n_points);
                    line.z = linspace(start.z, finish.z, n_points);
                otherwise
                    line = cell(n_frames, 1);
                    for frame = 1:n_frames
                        line{frame}.x = linspace(start.x(frame), ...
                            finish.x(frame), n_points);
                        line{frame}.z = linspace(start.z(frame), ...
                            finish.z(frame), n_points);
                    end
            end
            
        end
        
        function polygon = constructPolygon(point_set)
            
            n_points = length(point_set);
            polygon = cell(n_points, 1);
            for i=1:n_points
                next = mod(i, n_points) + 1;
                polygon{i} = ...
                    GaitCycle.constructLine(point_set{i}, point_set{next});
            end
            
        end
    
    end

end