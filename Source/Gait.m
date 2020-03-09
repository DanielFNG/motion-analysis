classdef Gait < Motion
% Class for using OpenSim analysis data to perform calculations.

    properties (Access = protected)
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
            if nargin >= 2
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
            
            % Compute CoM position.
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
            if nargin >= 2
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
            if nargin >= 2
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
        
        function visualise(obj, frame, jump, filename, varargin)
            
            [points, polygons, lines, labels] = ...
                obj.parseVisualiseArgs(varargin{:});
            
            n_points = length(points);
            n_lines = length(lines);
            n_polygons = length(polygons);
            
            % Control frame argument behaviour.  
            if isa(frame, 'char') && strcmp(frame, 'all')
                start_frame = 1;
                n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            else
                if length(frame) == 1
                    start_frame = frame;
                    n_frames = frame;
                else
                    start_frame = frame(1);
                    n_frames = frame(2);
                end
            end
            
            % Control gif argument behaviour.
            if isempty(filename)
                make_gif = false;
            else
                make_gif = true;
            end
            
            % Create plot axis properties etc.
            fig = figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            title('Stability', 'FontSize', 20);
            xlabel('z (m)', 'FontSize', 15);
            ylabel('x (m)', 'FontSize', 15);
            hold on;
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
            
            % Get the CoM positions which we use for axis scaling.
            [com_pos, ~] = obj.getCoMPositionAndVelocity();
            
            for frame = start_frame:jump:n_frames
                
                % Clear the plot.
                if frame ~= start_frame
                    delete(findobj(gca, 'type', 'line'));
                    for p=1:n_polygons
                        clearpoints(animated_lines{p});
                    end
                end
                
                for p = 1:n_polygons
                    line_set = polygons{p}{frame}.LineSet;
                    % Draw each line.
                    for l = 1:line_set.NLines
                        addpoints(animated_lines{p}, ...
                            line_set.Lines(l).z(:), line_set.Lines(l).x(:));
                    end
                end
                
                % Draw each point, noting the x values.
                for p = 1:n_points
                    plot(points{p}.z(frame), points{p}.x(frame), ...
                        point_style{p}, 'LineWidth', 1.5, 'MarkerSize', 12);
                end
                
                % Draw each line.
                for l=1:n_lines
                    plot(lines{l}(frame).z, lines{l}(frame).x, ...
                        'LineWidth', 1.5, 'color', line_colors{l});
                end
                
                % Adjust the axes to match the movement of the CoM.
                axis([-0.3, 0.3, ...
                    com_pos.x(frame) - 0.8, com_pos.x(frame) + 0.8]);
                
                % Add legend.
                lg = legend(labels{:}, 'Location', 'bestoutside');
                lg.FontSize = 15;
                
                % Update the plot.
                drawnow;
                
                % If requested save a gif.
                if make_gif
                    fr = getframe(gcf);
                    im = frame2im(fr);
                    [imind, cm] = rgb2ind(im, 256);
                    if frame == start_frame
                        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf);
                    else
                        imwrite(imind, cm, filename, 'gif', 'DelayTime', 0, ...
                            'WriteMode', 'append');
                    end
                end
                
                % Stop the loop if the figure is closed.
                if ~ishghandle(fig)
                    break;
                end
            end
            
        end
        
        function visualiseCoP(obj)
            
            time = obj.MotionData.GRF.Forces.getTimesteps();
           
            for direction = {obj.Forward, obj.Sideways}
                figure;
                hold on;
                switch direction{1}
                    case obj.Forward
                        const = obj.MotionData.ToeLength;
                        variable_label = obj.HeelMarker;
                    case obj.Sideways
                        const = 0;
                        variable_label = obj.MTP5Marker;
                end
                for side = {'R', 'L'}
                    switch side{1}
                        case 'R'
                            grf_label = obj.GRFRightFoot;
                            colour = 'b';
                        case 'L' 
                            grf_label = obj.GRFLeftFoot;
                            colour = 'r';
                    end
                    cop = obj.MotionData.GRF.Forces.getColumn(...
                        [grf_label 'p' direction{1}]);
                    toe = obj.MotionData.Markers.Trajectories.getColumn(...
                        [side{1} obj.MTP1Marker direction{1}]) + const;
                    variable = obj.MotionData.Markers.Trajectories.getColumn(...
                        [side{1} variable_label direction{1}]);
                    extended = [time', fliplr(time')];
                    region = [variable', fliplr(toe')];
                    fill(extended, region, colour, 'LineStyle', 'none', 'FaceAlpha', 0.25);
                    plot(time, cop, colour, 'LineWidth', 2);
                end
            end
        end
        
        function visualiseFootstepPlot(obj, frames)
        % Plot footstep dynamics.
        
            % set timeframe for plot 
            if nargin<2
                frames = 1:obj.MotionData.Markers.Trajectories.NFrames;
            end
            
            % Identify the leading foot and off foot.
            [lead, side, off, off_side] = obj.identifyStanceFoot();
        
            % Get relevant marker positions.
            marker_data = obj.MotionData.Markers.Trajectories;   
            front_swing = marker_data.getValue(frames, ...
                [side obj.AnkleMarker obj.Forward]);
            front_big_toe = marker_data.getValue(frames, ...
                [side obj.MTP1Marker obj.Forward]) + obj.MotionData.ToeLength;
            front_heel = marker_data.getValue(frames, ...
                [side obj.HeelMarker obj.Forward]);
            
            back_swing = marker_data.getValue(frames, ...
                [off_side obj.AnkleMarker obj.Forward]);
            back_big_toe = marker_data.getValue(frames, ...
                [off_side obj.MTP1Marker obj.Forward]) + ...
                obj.MotionData.ToeLength;
            back_heel = marker_data.getValue(frames, ...
                [off_side obj.HeelMarker obj.Forward]);
            
            % Identify the frames corresponding to double support.
            lss_frames = obj.isolateStancePhase(lead);
            oss_frames = obj.isolateStancePhase(off);
            ds_frames = intersect(lss_frames, oss_frames);
            
            ds_jumps = diff(ds_frames);
            idx = find(ds_jumps > 1);
            ds_end_frame = ds_frames([idx;length(ds_frames)]);
            
            if ds_jumps(1) == 1
                idx = [0; idx];
            end
            
            ds_start_frame = ds_frames(idx+1);

            % Get CoM and XCoM.
            com_label = [obj.CoM obj.Forward];
            com_p = obj.MotionData.BK.Positions.getColumn(com_label);
            com_v = obj.MotionData.BK.Velocities.getColumn(com_label);
            xcom = ...
                extrapolatePendulum(com_p, com_v, 0, obj.MotionData.LegLength);
            
            % get walking distance to plot DS areas
            y_min = min(min(front_heel), min(back_heel));
            y_max = max(max(front_big_toe), max(back_big_toe));
            
            % Plot wavy foot thing
            figure('units', 'normalized', 'outerposition', [0 0 1 1]);
            hold on
            
            %plot front foot + BoS
            timesteps = obj.MotionData.Markers.Trajectories.getColumn('time');
            timesteps = timesteps(frames);
            la = plot(timesteps, front_swing, 'b');
            x = [timesteps; flipud(timesteps)]';
            front_foot = [front_big_toe; flipud(front_heel)]';
            lf = fill(x, front_foot, 'c');
            lf.FaceAlpha = 0.2;
            
            % plot back foot + BoS
            ta = plot(timesteps, back_swing, 'r');
            back_foot = [back_big_toe; flipud(back_heel)]';
            tf = fill(x, back_foot, 'm');
            tf.FaceAlpha = 0.2;
            
            % plot  CoM
            h = plot(timesteps, com_p(frames), 'g');
            
            % plot capture point
            h1 = plot(timesteps, xcom(frames), 'k');
            
            % Shade areas where the subject is in double support phase 
            ds_fill = [];
            ds_duration = ds_end_frame - ds_start_frame;
            
            % there are cleaner ways of doing this, but works for now
            for i = 1:length(ds_start_frame)
                s = ds_start_frame(i);
                d = ds_duration(i);
                x = [timesteps(s:s+d); timesteps(s+d:-1:s)];
                y = [ones(d+1,1)*y_min;ones(d+1,1)*y_max];
                ds_fill = [ds_fill, fill(x,y,'y','FaceAlpha',0.2,...
                    'LineStyle','none','FaceColor',[0.8,0.8,0.8])];
            end
            
            % Get front foot for legend
            hl = legend([la,lf,ta,tf,h,h1],...
                sprintf ("%s Ankle", side),sprintf ("%s BoS", side),...
                sprintf ("%s Ankle", off_side),sprintf ("%s BoS", off_side),...
                'CoM','CP');
            hl.Location = 'northwest';
            
            % Adjust figure range.
            ylim([y_min y_max]);
            xlim([timesteps(1) timesteps(end)]);
        end
        
    end
    
    methods (Access = protected)
        
        function [p, v] = getCoMPositionAndVelocity(obj)
        % Provide simpler access to CoM position & velocity.
        %
        % Outputs structs p and v which are indexed by x/y/z.
        
            % Analysis requirements.
            obj.require('BK');
            
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
            
            xcom.x = extrapolatePendulum(p.x, v.x, 0, obj.MotionData.LegLength);
            xcom.z = extrapolatePendulum(p.z, v.z, 0, obj.MotionData.LegLength);
            
        end
        
        function [points, polygons, lines, labels] = ...
                parseVisualiseArgs(obj, varargin)
            
            points = {};
            polygons = {};
            lines = {};
            labels = {};
            
            if any(strcmp('BoS', varargin))
                bos = obj.computeBoS();
                polygons = [polygons {bos}];
                labels = [labels 'BoS'];
            end
            
            if any(strcmp('PBoS', varargin))
                pbos = obj.computePBoS();
                polygons = [polygons {pbos}];
                labels = [labels 'PBoS'];
            end
            
            if any(strcmp('XPBoS', varargin))
                xpbos = obj.computeXPBoS();
                polygons = [polygons {xpbos}];
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
            
            if any(strcmp('CoP', varargin))
                right.x = obj.MotionData.GRF.Forces.getColumn('ground_force1_px');
                right.z = obj.MotionData.GRF.Forces.getColumn('ground_force1_pz');
                left.x = obj.MotionData.GRF.Forces.getColumn('ground_force2_px');
                left.z = obj.MotionData.GRF.Forces.getColumn('ground_force2_pz');
                points = [points right left];
                labels = [labels 'CoP-R' 'CoP-L'];
            end

            if any(strcmp('MoS', varargin))
                if ~any(strcmp(labels, 'XCoM')) || ~any(strcmp(labels, 'BoS'))
                    error('Need XCoM and BoS for MoS.');
                end
                u_max = calculateUMax(bos, xcom.x, xcom.z);
                lines = [lines {obj.constructLineTrajectory(u_max.x, xcom)} ...
                    {obj.constructLineTrajectory(u_max.z, xcom)}];
                labels = [labels 'MoS-x' 'MoS-z'];
            end
            
            if any(strcmp('MoSCoM', varargin))
                if ~any(strcmp(labels, 'CoM')) || ~any(strcmp(labels, 'PBoS'))
                    error('Need CoM and PBoS for MoSCoM.');
                end
                u_max = calculateUMax(pbos, p.x, p.z);
                lines = [lines {obj.constructLineTrajectory(u_max.x, p)} ...
                    {obj.constructLineTrajectory(u_max.z, p)}];
                labels = [labels 'MoSCoM-x' 'MoSCoM-z'];
            end
            
            if any(strcmp('XPMoS', varargin))
                if ~any(strcmp(labels, 'XCoM')) || ~any(strcmp(labels, 'XPBoS'))
                    error('Need XCoM and XPBoS for XPMoS.');
                end
                u_max = calculateUMax(...
                    xpbos, xcom.x, xcom.z);
                lines = [lines {obj.constructLineTrajectory(u_max.x, xcom)} ...
                    {obj.constructLineTrajectory(u_max.z, xcom)}];
                labels = [labels 'XPMoS' 'XPMoS-z'];
            end
        end
        
        function [foot, side, other_foot, other_side] = ...
                identifyFrontFootMarkers(obj, frame, markers)
            % Identifies the front foot at a specified frame using marker data.
            %   Front foot is simply defined as the foot which is most
            %   forward.
            %
            %   Marker data can be provided or the objects own data can be
            %   used if available.
           
            % Use Gait objects own markers unless some are provided.    
            if nargin < 3
                obj.require({'Markers'});
                markers = obj.MotionData.Markers.Trajectories;
            end
            
            % Isolate right & left MTP1 marker data.
            right = markers.getColumn(['R' obj.MTP1Marker obj.Forward]);
            left = markers.getColumn(['L' obj.MTP1Marker obj.Forward]);
            
            % Choose front foot as the one which is most forward. 
            [foot, side, other_foot, other_side] = ...
                obj.assignSideParams(right(frame) > left(frame));
            
        end
        
        function [foot, side, other_foot, other_side] = assignSideParams(...
                obj, bool)
            % Assigns foot, side, other_foot & other_side parameters.
            %   Input should be a boolean value which is 1 if foot/side 
            %   correspond to the right side & 0 if foot/side correspond to
            %   the left side. 
            
            if bool
                side = 'R';
                other_side = 'L';
                foot = obj.GRFRightFoot;
                other_foot = obj.GRFLeftFoot;
            else
                side = 'L';
                other_side = 'R';
                foot = obj.GRFLeftFoot;
                other_foot = obj.GRFRightFoot;
            end
            
        end
        
        function [foot, side, other_foot, other_side] = ...
                identifyStanceFoot(obj)
            % Identifies the stance foot. 
            %   Defined as the foot which is currently in stance. If both
            %   are in stance, then the foot which is at the earlier point
            %   of its stance phase.
            
            if obj.require({'GRF'})
                [foot, side, other_foot, other_side] = ...
                    identifyStanceFootGRF(obj);
            else
                error('Require GRF to find leading foot for generic Gait.');
            end
            
        end
        
        function [foot, side, other_foot, other_side] = ...
                identifyStanceFootGRF(obj)
            % Use GRF to identify the leading foot.
            
            % Isolate forward position of each foot.
            pos = ['p' lower(obj.Forward)];
            right = obj.MotionData.GRF.Forces.getColumn([obj.GRFRightFoot pos]);
            left = obj.MotionData.GRF.Forces.getColumn([obj.GRFLeftFoot pos]);
            
            % Choose leading foot as that which is most forward. 
            [foot, side, other_foot, other_side] = ...
                obj.assignSideParams(right(1) > left(1));
            
        end
        
        function [stance, swing]  = isolateStancePhase(obj, foot)
        % Get the indices corresponding to stance phase using GRF data.
        
            vert = 'vy';
            frames = 1:obj.MotionData.GRF.Forces.NFrames;
            stance = find(obj.MotionData.GRF.Forces.getColumn([foot vert]) ...
                > obj.MotionData.GRFCutoff);
            swing = setdiff(frames, stance);
                
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
                        adjusted = extrapolatePendulum(...
                            x, ankle_v, com_v, obj.MotionData.LegLength);
                        marker_data.setColumn(marker_label, adjusted);
                    end
                end
            end
            
        end
        
        function polygons = computeXPBoS(obj)
            
            % Analysis requirements.
            obj.require({'Markers', 'BK'});
           
            % Initialise.
            n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            polygons = cell(n_frames, 1);
            
            % Create the extrapolated markers.
            markers = obj.extrapolateMarkers();
            
            for frame = 1:n_frames
                
                % Compute as if we were in double support.
                point_set = obj.computeDoubleSupportPoints(frame, markers);
                
                % Create the polygons for this frame.
                polygons{frame} = point_set.constructPolygon();
                
            end
            
        end
        
        function polygons = computePBoS(obj)
            
            % Analysis requirements.
            obj.require('Markers');
           
            % Initialise.
            n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            polygons = cell(n_frames, 1);
            
            for frame = 1:n_frames
                
                % Compute as if we were in double support.
                point_set = obj.computeDoubleSupportPoints(frame);
                
                % Create the polygons for this frame.
                polygons{frame} = point_set.constructPolygon();
                
            end
            
        end
       
        function polygons = computeBoS(obj)
            
            % Analysis requirements.
            obj.require({'Markers', 'GRF'});
            
            % Initialise.
            n_frames = obj.MotionData.Markers.Trajectories.NFrames;
            polygons = cell(n_frames, 1);
            
            % Identify the leading foot and off foot.
            [lead, side, off, off_side] = obj.identifyStanceFoot();
            
            % Get the frames at which we're in single & double support.
            lss_frames = obj.isolateStancePhase(lead);
            oss_frames = obj.isolateStancePhase(off);
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
               polygons{frame} = point_set.constructPolygon();
            end
            
        end
        
        function point_set = computeSingleSupportPoints(obj, frame, side)
            
            % Get required marker trajectories.
            markers = obj.MotionData.Markers.Trajectories;
            big_toe_x = markers.getValue(frame, [side obj.MTP1Marker 'X']);
            heel_x = markers.getValue(frame, [side obj.HeelMarker 'X']);
            big_toe_z = markers.getValue(frame, [side obj.MTP1Marker 'Z']);
            small_toe_z = markers.getValue(frame, [side obj.MTP5Marker 'Z']);
            
            % Create points.
            top_left.x = big_toe_x + obj.MotionData.ToeLength;
            top_left.z = big_toe_z;
            
            top_right.x = top_left.x;
            top_right.z = small_toe_z;
            
            bottom_right.x = heel_x;
            bottom_right.z = top_right.z;
            
            bottom_left.x = bottom_right.x;
            bottom_left.z = top_left.z;
            
            % Create point set.
            point_set = PointSet([top_left top_right bottom_right bottom_left]);
            
        end
        
        function point_set = computeDoubleSupportPoints(obj, frame, markers)
            
            % Handle input arguments.
            if nargin < 3
                markers = obj.MotionData.Markers.Trajectories;
            end
            
            % Check which side leads the double support phase. 
            [~, side, ~, other_side] = ...
                obj.identifyFrontFootMarkers(frame, markers);
            
            % Get required marker trajectories.
            lead_big_toe_x = markers.getValue(...
                frame, [side obj.MTP1Marker 'X']);
            lead_heel_x = markers.getValue(...
                frame, [side obj.HeelMarker 'X']);
            
            lead_big_toe_z = markers.getValue(...
                frame, [side obj.MTP1Marker 'Z']);
            lead_small_toe_z = markers.getValue(...
                frame, [side obj.MTP5Marker 'Z']);
            
            off_big_toe_x = markers.getValue(...
                frame, [other_side obj.MTP1Marker 'X']);
            off_heel_x = markers.getValue(...
                frame, [other_side obj.HeelMarker 'X']);
            
            off_big_toe_z = markers.getValue(...
                frame, [other_side obj.MTP1Marker 'Z']);
            off_small_toe_z = markers.getValue(...
                frame, [other_side obj.MTP5Marker 'Z']);
            
            % Create points.
            lead_top_left.x = lead_big_toe_x + obj.MotionData.ToeLength;
            lead_top_left.z = lead_small_toe_z;
            
            lead_top_right.x = lead_top_left.x;
            lead_top_right.z = lead_big_toe_z;
            
            off_top.x = off_big_toe_x + obj.MotionData.ToeLength;
            off_top.z = off_small_toe_z;
            
            off_bottom_right.x = off_heel_x;
            off_bottom_right.z = off_top.z;
            
            off_bottom_left.x = off_bottom_right.x;
            off_bottom_left.z = off_big_toe_z;
            
            lead_bottom.x = lead_heel_x;
            lead_bottom.z = lead_top_left.z;
            
            lead_bottom_right.x = lead_bottom.x;
            lead_bottom_right.z = lead_top_right.z;
            
            % Create point set.
            if lead_bottom.x >= off_bottom_right.x
                points = [lead_top_left, lead_top_right, off_top, ...
                    off_bottom_right, off_bottom_left, lead_bottom];
            else
                points = [lead_top_left, lead_top_right, off_top, ...
                    off_bottom_right, lead_bottom_right, lead_bottom];
            end
            point_set = PointSet(points);
            
        end
        
    end
    
    methods (Static, Access = private)
        
        function line = constructLineTrajectory(start, finish)
        % Construct a trajectory of lines from a trajectory of points. 
            
            n_frames = length(start.x);
            switch n_frames
                case 1
                    line = Line(start, finish);
                otherwise
                    line(n_frames) = Line();
                    for frame = 1:n_frames
                        orig.x = start.x(frame);
                        orig.z = start.z(frame);
                        dest.x = finish.x(frame);
                        dest.z = finish.z(frame);
                        line(frame) = Line(orig, dest);
                    end
            end
            
        end
        
    end

end