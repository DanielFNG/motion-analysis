classdef MotionData < handle & dynamicprops
% Class for loading & obtaining read access to OpenSim simulation data.

    properties (SetAccess = private)
        TimeRange
        Frequency = 1000
        ModelMass
        MotionSpeed = 0
        MotionDirection = 'N/A'
    end
    
    properties (Access = private)
        LoadedAnalyses
        Trial
    end
        
    methods
        
        function obj = MotionData(trial, analyses, speed, direction)
            if nargin > 0
                obj.Trial = trial;
                obj.computeModelMass();
                if nargin > 1
                    obj.load(analyses);
                end
                if nargin > 2
                    obj.accountForFixedSpeed(speed, direction);
                    obj.MotionSpeed = speed;
                    obj.MotionDirection = direction;
                end
            end
        end
        
        function load(obj, analyses)
        % Load the data from each analysis in turn.
        
            % Support for a single analysis input as a string. 
            if isa(analyses, 'char')
                analyses = {analyses};
            end
            
            for i=1:length(analyses)
            
                % Check analysis is computed - if so get results folder.
                analysis = analyses{i};
                folder = obj.preload(analysis);
            
                % Load analysis Data.
                obj.addprop(analysis);
                switch analysis
                    
                    case 'GRF'
                        obj.(analysis).Forces = Data(obj.Trial.grfs_path);
                    case 'Markers'
                        obj.(analysis).Trajectories = ...
                            Data(obj.Trial.input_coordinates);
                        obj.(analysis).Trajectories.convertUnits('m');
                    case 'IK'
                        obj.(analysis).Kinematics = ...
                            Data([folder filesep 'ik.mot']);
                        obj.(analysis).OutputMarkers = Data(...
                            [folder filesep 'ik_model_marker_locations.sto']);
                    case 'RRA'
                        obj.(analysis).Kinematics = ...
                            Data([folder filesep 'RRA_Kinematics_q.sto']);
                        obj.(analysis).Forces = ...
                            Data([folder filesep 'RRA_Actuation_force.sto']);
                        obj.(analysis).TrackingErrors = ...
                            Data([folder filesep 'RRA_Kinematics_q.sto']);
                    case 'BK'
                        obj.(analysis).Positions = Data([folder filesep ...
                            'Analysis_BodyKinematics_pos_global.sto']);
                        obj.(analysis).Velocities = Data([folder filesep ...
                            'Analysis_BodyKinematics_vel_global.sto']);
                        obj.(analysis).Accelerations = Data([folder ...
                            filesep 'Analysis_BodyKinematics_acc_global.sto']);
                    case 'ID'
                        obj.(analysis).JointTorques = ...
                            Data([folder filesep 'id.sto']);
                    case 'CMC'
                        % Not yet implemented - need to see which files need to
                        % be read in. Will wait until more CMC-based analysis
                        % is required.
                end
                
                % Updated loaded memory.
                obj.LoadedAnalyses = [obj.LoadedAnalyses {analysis}];
            end
            
            % Update time range.
            obj.updateTimeRange();
            
        end
        
        function bool = isLoaded(obj, analysis)
            
            bool = isprop(obj, analysis);
            
        end
        
    end
    
    methods (Access = private)
        
        function folder = preload(obj, analysis)
        % Return path to folder containing analysis Data.
        %
        % Throws an error if the analysis data has not been computed.
            folder = [];
            if ~strcmp(analysis, {'GRF', 'Markers'})
                if ~obj.Trial.computed.(analysis)
                    error([analysis ' not computed.']);
                end
                folder = obj.Trial.results_paths.(analysis);
            end
            
        end
        
        function splineData(obj, start, finish)
            
            % Define our timestep from our frequency.
            time_step = 1/obj.Frequency;
            
            % Slightly widen the time range.
            time_range = round([start - time_step, finish + time_step], 3);
            
            % Construct timesteps from the new timerange - get rid of ones
            % which are outwith the true timerange.
            timesteps = transpose(time_range(1):time_step:time_range(2));
            timesteps(timesteps < start | timesteps > finish) = [];
            
            % Spline all the data.
            for i=1:length(obj.LoadedAnalyses)
                analysis = obj.LoadedAnalyses{i};
                fields = fieldnames(obj.(analysis));
                for j=1:length(fields)
                    obj.(analysis).(fields{j}).spline(timesteps);
                end
            end
            
            % Final adjustment to the objects time range - note that this 
            % can only ever make it tighter, not looser, so we don't run in
            % to circular problems.
            obj.TimeRange = [timesteps(1), timesteps(end)];
            
        end
        
        function updateTimeRange(obj)
            start_times = [];
            finish_times = [];
            timesteps = [];
            
            % Calculate the total time of all loaded analyses.
            for i=1:length(obj.LoadedAnalyses)
                analysis = obj.LoadedAnalyses{i};
                inner_fields = fieldnames(obj.(analysis));
                data = obj.(analysis).(inner_fields{1});
                range = data.getTimeRange();
                start_times = [start_times, range(1)];  %#ok<AGROW>
                finish_times = [finish_times, range(2)];  %#ok<AGROW>
                timesteps = [timesteps 1/data.Frequency];  %#ok<AGROW>
            end
            
            % Get the appropriate tolerance to use.
            tol = max(timesteps);
            
            % Ensure that these are not too different.
            if length(start_times) > 1
                if max(abs(diff(start_times))) > tol || ...
                        max(abs(diff(finish_times))) > tol
                    obj.TimeRange = 'Inconsistent';
                    error(['Analyses seem to have been run using different '...
                        'time ranges. Data has been loaded, but please '...
                        'attempt to identify the source of this error '...
                        'before continuing your analyses.']);
                end
            end
            
            % If changed, spline the data, which sets the new time range.
            latest_start = max(start_times);
            earliest_finish = min(finish_times);
            if isempty(obj.TimeRange) || ...
                    (obj.TimeRange(1) < latest_start ||  ...
                    obj.TimeRange(2) > earliest_finish)
                obj.splineData(latest_start, earliest_finish);
            end
            
        end
        
        function accountForFixedSpeed(obj, speed, direction)
            
            % Cartesian position adjustment.
            positions = {obj.Markers.Trajectories, ...
                obj.BK.Positions};
            for i=1:length(positions)
                time = positions{i}.getTotalTime();
                for j=1:positions{i}.NCols
                    if strcmpi(positions{i}.Labels{j}(end), direction)
                        initial_values = positions{i}.getColumn(j);
                        adjusted_values = accountForMovingReferenceFrame(...
                            initial_values, time, speed);
                        positions{i}.setColumn(j, adjusted_values);
                    end
                end
            end
            
            % CoP adjustment.
            forces = obj.GRF.Forces;
            for j=1:forces.NCols
                if strcmpi(forces.Labels{j}(end-1:end), ['p' direction])
                    initial_values = forces.getColumn(j);
                    adjusted_values = accountForMovingReferenceFrame(...
                        initial_values, time, speed);
                    forces.setColumn(j, adjusted_values);
                end
            end
            
            % BK velocity adjustment.
            velocity = obj.BK.Velocities;
            for j=1:velocity.NCols
                if strcmpi(velocity.Labels{j}(end), direction)
                    initial_values = velocity.getColumn(j);
                    velocity.setColumn(j, initial_values + speed);
                end
            end
            
        end
        
        function computeModelMass(obj)
            
            obj.ModelMass = obj.Trial.getInputModelMass();
            
        end
        
    end

end