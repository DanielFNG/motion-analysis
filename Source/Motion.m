classdef Motion < handle
% Class for using OpenSim analysis data to perform calculations.

    properties (SetAccess = private)
        TimeRange = 'Not calculated'
        GRF = 'Not loaded'
        Markers = 'Not loaded'
        IK = 'Not loaded'
        RRA = 'Not loaded'
        BK = 'Not loaded'
        ID = 'Not loaded'
        CMC = 'Not loaded'
    end
    
    % Model-specific properties; should be modified if alternative models
    % are used e.g. not gait2392.
    properties (Access = protected)
        Trial
        Loaded
        Gravity = 9.80665;
        CoM = 'center_of_mass_'
        Torque = '_moment'
    end
        
    methods
        
        function obj = Motion(trial, analyses, speed, direction)
            if nargin > 0
                obj.Trial = trial;
                obj.initialiseLoadStatus();
                if nargin > 1
                    obj.load(analyses);
                    if nargin == 4
                        obj.accountForFixedSpeed(speed, direction);
                    end
                end
            end
        end
        
        function accountForFixedSpeed(obj, speed, direction)
            
            % Cartesian position adjustment.
            positions = {obj.Markers.Trajectories, obj.BK.Positions};
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
        
        function initialiseLoadStatus(obj)
        % Initialise Loaded to false for each possible analysis.
            
            obj.Loaded.Markers = false;
            obj.Loaded.GRF = false;
            obj.Loaded.IK = false;
            obj.Loaded.RRA = false;
            obj.Loaded.BK = false;
            obj.Loaded.ID = false;
            obj.Loaded.CMC = false;
            
        end
        
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
                obj.(analysis) = {};
            
                % Load analysis Data. 
                switch analysis
                    case 'GRF'
                        obj.GRF.Forces = Data(obj.Trial.grfs_path);
                    case 'Markers'
                        obj.Markers.Trajectories = ...
                            Data(obj.Trial.input_coordinates);
                        obj.Markers.Trajectories.convertUnits('m');
                    case 'IK'
                        obj.IK.Kinematics = ...
                            Data([folder filesep 'ik.mot']);
                        obj.IK.OutputMarkers = Data(...
                            [folder filesep 'ik_model_marker_locations.sto']);
                    case 'RRA'
                        obj.RRA.Kinematics = ...
                            Data([folder filesep 'RRA_Kinematics_q.sto']);
                        obj.RRA.Forces = ...
                            Data([folder filesep 'RRA_Actuation_force.sto']);
                        obj.RRA.TrackingErrors = ...
                            Data([folder filesep 'RRA_Kinematics_q.sto']);
                    case 'BK'
                        obj.BK.Positions = Data([folder filesep ...
                            'Analysis_BodyKinematics_pos_global.sto']);
                        obj.BK.Velocities = Data([folder filesep ...
                            'Analysis_BodyKinematics_vel_global.sto']);
                        obj.BK.Accelerations = Data([folder filesep ...
                            'Analysis_BodyKinematics_acc_global.sto']);
                    case 'ID'
                        obj.ID.JointTorques = ...
                            Data([folder filesep 'id.sto']);
                    case 'CMC'
                        % Not yet implemented - need to see which files need to
                        % be read in. Will wait until more CMC-based analysis
                        % is required.
                end
                
                % Note loaded status.
                obj.Loaded.(analysis) = true;
                
                % Check time range consistency.
                obj.updateTimeRange();
            end
        end
        
        function updateTimeRange(obj)
            
            analyses = fieldnames(obj.Loaded);
            start_times = [];
            finish_times = [];
            timesteps = [];
            
            % Calculate the total time of all loaded analyses.
            for i=1:length(analyses)
                if obj.Loaded.(analyses{i})
                    inner_fields = fieldnames(obj.(analyses{i}));
                    data = obj.(analyses{i}).(inner_fields{1});
                    range = data.getTimeRange();
                    start_times = [start_times, range(1)];  %#ok<AGROW>
                    finish_times = [finish_times, range(2)];  %#ok<AGROW>
                    timesteps = [timesteps 1/data.Frequency];  %#ok<AGROW>
                end
            end
            
            % Get the appropriate tolerance to use.
            if all(diff(timesteps) == 0)
                tol = 0.5*timesteps(1);
            else
                tol = 1.5*min(timesteps);
            end
            
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
            
            % Set the time range. 
            obj.TimeRange = data.getTimeRange();
            
        end
        
        function require(obj, analyses)
        % Throw an error if any input analyses haven't been loaded.
        
            if isa(analyses, 'char')
                analyses = {analyses};
            end
        
            for i=1:length(analyses)
                analysis = analyses{i};
                if ~obj.Loaded.(analysis)
                    error('Required analysis data not loaded.');
                end
            end
        
        end
        
        function result = getTotalTime(obj)
           
            % Analysis requirements.
            obj.require('IK');
            
            % Cycle time calculation.
            range = obj.IK.Kinematics.getTimeRange();
            result = range(2) - range(1);
            
        end
        
        function result = calculateCoMD(obj, direction)
        % Calculate CoM displacement.
        
            % Analysis requirements.
            obj.require('BK');
            
            % CoMD calculation. 
            directions = {'y', 'z'};
            for i=1:length(directions)
                label = [obj.CoM directions{i}];
                data = obj.BK.Positions.getColumn(label);
                result.(directions{i}) = peak2peak(data);
            end
            
            % Optionally, return only one direction.
            if nargin == 2
                result = result.(direction);
            end
        end
        
        function result = calculateWNPPT(obj, joint)
        % Calculate weight-normalised peak-to-peak torque at given joint.
        
            % Analysis requirements.
            obj.require('ID');
            
            % WNPT calculation. 
            torque = ...
                obj.ID.JointTorques.getColumn([joint obj.Torque]);
            mass = obj.Trial.getInputModelMass();
            result = peak2peak(torque)/mass;
        end
        
        function result = calculateROM(obj, joint)
        % Calculate weight-normalised peak torque at given joint.
        
            % Analysis requirements.
            obj.require('IK');
            
            % ROM calculation. 
            trajectory = obj.IK.Kinematics.getColumn(joint);
            result = peak2peak(trajectory);
        end
        
    end

end