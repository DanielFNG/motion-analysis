classdef Motion < handle
% Class for using OpenSim analysis data to perform calculations.

    properties (SetAccess = protected)
        MotionData
    end
    
    properties (Access = protected)
        CoM = 'center_of_mass_'
        Gravity = 9.80665
        Torque = '_moment'
    end
        
    methods
        
        function obj = Motion(motion_data)
            if nargin > 0
                obj.MotionData = motion_data;
            end
        end
        
        function load(obj, analyses)
            
            obj.MotionData.load(analyses)
            
        end
        
        function result = calculateTimeOfMotion(obj)
            
            if isempty(obj.MotionData.TimeRange)
                error('Require loaded data.');
            end
            
            % Cycle time calculation.
            result = obj.MotionData.TimeRange(2) - obj.MotionData.TimeRange(1);
            
        end
        
        function result = calculateCoMD(obj, direction)
        % Calculate CoM displacement.
        
            % Analysis requirements.
            obj.require('BK');
            
            % CoMD calculation. 
            directions = {'x', 'y', 'z'};
            for i=1:length(directions)
                label = [obj.CoM directions{i}];
                data = obj.MotionData.BK.Positions.getColumn(label);
                result.(directions{i}) = peak2peak(data);
            end
            
            % Optionally, return only one direction.
            if nargin == 2
                result = result.(direction);
            end
        end
        
        function result = calculateROM(obj, joint)
        % Calculate weight-normalised peak torque at given joint.
        
            % Analysis requirements.
            obj.require('IK');
            
            % ROM calculation. 
            trajectory = obj.MotionData.IK.Kinematics.getColumn(joint);
            result = peak2peak(trajectory);
        end
        
    end
    
    methods (Access = protected)
        
        function require(obj, analyses)
        % Throw an error if any input analyses haven't been loaded.
        
            if isa(analyses, 'char')
                analyses = {analyses};
            end
        
            for i=1:length(analyses)
                analysis = analyses{i};
                if ~obj.MotionData.isLoaded(analysis)
                    error('Requires %s data loaded.', analysis);
                end
            end
        
        end
        
    end

end