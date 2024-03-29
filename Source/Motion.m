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
        % Construct Motion from MotionData object. 
        
            if nargin > 0
                obj.MotionData = motion_data;
            end
        end
        
        function load(obj, analyses)
            
            obj.MotionData.load(analyses)
            
        end
        
        function result = calculateMetabolicRate(obj)
            
            % Analysis requirements.
            obj.require('SO');
            
            % Acquire data, timesteps & total time.
            data = obj.MotionData.SO.Metabolics.getColumn(...
                'metabolics_TOTAL');
            time = obj.MotionData.SO.Metabolics.getTotalTime();
            timesteps = obj.MotionData.SO.Metabolics.Timesteps;
            
            % Metabolic rate calculation
            result = trapz(timesteps, data)/time;
            
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
        
        function result = calculatePeak(obj, joint)
            
            % Analysis requirements.
            obj.require('IK');
            
            % Calculation.
            trajectory = obj.MotionData.IK.Kinematics.getColumn(joint);
            result = max(abs(trajectory));
        end
        
    end
    
    methods (Access = protected)
        
        function bool = require(obj, analyses)
        % Check if input analyses have been loaded. 
        %   Either throws error or returns boolean value if requested. 
        
            if isa(analyses, 'char')
                analyses = {analyses};
            end
        
            for i=1:length(analyses)
                analysis = analyses{i};
                if ~obj.MotionData.isLoaded(analysis)
                    if nargout == 0
                        error('Requires %s data loaded.', analysis);
                    else
                        bool = 0;
                        return
                    end
                else
                    bool = 1;
                end
            end
        
        end
        
    end

end