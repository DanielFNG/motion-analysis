classdef GaitCycle < Gait

    methods
        
        function result = calculateTotalTime(obj)
           
            result = obj.MotionData.TimeRange(2) - obj.MotionData.TimeRange(1);
            
        end
    
        function result = calculateStepFrequency(obj)
            
            % Frequency calculation.
            result = 1/obj.calculateTotalTime();
            
        end
        
        function result = calculateStepWidth(obj)
            
            % Analysis requirements.
            obj.require('IK');
            
            % Identify the leading foot using IK.
            [side, other_side] = obj.identifyLeadingFootIK();
            
            % Construct labels.
            first = [side obj.HeelMarker obj.Sideways];
            second = [other_side obj.HeelMarker obj.Sideways];
            
            % Get data.
            first = obj.MotionData.Markers.Trajectories.getColumn(first);
            second = obj.MotionData.Markers.Trajectories.getColumn(second);
            
            % Compute step width.
            result = abs(second(1) - first(1));
        
        end
        
        function result = calculateWNPPT(obj, joint)
        % Calculate weight-normalised peak-to-peak torque at given joint.
        
            % Analysis requirements.
            obj.require('ID');
            
            % WNPT calculation. 
            torque = ...
                obj.MotionData.ID.JointTorques.getColumn([joint obj.Torque]);
            result = peak2peak(torque)/obj.MotionData.ModelMass;
        end
        
        function result = calculateCoPD(obj, direction)
        % Calculate CoP displacement at the leading foot. 
        
            % Analysis requirements.
            obj.require('GRF');
            
            % CoPD calculation.  
            directions = {'x', 'z'};
            foot = obj.identifyLeadingFootGRF();
            stance = obj.isolateStancePhase(foot);
            for i=1:length(directions)
                label = directions{i};
                cp = ['p' label];
                cop = obj.MotionData.GRF.Forces.getColumn([foot cp]);
                result.(directions{i}) = peak2peak(cop(stance));
            end
            
            % Optionally, return only one direction.
            if nargin == 2
                result = result.(direction);
            end
        end
        
    end

end