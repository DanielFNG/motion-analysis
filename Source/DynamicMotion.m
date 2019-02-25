classdef DynamicMotion < Motion
   
    properties (Access = private)
       Gravity = 9.80665
       Torque = '_moment'
    end
    
    methods
       
        function result = calculateWNPPT(obj, joint)
        % Calculate weight-normalised peak-to-peak torque at given joint.
        
            % Analysis requirements.
            obj.require('ID');
            
            % WNPT calculation. 
            torque = ...
                obj.MotionData.ID.JointTorques.getColumn([joint obj.Torque]);
            mass = obj.MotionData.Trial.getInputModelMass();
            result = peak2peak(torque)/mass;
        end
        
    end
    
end