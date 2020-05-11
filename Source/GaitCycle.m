classdef GaitCycle < Gait

    methods
        
        function result = calculateStanceRatio(obj)
           
            obj.require('GRF');
            
            total = obj.calculateTotalTime();
            
            [foot, ~, ~, ~] = obj.identifyStanceFoot();
            
            [stance, ~] = obj.isolateStancePhase(foot);
            
            timesteps = obj.MotionData.GRF.Forces.getTimesteps();
            
            stance_time = timesteps(stance(end)) - timesteps(stance(1));
            
            result = stance_time/total;
            
        end
        
        function result = calculatePushRatio(obj)
            
            obj.require('GRF');
            
            [foot, ~, ~, ~] = obj.identifyStanceFoot();
            
            [pks, locs] = ...
                findpeaks(obj.MotionData.GRF.Forces.getColumn([foot 'vy']));
            
            % Identify the correct GRF1/2 peaks.
            n_pks = length(pks);
            if n_pks > 2
                dist = zeros(length(pks), 1);
                for i=1:length(pks)
                    dist(i) = max(abs(locs(i) - locs));
                end
                max_dist = max(dist);
                max_locs = locs(dist == max_dist);
            elseif n_pks == 2
                max_locs = locs;
            else
                error('Only one GRF peak detected.');
            end
            
            result = pks(locs == max_locs(1))/pks(locs == max_locs(2));
            
        end
        
        function result = getJointTrajectory(obj, joint)
            
            [~, side, ~, ~] = obj.identifyStanceFoot();
            
            result = obj.MotionData.IK.Kinematics.getColumn([joint '_' side]);
            
        end
        
        function result = getJointTorqueTrajectory(obj, joint)
            
            [~, side, ~, ~] = obj.identifyStanceFoot();
            
            result = obj.MotionData.ID.JointTorques.getColumn(...
                [joint '_' side '_moment']);
        end
        
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
            
            % Identify the stance foot.
            [~, side, ~, other_side] = obj.identifyStanceFoot();
            
            % Construct labels.
            first = [side obj.HeelMarker obj.Sideways];
            second = [other_side obj.HeelMarker obj.Sideways];
            
            % Get data.
            first = obj.MotionData.Markers.Trajectories.getColumn(first);
            second = obj.MotionData.Markers.Trajectories.getColumn(second);
            
            % Compute step width.
            result = abs(second(1) - first(1));
        
        end
        
        function result = calculateROM(obj, joint, unit)
           
            obj.require('IK');
            
            kinematics = obj.getJointTrajectory(joint);
            
            if nargin == 3 && strcmp(unit, 'rad')
                result = deg2rad(peak2peak(kinematics));
            else
                result = peak2peak(kinematics);
            end
            
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
        
        function result = calculateCoMD(obj, direction)
        % Calculate CoM displacement.
        
            % Analysis requirements.
            obj.require('BK');
            
            % CoMD calculation.
            directions = {'y', 'z'};
            for i=1:length(directions)
                label = directions{i};
                com = obj.MotionData.BK.Positions.getColumn(...
                    ['center_of_mass_' label]);
                result.(directions{i}) = peak2peak(com);
            end
            
            % Optionally, return only one direction.
            if nargin == 2
                result = result.(direction);
            end
        
        end
        
        function result = calculateCoPD(obj, direction)
        % Calculate CoP displacement at the leading foot. 
        
            % Analysis requirements.
            obj.require('GRF');
            
            % CoPD calculation.  
            directions = {'x', 'z'};
            [foot, ~, ~, ~] = obj.identifyStanceFoot();
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
        
        function result = calculateFootInversion(obj)
            
            [foot, side, ~, ~] = obj.identifyStanceFoot();
            [~, swing] = obj.isolateStancePhase(foot);
            subtalar = obj.MotionData.IK.Kinematics.getColumn(...
                ['subtalar_angle_' side]);
            result = mean(subtalar(swing));
            
        end
        
        function result = calculateFootDrop(obj)
            
            [foot, side, ~, ~] = obj.identifyStanceFoot();
            [~, swing] = obj.isolateStancePhase(foot);
            ankle = ...
                obj.MotionData.IK.Kinematics.getColumn(['ankle_angle_' side]);
            result = mean(ankle(swing));
            
        end
        
    end
    
    methods (Access = protected)
        
        function [foot, side, other_foot, other_side] = identifyStanceFoot(obj)
            % Identifies the leading foot. 
            %   Defined as the foot which is currently in stance. If both
            %   are in stance, then the foot which is at the earlier point
            %   of its stance phase.
            
            if obj.require({'GRF'})
                [foot, side, other_foot, other_side] = ...
                    identifyStanceFootGRF(obj);
            elseif obj.require({'IK'})
                [foot, side, other_foot, other_side] = ...
                    identifyStanceFootIK(obj);
            elseif obj.require({'Markers'})
                [foot, side, other_foot, other_side] = ...
                    identifyStanceFootMarkers(obj);
            else
                error('Require GRF, IK or Markers to find leading foot.');
            end
            
        end
        
        function [foot, side, other_foot, other_side] = ...
                identifyStanceFootMarkers(obj)
            % Use marker data to identify the leading foot.
            %   Leverages knowledge that gait data corresponds to one full
            %   gait cycle.
            
            % Identify forward position of MTP1 marker.
            right = markers.getColumn(['R' obj.MTP1Marker obj.Forward]);
            left = markers.getColumn(['L' obj.MTP1Marker obj.Forward]);
            
            % Choose leading foot as that of the front foot.
            [foot, side, other_foot, other_side] = ...
                obj.assignSideParams(right(1) > left(1));
            
        end
        
        function [foot, side, other_foot, other_side] = ...
                identifyStanceFootIK(obj)
            % Use IK to identify the leading foot.
            %   Leverages knowledge that gait data corresponds to one full
            %   gait cycle.
            
            % Isolate hip flexion angles.
            right = obj.MotionData.IK.Kinematics.getColumn('hip_flexion_r');
            left = obj.MotionData.IK.Kinematics.getColumn('hip_flexion_l');
            
            % Choose leading foot as that of the front leg.
            [foot, side, other_foot, other_side] = ...
                obj.assignSideParams(right(1) > left(1));
        end
        
    end

end