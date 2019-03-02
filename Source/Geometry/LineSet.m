classdef LineSet
% Class for storing & working with sets of lines in x/z space.
   
    properties
        Lines
        NLines 
    end
    
    methods
       
        function obj = LineSet(lines)
        % Construct a LineSet from an array of Line objects.
           
            if nargin > 0
                
                obj.Lines = lines;
                obj.NLines = length(lines);
                
            end
            
        end
        
        function [start, finish] = getEndPoints(obj)
        % Return the start and finish points for all Line objects in the 
        % LineSet as two PointSet objects.
            
            % Initialise an empty struct array for the points.
            start = struct('x', {}, 'z', {});
            finish = start;
        
            % Store the start and end points for each line.
            for i=1:obj.NLines
                start(i) = obj.Lines(i).getStartPoint();
                finish(i) = obj.Lines(i).getFinishPoint();
            end
            
            % Create and return two PointSet objects. 
            start = PointSet(start);
            finish = PointSet(finish);
            
        end
        
    end
    
end