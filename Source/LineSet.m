classdef LineSet
   
    properties
        Lines
        NLines 
    end
    
    methods
       
        function obj = LineSet(lines)
           
            if nargin > 0
                
                obj.Lines = lines;
                obj.NLines = length(lines);
                
            end
            
        end
        
        function [start, finish] = getEndPoints(obj)
            
            start = zeros(obj.NLines);
            finish = start;
            
            for i=1:obj.NLines
                start(i) = obj.Lines{i}.getStartPoint();
                finish(i) = obj.Lines{i}.getFinishPoint();
            end
            
        end
        
    end
    
end