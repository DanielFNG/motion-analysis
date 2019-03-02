classdef Polygon
   
    properties
        x
        z
    end
    
    methods
       
        function obj = Polygon(line_set)
            
            obj.verifyLineSet(line_set);
            obj.x = [line_set.x];
            obj.z = [line_set.z];
            
        end
        
    end
    
    methods (Static)
        
        function verifyLineSet(line_set)
           
           [start, finish] = line_set.getEndPoints();
            if ~(start == finish)
                error('These lines are not connected, this is not a polygon.');
            end
            
        end
        
    end
    
end