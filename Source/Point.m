classdef Point
   
    properties
        x
        z
    end
    
    methods 
        
        function obj = Point(x, z)
           
            if nargin > 0
                obj.x = x;
                obj.z = z;
            end
            
        end
        
        function bool = eq(obj, another_obj)
           
            if obj.x == another_obj.x && obj.z == another_obj.z
                bool = true;
            else
                bool = false;
            end
            
        end
        
    end
    
end