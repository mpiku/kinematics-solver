classdef (Abstract) constraint < matlab.mixin.Heterogeneous
    %CONSTRAINT Abstract class for all constraint
    % This class ensures that every implemented constraint provides methods
    % used in solving process. Following methods are obligated:
    % * getConstraint() - returns Phi matrix of the constraint
    % * getJacobi() - returns part of Jacobi matrix
    
    properties
        omega = [0 -1; 1 0];
    end
    
    methods (Abstract)
        getConstraint(obj);
        getJacobi(obj);
    end
    
end

