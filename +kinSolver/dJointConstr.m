classdef dJointConstr < kinSolver.constraint
    %DJOINTCONSTR Joint constraint - driving constraint
    % Implementation of joint constraint (pol. para obrotowa wiêŸ
    % kieruj¹cy).
    properties
        el_A = 0; % element A
        el_B = 0; % element B
        
        % Driving function and its derivatives
        f = 0;
        f_prim = 0;
        f_bis = 0;
    end
    
    methods
        function obj = dJointConstr(el_A, el_B, f, f_prim, f_bis)
            obj.el_A = el_A;
            obj.el_B = el_B;
            
            obj.f = f;
            obj.f_prim = f_prim;
            obj.f_bis = f_bis;
        end
        function Phi = getConstraint(obj)
            Phi = obj.el_A.fi_c - obj.el_B.fi_c - obj.f( obj.el_A.solver.time );
        end
        function Jacobi = getJacobi(obj)
            j_i_degree = [zeros(1, 3*(obj.el_A.index - 1)) zeros(1, 2) ...
                1 zeros(1, 3*( obj.el_B.index - obj.el_A.index - (-sign(obj.el_A.index) + 1)))];
            j_j_degree = [zeros(1, 3*(obj.el_B.index - 1)) zeros(1, 2) ...
                -1 zeros(1, 3*( obj.el_A.index - obj.el_B.index - (-sign(obj.el_B.index) + 1)))];
            
            Jacobi = sign(obj.el_A.index)*j_i_degree + sign(obj.el_B.index)*j_j_degree; 
        end
        function Gamma = getGamma(obj)
            Gamma = obj.f_bis( obj.el_A.solver.time );
        end
        function Phi_prim = getPhiPrim(obj)
            Phi_prim = -obj.f_prim( obj.el_A.solver.time );
        end
    end
    
end

