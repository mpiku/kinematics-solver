classdef kJointConstr < kinSolver.constraint
    %KJOINTCONSTR Joint constraint - kindematic constraint
    % Implementation of joint constraint (pol. para obrotowa wiêz
    % kinematyczny).
    properties
        el_A = 0; % element A
        el_B = 0; % element B
        
        A_index = 0; % point's index in el. A
        B_index = 0; % point's index in el. B
        
        s_A = 0; % local vector of point A
        s_B = 0; % local vector of point B
    end
    
    methods
        function obj = kJointConstr(el_A, A_i, el_B, B_i)
            obj.el_A = el_A;
            obj.A_index = A_i;
            
            obj.el_B = el_B;
            obj.B_index = B_i;
            
            obj.s_A = cell2mat( el_A.cell_points( A_i ) );
            obj.s_B = cell2mat( el_B.cell_points( B_i ) );
        end
        function Phi = getConstraint(obj)
            Phi = obj.el_A.r_c + kinSolver.rot( obj.el_A.fi_c ) * obj.s_A + ...
                - ( obj.el_B.r_c + kinSolver.rot( obj.el_B.fi_c ) * obj.s_B );
        end
        function Jacobi = getJacobi(obj)
            j_i = [zeros(2, 3*(obj.el_A.index - 1)) eye(2) ...
                obj.omega * kinSolver.rot( obj.el_A.fi_c ) * obj.s_A ...
                zeros(2, 3*( obj.el_B.index - obj.el_A.index - (-sign(obj.el_A.index) + 1)))];
            j_j = [zeros(2, 3*(obj.el_B.index  - 1)) -eye(2) ...
                -obj.omega * kinSolver.rot( obj.el_B.fi_c ) * obj.s_B ...
                zeros(2, 3*( obj.el_A.index - obj.el_B.index - (-sign(obj.el_B.index) + 1)))];
            % sign() function is needed to turn off extending matrices for
            % ground element.
            Jacobi = sign(obj.el_A.index)*j_i + sign(obj.el_B.index)*j_j;
            % If one of the elements is the ground (element.index == 0), it
            % will not be added to Jacobi.
        end
        function Gamma = getGamma(obj)
            Gamma = kinSolver.rot( obj.el_A.fi_c ) * obj.s_A * ( obj.el_A.fi_c_prim^2 ) + ...
                - kinSolver.rot( obj.el_B.fi_c ) * obj.s_B * ( obj.el_B.fi_c_prim^2 );
        end
        function Phi_prim = getPhiPrim(obj)
            Phi_prim = zeros(2, 1);
        end
    end
end

