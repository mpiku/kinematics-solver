classdef kPrismConstr < kinSolver.constraint
    %KPRISMCONSTR Prismatic constraint - kinematic constraint
    % Implements driving constraint (para postêpowa - wiêz kinematyczny).
    properties
        el_A = 0; % element A
        el_B = 0; % element B
        
        A_index = 0; % point's index in el. A
        B_index = 0; % point's index in el. B
        
        s_A = 0; % local vector of point A
        s_B = 0; % local vector of point B
        
        v_B = 0; % vector parallel to the axis of movement
    end
    
    methods
      function obj = kPrismConstr(el_A, A_i, el_B, B_i, v_B)
            obj.el_A = el_A;
            obj.A_index = A_i;
            
            obj.el_B = el_B;
            obj.B_index = B_i;
            
            obj.v_B = v_B;
            
            obj.s_A = cell2mat( el_A.cell_points( A_i ) );
            obj.s_B = cell2mat( el_B.cell_points( B_i ) );
      end
      function Phi = getConstraint(obj)
          Phi = obj.el_A.fi_c - obj.el_B.fi_c;
          Phi = [Phi; ...
              (obj.el_B.r_c + kinSolver.rot(obj.el_B.fi_c) * obj.s_B + ...
              - obj.el_A.r_c - kinSolver.rot(obj.el_A.fi_c) * obj.s_A)' * kinSolver.rot(obj.el_B.fi_c) * obj.v_B];
      end
      function Jacobi = getJacobi(obj)
          j_i_degree = [zeros(1, 3*(obj.el_A.index - 1)) zeros(1, 2) ...
              1 zeros(1, 3*( obj.el_B.index - obj.el_A.index - (-sign(obj.el_A.index) + 1)))];
          j_j_degree = [zeros(1, 3*(obj.el_B.index - 1)) zeros(1, 2) ...
              -1 zeros(1, 3*( obj.el_A.index - obj.el_B.index - (-sign(obj.el_B.index) + 1)))];
          
          R_j_v_j_T = (kinSolver.rot(obj.el_B.fi_c)*obj.v_B)';
          j_i_transl = [zeros(1, 3*(obj.el_A.index - 1)) -R_j_v_j_T ...
              -R_j_v_j_T*obj.omega*kinSolver.rot(obj.el_A.fi_c)*obj.s_A ...
              zeros(1, 3*( obj.el_B.index - obj.el_A.index - (-sign(obj.el_A.index) + 1)))];
          j_j_transl = [zeros(1, 3*(obj.el_B.index  - 1)) R_j_v_j_T ...
              -R_j_v_j_T*obj.omega*(obj.el_B.r_c - obj.el_A.r_c - kinSolver.rot(obj.el_A.fi_c)*obj.s_A) ...
              zeros(1, 3*( obj.el_A.index - obj.el_B.index - (-sign(obj.el_B.index) + 1)))];
          
          j_i = [j_i_degree; j_i_transl]; j_j = [j_j_degree; j_j_transl];
          Jacobi = sign(obj.el_A.index)*j_i + sign(obj.el_B.index)*j_j; 
      end
      function Gamma = getGamma(obj)
          Gamma(1, 1) = 0;
          Gamma(2, 1) = (kinSolver.rot( obj.el_B.fi_c ) * obj.v_B)' * ...
              (2*obj.omega*( obj.el_B.r_c_prim - obj.el_A.r_c_prim )*obj.el_B.fi_c_prim + ...
              ( obj.el_B.r_c - obj.el_A.r_c )*(obj.el_B.fi_c_prim^2) + ...
              - kinSolver.rot( obj.el_A.fi_c )*obj.s_A*( (obj.el_B.fi_c_prim - obj.el_A.fi_c_prim)^2 ));
      end
      function Phi_prim = getPhiPrim(obj)
          Phi_prim = zeros(2, 1);
      end
    end
    
end

