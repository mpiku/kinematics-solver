classdef dPrismConstr < kinSolver.constraint
    %DJOINTCONSTR Prismatic constraint - driving constraint
    % Implements driving constraint (para postêpowa - wiêz kieruj¹cy).
    properties
        el_A = 0; % element A
        el_B = 0; % element B
        
        A_index = 0; % point's index in el. A
        B_index = 0; % point's index in el. B
        
        s_A = 0; % local vector of point A
        s_B = 0; % local vector of point B
        
        u_B = 0; % vector perpendicular to the axis of movement
        
        % Driving function and its derivatives
        f = 0;
        f_prim = 0;
        f_bis = 0;
    end
    
    methods
      function obj = dPrismConstr(el_A, A_i, el_B, B_i, u_B, f, f_prim, f_bis)
            obj.el_A = el_A;
            obj.A_index = A_i;
            
            obj.el_B = el_B;
            obj.B_index = B_i;
            
            obj.u_B = u_B;
            
            obj.s_A = cell2mat( el_A.cell_points( A_i ) );
            obj.s_B = cell2mat( el_B.cell_points( B_i ) );
            
            obj.f = f;
            obj.f_prim = f_prim;
            obj.f_bis = f_bis;
      end
      function Phi = getConstraint(obj)
          Phi = (obj.el_B.r_c + kinSolver.rot(obj.el_B.fi_c) * obj.s_B + ...
              - obj.el_A.r_c - kinSolver.rot(obj.el_A.fi_c) * obj.s_A)' * ...
              kinSolver.rot(obj.el_B.fi_c) * obj.u_B - obj.f( obj.el_A.solver.time );
      end
      function Jacobi = getJacobi(obj)
          R_j_v_j_T = (kinSolver.rot(obj.el_B.fi_c)*obj.u_B)';
          j_i_transl = [zeros(1, 3*(obj.el_A.index - 1)) -R_j_v_j_T ...
              -R_j_v_j_T*obj.omega*kinSolver.rot(obj.el_A.fi_c)*obj.s_A ...
              zeros(1, 3*( obj.el_B.index - obj.el_A.index - (-sign(obj.el_A.index) + 1)))];
          j_j_transl = [zeros(1, 3*(obj.el_B.index  - 1)) R_j_v_j_T ...
              -R_j_v_j_T*obj.omega*(obj.el_B.r_c - obj.el_A.r_c - kinSolver.rot(obj.el_A.fi_c)*obj.s_A) ...
              zeros(1, 3*( obj.el_A.index - obj.el_B.index - (-sign(obj.el_B.index) + 1)))];
          
          Jacobi = sign(obj.el_A.index)*j_i_transl + sign(obj.el_B.index)*j_j_transl; 
      end
      function Gamma = getGamma(obj)
          Gamma = obj.f_bis( obj.el_A.solver.time );
      end
      function Phi_prim = getPhiPrim(obj)
          Phi_prim = -obj.f_prim( obj.el_A.solver.time );
      end
    end
    
end

