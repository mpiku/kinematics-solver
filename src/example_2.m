% #------------------------------------------------------------------------
% EXAMPLE_2
% This is an example presenting possibilities of grapher.trajectory()
% function.
% #------------------------------------------------------------------------
clear; clear element; clear indexer; % Workspace cleanup (additional clears just in case)

% Buld points mesh
u=[1 0]'; alpha_1=deg2rad(60); alpha_2=deg2rad(45); alpha_3=deg2rad(35); ratio=2;
a=4; b=9; c=1;
A=[0 0]'; B=rot(alpha_1)*u*a; C=B+rot(alpha_3)*u*b; D=B+0.7*(C-B);
E=[7 0]'; F=E+rot(alpha_2)*u*c; G=F+0.3*(C-B); H=C+1.5*(G-C);
C1=.5*(B-A); C2=E+.5*(F-E); C3=B+.5*(C-B); C4=D+.5*(F-D); C5=F+.5*(G-F); C6=C+.5*(H-G);

% Initialize solver
solver = solver();
ground = solver.getGround(); ground.addPoint(A); ground.addPoint(E);

% Define elements of the mechanism
el_1 = element(C1); el_1.addPoint(A); el_1.addPoint(B);
el_2 = element(C2); el_2.addPoint(E); el_2.addPoint(F);
el_3 = element(C3); el_3.addPoint(B); el_3.addPoint(C); el_3.addPoint(D);
el_4 = element(C4); el_4.addPoint(F); el_4.addPoint(D);
el_5 = element(C5); el_5.addPoint(F); el_5.addPoint(G);
el_6 = element(C6); el_6.addPoint(C); el_6.addPoint(G); el_6.addPoint(H);

% Define kinematics constraints
el_1.add_K_JointConstr(ground, A);
el_1.add_K_JointConstr(el_3, B);
el_2.add_K_JointConstr(ground, E);
el_2.add_K_JointConstr(el_4, F);
el_2.add_K_JointConstr(el_5, F);
el_3.add_K_JointConstr(el_4, D);
el_3.add_K_JointConstr(el_6, C);
el_5.add_K_JointConstr(el_6, G);

% Define driving constraints
el_1.add_D_JointConstr(ground, A, @(t) -t*1, @(t) -1, @(t) 0);
el_2.add_D_JointConstr(ground, E, @(t) t*ratio*1, @(t) 1*ratio, @(t) 0);

solver.drawMechanism();

% Solve mechianism
solver.solve([0:0.05:2*pi]);

% Graph trajectory of point H in element 6
grapher.trajectory(el_6, H);