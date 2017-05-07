% #------------------------------------------------------------------------
% EXAMPLE_4
% Another example with more complex mechanism
% #------------------------------------------------------------------------
clear; clear kinSolver.element; clear kinSolver.indexer; % Workspace cleanup (additional clears just in case)

% Build points mesh
A=[0 0]';B=[0.3 0]';D=[0.2 0.6]';E=[-0.1 0.7]';F=[-0.2 0.8]';G=[0 1.2]';
H=[.5 .8]';I=[.5 -0.1]';J=[0.7 0.3]';K=[0.8 .6]';L=[1 -0.5]';M=[1.3 -.3]';N=[1.3 0]';
C1=[0 .85]';C2=[.4 .75]';C3=[.15 .45]';C4=[.05 .15]';C5=[.45 .6]';
C6=[.35 .2]';C7=[.4 .1]';C8=[.65 0.25]';C9=[1.05 .3]';C10=[1.2 -.25]';

% Initialize solver
solver = kinSolver.solver();
ground = solver.getGround(); ground.addPoint(A); ground.addPoint(B);
ground.addPoint(E);

% Define elements of the mechanism
el_1 = kinSolver.element(C1); el_1.addPoint(D); el_1.addPoint(E); el_1.addPoint(G);
el_1.addPoint(F);
el_2 = kinSolver.element(C2); el_2.addPoint(G); el_2.addPoint(H); el_2.addPoint(J);
el_3 = kinSolver.element(C3); el_3.addPoint(D);
el_4 = kinSolver.element(C4); el_4.addPoint(A);
el_5 = kinSolver.element(C5); el_5.addPoint(H);
el_6 = kinSolver.element(C6); el_6.addPoint(B);
el_7 = kinSolver.element(C7); el_7.addPoint(F); el_7.addPoint(I); el_7.addPoint(L);
el_8 = kinSolver.element(C8); el_8.addPoint(I); el_8.addPoint(J); el_8.addPoint(K);
el_9 = kinSolver.element(C9); el_9.addPoint(K); el_9.addPoint(N);
el_10 = kinSolver.element(C10); el_10.addPoint(N); el_10.addPoint(M); el_10.addPoint(L);

% Define kinematics constraints
el_1.add_K_JointConstr(ground, E); el_1.add_K_JointConstr(el_2, G);
el_1.add_K_JointConstr(el_3, D); el_1.add_K_JointConstr(el_7, F);
el_2.add_K_JointConstr(el_5, H); el_2.add_K_JointConstr(el_8, J);
el_3.add_K_PrismConstr(el_4, D, A);
el_4.add_K_JointConstr(ground, A);
el_5.add_K_PrismConstr(el_6, H, B);
el_6.add_K_JointConstr(ground, B);
el_7.add_K_JointConstr(el_8, I); el_7.add_K_JointConstr(el_10, L);
el_8.add_K_JointConstr(el_9, K);
el_9.add_K_JointConstr(el_10, N);

% Define driving constraints
l_3 = norm(D-A)-.1; a_3 = .1; omega_3 = 1; fi_3 = 0;
el_3.add_D_PrismConstr(el_4, D, A, @(t) l_3 + a_3*sin(omega_3*t + fi_3), ...
    @(t) a_3*omega_3*cos(omega_3*t + fi_3), ...
    @(t) -a_3*omega_3*omega_3*sin(omega_3*t + fi_3));
l_5 = norm(H-B)-.1; a_5 = .1; omega_5 = 2; fi_5 = 0;
el_5.add_D_PrismConstr(el_6, H, B, @(t) l_5 + a_5*sin(omega_5*t + fi_5), ...
    @(t) a_5*omega_5*cos(omega_5*t + fi_5), ...
    @(t) -a_5*omega_5*omega_5*sin(omega_5*t + fi_5));

%solver.getJacobi(); % Uncomment to get Jacobian matrix for drawn mechanism

% Solve mechianism
solver.solve([0:.05:2*pi]);

solver.animateSolution;