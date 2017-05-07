% #------------------------------------------------------------------------
% EXAMPLE_3
% Another example with more complex mechanism
% #------------------------------------------------------------------------
clear; clear kinSolver.element; clear kinSolver.indexer; % Workspace cleanup (additional clears just in case)

% Build points mesh
A=[-1 .2]';B=[-.7 .8]';D=[-.2 .5]';E=[.1 .8]';F=[.9 .7]';G=[.3 .6]';
H=[.7 -.4]';I=[.7 .3]';J=[0 .9]';K=[-1.2 .6]';L=[.6 .3]';M=[.2 .6]';N=[.4 -.2]';
C1=[-.95 .55]';C2=[.4 .75]';C3=[.6 .5]';C4=[-.15 .35]';C5=[-.35 .85]';
C6=[-.05 .65]';C7=[.25 .4]';C8=[.35 0]';C9=[.4 .35]';C10=[.6 -.15]';

% Initialize solver
solver = kinSolver.solver();
ground = solver.getGround(); ground.addPoint(N); ground.addPoint(H);
ground.addPoint(L);

% Define elements of the mechanism
el_1 = kinSolver.element(C1); el_1.addPoint(A); el_1.addPoint(B); el_1.addPoint(K);
el_2 = kinSolver.element(C2); el_2.addPoint(J); el_2.addPoint(F); el_2.addPoint(M);
el_2.addPoint(E);
el_3 = kinSolver.element(C3); el_3.addPoint(I); el_3.addPoint(L); el_3.addPoint(G);
el_3.addPoint(F);
el_4 = kinSolver.element(C4); el_4.addPoint(A); el_4.addPoint(D); el_4.addPoint(I);
el_5 = kinSolver.element(C5); el_5.addPoint(B); el_5.addPoint(J);
el_6 = kinSolver.element(C6); el_6.addPoint(D); el_6.addPoint(E);
el_7 = kinSolver.element(C7); el_7.addPoint(M);
el_8 = kinSolver.element(C8); el_8.addPoint(N);
el_9 = kinSolver.element(C9); el_9.addPoint(G);
el_10 = kinSolver.element(C10); el_10.addPoint(H);

% Define kinematics constraints
el_1.add_K_JointConstr(el_4, A); el_1.add_K_JointConstr(el_5, B);
el_2.add_K_JointConstr(el_3, F); el_2.add_K_JointConstr(el_5, J);
el_2.add_K_JointConstr(el_6, E); el_2.add_K_JointConstr(el_7, M);
el_3.add_K_JointConstr(ground, L); el_3.add_K_JointConstr(el_4, I);
el_3.add_K_JointConstr(el_9, G);
el_4.add_K_JointConstr(el_6, D);
el_7.add_K_PrismConstr(el_8, M, N);
el_8.add_K_JointConstr(ground, N);
el_9.add_K_PrismConstr(el_10, G, H);
el_10.add_K_JointConstr(ground, H);

% Define driving constraints
l_7 = norm(M-N)-.2; a_7 = .2; omega_7 = 1; fi_7 = 0;
el_7.add_D_PrismConstr(el_8, M, N, @(t) l_7 + a_7*sin(omega_7*t + fi_7), ...
    @(t) a_7*omega_7*cos(omega_7*t + fi_7), ...
    @(t) -a_7*omega_7*omega_7*sin(omega_7*t + fi_7));
l_9 = norm(G-H)-.2; a_9 = -.2; omega_9 = 2; fi_9 = 0;
el_9.add_D_PrismConstr(el_10, G, H, @(t) l_9 + a_9*sin(omega_9*t + fi_9), ...
    @(t) a_9*omega_9*cos(omega_9*t + fi_9), ...
    @(t) -a_9*omega_9*omega_9*sin(omega_9*t + fi_9));

% Solve mechianism
solver.solve([0:.2:2*pi]);

solver.drawMechanism();