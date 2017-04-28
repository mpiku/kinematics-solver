# 2-D Kinematics Solver
2-D kinematics solver project created for Multibody Dynamics class at Faculty of Power and Aeronautical Engineering WUT, Warsaw. This piece of software is written in OOP Matlab with idea of easiness of future development. It was created in MATLAB R2016a software on academic licence.
## How to get started?
To see basics of usage of this little toolbox run script *example_1.m* in MATLAB
```matlab
example_1
```
Core class of the software is *solver* class. To start creating your mechanism always initialize it:
```matlab
solver = solver();
```
In order to obtain ground element, which certainly will be needed to build and connect the rest of mechanism, use function *solver.getGround()*:
```matlab
ground = solver.getGround();
```
To familiarize with use of the software please study *example_1.m* - you will *quickly* get the idea! As for now, documentation is not available in any common format but it should be created soon. If you need to get to know how this software works on deeper level do not hesitate to check in code - each *class*, *function* and *selected blocks of code* are described in a constant way.

**Code or architecture might not be perfect - it started with just a fun session of freestyle programming!**

![N|Solid](https://www.meil.pw.edu.pl/design/common_coi1/images/logo_meil_80px.png)