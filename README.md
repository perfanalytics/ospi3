# ospi
This library contains scripts for working with OpenSim files and pinocchio software. 

Loading OpenSim3.3 '.osim' model in Pinocchio as well as .trc and . mot file.

## Modifications from original package OSPI: 
- compatibily with python3
- fix knee-joint spline parsing
- add missing meshes (convert OpenSim .vtp in .obj thanks to vtp2obj.py code)
- add parsing of MarketSet included in .osim or in external .xml file. The markerset is included in the model wrapper.
- add parsing and visualisation of .trc file with Mocap data 
- add parsing and visualisation of ground force file 


## Limits:

- Complex Joints (e.g with spline) can't be converted faithfully since Pinocchio doens't support them. Complex knee joint with spline is simplified to revolute joint.
- Loading file from Opensim 4.X would require more code modifications since the .osim format is different between 3.X and 4.X version.
- Model scaling is not available. It must be done once on OpenSim and then use the scaled model in Pinocchio with OSPi3 package.

## Required Dependencies:
- Python 3. Additional libraries are directly handled by setup.py (numpy, math and elementtree,...)
- Pinocchio: library for rigid multi-body dynamics. Github: http://stack-of-tasks.github.io/pinocchio/ 

```json
   $ conda install pinocchio -c conda-forge
```

- Gepetto-viewer: A graphical interface for pinocchio. Github: https://github.com/humanoid-path-planner/gepetto-viewer.git

```json
   $ conda install gepetto-viewer gepetto-viewer-corba -c conda-forge
```

## Installation
```json
   $ python setup.py install --prefix='your install path'
```
