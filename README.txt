SMPL C++ implementation using Eigen, FLTK library with OpenGL, jsoncpp. Compile and install jsoncpp and update the makefile found in the "Project" directory.

Steps
======
1) Get SMPL Models from https://smpl.is.tue.mpg.de/ by registering.
2) Download the python version named SMPL_python_v.1.0.0.zip and extract it.
3) The model along with parameters can be found files in SMPL_python_v.1.0.0/smpl/models.
4) Convert SMPL weights from pickle to more reasonable frormat like json using SMPLpp which can be downloadded from https://github.com/YeeCY/SMPLpp
5) A useful python implementation of SMPL can be found at  https://github.com/CalciferZh/SMPL where numpy version is quite straightforward.
6) Run this code using command 
./Project data/smpl_np.obj data/joints.out data/skeleton.out data/smpl_male.json
Press l to toggle smpl corrections added to model.


