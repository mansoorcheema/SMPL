SMPL C++ implementation using Eigen, FLTK library with OpenGL, jsoncpp. Compile and install jsoncpp and update the makefile found in the "Project" directory.

### Steps

- Get SMPL Models from https://smpl.is.tue.mpg.de/ . you Need to register to download it.
- Download the python version named SMPL_python_v.1.0.0.zip and extract it.
- The SMPL template model and blend shapes along with parameters can be found in the extracte folder under SMPL_python_v.1.0.0/smpl/models directory.
- Convert the data from pickle to more reasonable frormat like json using python script which can be found [here](https://github.com/YeeCY/SMPLpp/blob/master/SMPL%2B%2B/scripts/preprocess.py).
- A useful python implementation of SMPL can be found at  https://github.com/CalciferZh/SMPL where numpy version is quite straightforward.
- A detailed concept about transformation in world and local coordinates can be found [here](https://github.com/YeeCY/SMPLpp/blob/master/SMPL%2B%2B/src/smpl/WorldTransformation.cpp).
- Run this code using command 
`./Project data/smpl_np.obj data/joints.out data/skeleton.out data/smpl_male.json`
Press l to toggle smpl corrections added to model.


