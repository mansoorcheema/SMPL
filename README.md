SMPL C++ implementation - realtime mesh deformation and animation (joint orientation is obtained using kinect). 

#### Dependencies
- Eigen 3
- FLTK with OpenGL
- JsonCpp for parsing SMPL config
- Pinnochio (included) for legacy support for auto rigging


### Steps

- Get SMPL template model w/ parameters from https://smpl.is.tue.mpg.de/ . You need to register to download it.
- Download the python version named SMPL_python_v.1.0.0.zip and extract it.
- The SMPL template model and blend shapes along with parameters can be found in the extracted folder under SMPL_python_v.1.0.0/smpl/models directory.
- Convert the data from pickle to simpler format like json using python script which can be found [here](https://github.com/YeeCY/SMPLpp/blob/master/SMPL%2B%2B/scripts/preprocess.py).
- A useful python implementation of SMPL can be found at  https://github.com/CalciferZh/SMPL where numpy version is quite straightforward.
- A detailed description about transformation in world and local coordinates for SMPL skeleton joints can be found [here](https://github.com/YeeCY/SMPLpp/blob/master/SMPL%2B%2B/src/smpl/WorldTransformation.cpp).

### Implementation

- SMPL implementation can be found under `Project` directory
- SMPL Algorithm along with LBS can be found at Project/defmesh.h
- Update the makefile in `Project` Directory with the paths for the specified libraries
- Compile using make command in main directory

- Run this code using command 
`./Project data/smpl_np.obj data/joints.out data/skeleton.out data/smpl_male.json`

### SMPL 
Press `l` to toggle smpl corrections added to model and   `t` to reset.
- Supports real-time blendshapes update.
- Update Model height, weight, muscles etc using simple controls by pressing keys from 1 to 9.
    - 1 controls height, 2 controls Weight and 3 -9 keys control different muscles and body shapes like shoulder, chest etc
    - Press b to toggle increase and decrease (by default increase)


