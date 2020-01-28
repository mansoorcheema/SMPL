#include <FL/Fl.H>
#include "MyWindow.h"
#include "motion.h"
#include "defmesh.h"
#include "../Pinocchio/skeleton.h"
#include "../Pinocchio/pinocchioApi.h"
#include <fstream>
#include <iostream>
#include "eigen3/Eigen/Core"


int main(int argc, char **argv) {
    const string meshPath = argv[1];
    const string skeletonPath = argv[2];
    const string motionPath = argv[3];
    const string smplPath = argv[4];

    MyWindow *window = new MyWindow();
    Mesh mesh(meshPath);
    
    //Skeleton skelton = FileSkeleton(skeletonPath);
    //skelton.scale(0.7);

    // compute joint skin association and weights to bones attachment
    //PinocchioOutput riggedOut = autorig(skelton, mesh);
    SMPL *smpl =  SMPL::loadSMPL(smplPath);
    DeformableMesh defmesh(mesh, Skeleton(), std::vector<Vector3>(), Attachment(), new Motion(motionPath), smpl);

    window->addMesh(& defmesh);
    window->show();

    return Fl::run();
}
