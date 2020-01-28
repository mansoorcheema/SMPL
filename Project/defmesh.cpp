#include "defmesh.h"
#include "../Pinocchio/intersector.h"


Eigen::Vector3f transformToEigen(const Vector3 & vec) {
    return Eigen::Vector3f(vec[0], vec[1], vec[2]);
} 

Vector3 transformToVec(const Eigen::Vector3f & vec) {
    return Vector3(vec.x(), vec.y(), vec.z());
}

Eigen::Vector3f getDirection(const Rotation & rot) {
    Eigen::Quaternionf quart(-rot.rotation[3], rot.rotation[0], rot.rotation[1], -rot.rotation[2]);

    return quart * Eigen::Vector3f(0, 1, 0);
}

Eigen::Quaternionf getRootRotation(const vector<Vector3> & pose, const Rotation & rot) {
    Eigen::Vector3f directionRoot = getDirection(rot);
    Eigen::Quaternionf rootRotation = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 1, 0), directionRoot);

    Eigen::Vector3f parent = transformToEigen(pose[0]);
    Eigen::Vector3f child = transformToEigen(pose[0]);

    Eigen::Vector3f directionPose = child - parent;
    Eigen::Vector3f directionAvatar = getDirection(rot);

    return Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f(0, 1, 0), directionAvatar);
}

map<int, Eigen::Transform<float, 3, Eigen::Affine>> DeformableMesh::computeTransforms() const {
    map<int, Eigen::Transform<float, 3, Eigen::Affine>> transforms;
    std::vector<Eigen::Quaternionf> rots;

    static const std::vector<int> kinectToSMPL = {0, 16,12,-1,17,13,1,18,14,-1,19,15,2,-1,-1,3,8,4,9,5,10,6,11,7 };

    const vector<Rotation> & rotations = motion->getRelative();
    vector<Vector3> joints = smpl->getJoints();
    
    Eigen::Quaternionf rootRotation = getRootRotation(joints, rotations[kinectToSMPL[0]]);
    transforms.insert({0, Eigen::Transform<float, 3, Eigen::Affine>(rootRotation)});
    rots.push_back(Eigen::Quaternionf::Identity());

    Eigen::Vector3f root = transformToEigen(joints[0]);

    Rotation idty;

    for(int g = 1; g < joints.size(); g++) {
        const Rotation & rot = kinectToSMPL[g]>=0?rotations[kinectToSMPL[g]]:idty;

        Eigen::Vector3f parent = transformToEigen(joints[smpl->getParent(g)]);
        Eigen::Vector3f child = transformToEigen(joints[g]);

        auto stack = transforms.find(smpl->getParent(g));
        if (stack != transforms.end()) {
            parent = stack->second * parent;
            child = stack->second * child;
        }

        Eigen::Vector3f directionPose = child - parent;

        Eigen::Translation<float, 3> translForward(parent);
        Eigen::Translation<float, 3> translBack = translForward.inverse();

        Eigen::Vector3f directionAvatar = getDirection(rot);

        Eigen::Quaternionf rotation = Eigen::Quaternionf::FromTwoVectors(directionPose, directionAvatar);

        if(kinectToSMPL[g] == -1) {
            rotation= Eigen::Quaternionf::Identity();
        }
        rots.push_back(rotation);

        Eigen::Transform<float, 3, Eigen::Affine> transform =  translForward * rotation * translBack;
        if (transforms.find(smpl->getParent(g)) != transforms.end()) {
            transform = transform * transforms.find(smpl->getParent(g))->second;
        } else {
            transform = transform * rootRotation;
        }

        transforms.insert({g, transform});
    }

    smpl->updateTheta(rots);
    
    return transforms;
}


// Linear Blend Skinning
void DeformableMesh::updateMesh() const {
    map<int, Eigen::Transform<float, 3, Eigen::Affine>> transforms = computeTransforms();

    const vector<Vector3> & positions = motion->getSkeletonBones();
    Eigen::Translation<float, 3> rootTranslation(transformToEigen(positions[0]).array() * Eigen::Vector3f(1.0, 1.0, -3.0).array());

    auto rotAxis = Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY());

    curMesh = origMesh;
    for(int i = 0; i < origMesh.vertices.size(); i++) {
        Eigen::Vector4d updatedPos = Eigen::VectorXd::Zero(4);
        for(int j = 0; j < smpl->getJoints().size(); j++) {
            updatedPos += smpl->getWeights(i,j) * (smpl->G(j) * smpl->getMeshVertexCoords(i));
        }

        curMesh.vertices[i].pos = transformToVec(rootTranslation * updatedPos.block(0,0,3,1).cast<float>());
    }

}


void DeformableMesh::toggleSMPL() {
    smpl->toggle();
}
 
vector<Vector3> DeformableMesh::getAvatarSkeleton() {
    return motion->positions[motion->getFrameIdx()];
}

vector<Vector3> DeformableMesh::getSkel() const {
    vector<Vector3> out = smpl->getJoints();
        
    for(int i = 0; i < (int)out.size(); ++i) {
        // update bone positions by applying rotation and translations wrt orignal skeleton
        Eigen::Vector4d v;
        v<< out[i][0],out[i][1],out[i][2],1;
        v = smpl->G(i) *v;
        out[i] = Vector3(v.x(),v.y(),v.z());
    }
    
    return out;
}
