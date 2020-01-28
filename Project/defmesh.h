#ifndef DEFMESH_H_INCLUDED
#define DEFMESH_H_INCLUDED

#include "../Pinocchio/attachment.h"
#include "motion.h"
#include <json/json.h>
#include <map>

#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>


struct JointTransformation {
    Eigen::Quaternion<double> rot;
    Eigen::Vector3d parent;
    Eigen::Vector3d childOld;
    Eigen::Vector3d childNew;
};


class SMPL
{
private:
    Eigen::MatrixXd templateMeshVertices;
    Eigen::MatrixXd joint_regressor;
    Eigen::MatrixXd weights;
    Eigen::MatrixXd shape_blend_shapes;
    Eigen::MatrixXd pose_blend_shapes;
    Eigen::MatrixXd boneTree;
    Eigen::MatrixXd shapeBlends;
    Eigen::MatrixXd regressedJoints;
    std::vector<Vector3> joints;
    Eigen::VectorXd beta;
    Eigen::VectorXd thetas;
    std::vector<Eigen::MatrixXd> transformationMatrices;
    Eigen::MatrixXd poseBlends;
    bool active;

public:
    SMPL(const Eigen::MatrixXd &inVertTemp, const Eigen::MatrixXd &inJointReg, const Eigen::MatrixXd &inWeights,
         const Eigen::MatrixXd &inShapeBlendShapes, const Eigen::MatrixXd &inPoseBlendShapes, const Eigen::MatrixXd &inBonesOrder)
        : templateMeshVertices(inVertTemp), joint_regressor(inJointReg), weights(inWeights),
          shape_blend_shapes(inShapeBlendShapes), pose_blend_shapes(inPoseBlendShapes), boneTree(inBonesOrder)
    {
        // Eigen::VectorXd v(10);
        // v<<-5,-5,-5,-5,-5,-5,-4,-5,-5,-5;
        updateBetas(Eigen::VectorXd::Zero(10));
        thetas = Eigen::VectorXd::Zero(pose_blend_shapes.cols() - 1);
        active = true;
    }

    // Load SMPL learned parameters, weights, blend shapes
    static SMPL *loadSMPL(const string &path)
    {
        Json::Value obj;
        std::ifstream ifs;
        ifs.open(path);
        Json::Reader reader;
        reader.parse(ifs, obj);

        Eigen::MatrixXd templateMeshVertices = getEigenMatrix(obj["vertices_template"]);
        Eigen::MatrixXd joint_regressor = getEigenMatrix(obj["joint_regressor"]);
        Eigen::MatrixXd weights = getEigenMatrix(obj["weights"]);
        Eigen::MatrixXd shape_blend_shapes = getBlendShapes(obj["shape_blend_shapes"]);
        Eigen::MatrixXd pose_blend_shapes = getBlendShapes(obj["pose_blend_shapes"]);
        Eigen::MatrixXd kinematic_tree = getEigenMatrix(obj["kinematic_tree"]);

        SMPL *smpl = new SMPL(templateMeshVertices, joint_regressor, weights, shape_blend_shapes, pose_blend_shapes, kinematic_tree);
        return smpl;
    }

    // get parent of a bone
    int getParent(const int i)
    {
        return boneTree(0, i);
    }

    void toggle()
    {
        active = !active;
        std::cout<<"SMPL "<<active?"Enabled\n":"Disabled\n";
    }

    // update with new joint angles
    void updateTheta(const std::vector<Eigen::Quaternionf> &rots)
    {
        //reshape pose thetas
        auto IdentityMatrix = Eigen::Quaternionf::Identity().toRotationMatrix();
        Eigen::MatrixXd poses((boneTree.cols() - 1) * IdentityMatrix.size(), 1);
        for (int i = 1; i < boneTree.cols(); i++)
        {
            Eigen::Matrix<float, 3, 3, Eigen::RowMajor> M = (rots[i].toRotationMatrix() - IdentityMatrix);
            Eigen::Map<Eigen::VectorXf> rotationsAsColumn(M.data(), M.size());

            poses.block((i - 1) * 9, 0, 9, 1) = rotationsAsColumn.cast<double>();
        }
        thetas = poses;

        //apply pose shapes
        poseBlends = Eigen::Map<Eigen::MatrixXd>(Eigen::MatrixXd(pose_blend_shapes * poses).data(), shapeBlends.rows(), shapeBlends.cols());

        //
        updateTransforms(rots);
    }

    // beta controls how much ration to pick from one of the basis blend shapes
    void updateBetas(const Eigen::VectorXd &betas)
    {
        beta = betas;
        shapeBlends = shape_blend_shapes * beta;
        Eigen::Map<Eigen::MatrixXd> reshapedBlendShapes(shapeBlends.data(), 6890, 3);
        shapeBlends = reshapedBlendShapes;
        regressedJoints = joint_regressor * (templateMeshVertices + shapeBlends);

        //
        joints.clear();
        for (size_t i = 0; i < regressedJoints.rows(); i++)
        {
            Vector3 v(regressedJoints(i, 0), regressedJoints(i, 1), regressedJoints(i, 2));
            joints.push_back(v);
        }
    }

    void incBeta(int no) 
    {
        Eigen::VectorXd b = beta;
        b(no) = b(no) + 0.1;
        updateBetas(b);
    }

    void decBeta(int no) 
    {
        Eigen::VectorXd b = beta;
        b(no) = b(no) - 0.1;
        updateBetas(b);
    }

    void resetBeta()
    {
        updateBetas(Eigen::VectorXd::Zero(10));
    }


    Eigen::Vector4d getMeshVertexCoords(const uint32_t i)
    {
        Eigen::Vector3d v = templateMeshVertices.row(i);
        // if smpl is active, add corretions from shape blend shapes and 
        // pose blend shapes
        if (active)
        {
            v += (shapeBlends.row(i) + poseBlends.row(i));
        }

        return Eigen::Vector4d(v.x(), v.y(), v.z(), 1);
    }


    // Returns joint location in world transform of shape (24,3)
    vector<Vector3> getJoints()
    {
        return joints;
    }


    // Return 4x4 TransformationMatrix for each joint,
    // in World coordinates, it is multiplied by vertices
    // in world coordinates
    Eigen::MatrixXd G(const uint32_t i)
    {
        return transformationMatrices[i];
    }


    // weights for each joint and for each vertex
    double getWeights(const int vertex, const int bone)
    {
        return weights(vertex, bone);
    }

    private:

    //---------------------------------TRANSFORMATION--------------------------------------------------+
    // Update transformation matrices based on new observed rotaton                                    |
    // Input is relative rotations for each bone wrt rest pose,in local coordinate system              |
    // wrt to its parent.                                                                              |
    //                                                                                                 |
    //      \            \                                                                             |
    //       *            *                                                                            |
    //       |    --->     \ rotation for 2nd bone wrt to second bone in rest pose                     |
    //       *              *                                                                          |
    //        \              \ (zero rotation wrt to bone 3 in rest pose (local)) still it has         |
    //                          rotation wrt respective bone in world coordinates                      |
    //                          because of parent bone rotation.                                       |
    //-------------------------------------------------------------------------------------------------+
    void updateTransforms(const std::vector<Eigen::Quaternionf> &rots)
    {
        transformationMatrices.clear();
        //calculate translation matrices for each bone
        Eigen::MatrixXd R0 = Eigen::MatrixXd::Zero(4, 4);
        R0.block(0, 0, 3, 3) = rots[0].toRotationMatrix().cast<double>();
        R0.block(0, 3, 3, 1) = regressedJoints.row(0).transpose();
        R0(3, 3) = 1;
        transformationMatrices.push_back(R0);

        for (size_t i = 1; i < boneTree.cols(); i++)
        {
            Eigen::MatrixXd Ri = Eigen::MatrixXd::Zero(4, 4);
            Ri.block(0, 0, 3, 3) = rots[i].toRotationMatrix().cast<double>();
            Ri.block(0, 3, 3, 1) = (regressedJoints.row(i) - regressedJoints.row(getParent(i))).transpose();
            Ri(3, 3) = 1;
            transformationMatrices.push_back(transformationMatrices[getParent(i)] * Ri);
        }

        // Remove transformations due to rest pose
        for (size_t i = 0; i < boneTree.cols(); i++)
        {
            Eigen::MatrixXd Ri = transformationMatrices[i];
            Eigen::Vector4d j = Eigen::Vector4d::Zero(4);
            j.block(0, 0, 3, 1) = regressedJoints.row(i).transpose();

            Eigen::MatrixXd t = Eigen::MatrixXd::Zero(4, 4);
            t.col(3) = Ri * j;
            transformationMatrices[i] -= t;
        }
    }

    //load 2nd Matrices for smpl
    static Eigen::MatrixXd getEigenMatrix(const Json::Value &obj)
    {
        Eigen::MatrixXd A(obj.size(), obj[0].size());

        for (int i = 0; i < obj.size(); i++)
        {
            for (int j = 0; j < obj[i].size(); j++)
            {
                A(i, j) = obj[i][j].asDouble();
            }
        }
        return A;
    }

    ////load function parameters for smpl
    // Tranined reference weights provided by smpl
    static Eigen::MatrixXd getBlendShapes(const Json::Value &obj)
    {
        Eigen::MatrixXd A(obj.size() * obj[0].size(), obj[0][0].size());

        for (int i = 0; i < obj.size(); i++)
        {
            for (int j = 0; j < obj[i].size(); j++)
            {
                for (int k = 0; k < obj[0][0].size(); k++)
                {
                    A(j * obj.size() + i, k) = obj[i][j][k].asDouble();
                }
            }
        }
        return A;
    }
};

class DeformableMesh
{
public:
 

    DeformableMesh(const Mesh inMesh, const Skeleton &inOrigSkel, const vector<Vector3> &inMatch,
            const Attachment &inAttachment, Motion *inMotion = NULL, SMPL * inSMPL = NULL) 
      : origSkel(inOrigSkel), match(inMatch), origMesh(inMesh), motion(inMotion),smpl(inSMPL) {

    }
      
    const int size() const {
        return motion->positions.size();
    };

    vector<Vector3> getSkel() const;
    vector<Vector3> getAvatarSkeleton();
    vector<Eigen::Quaternionf> getBasePose() const { return pose; };
    const Skeleton &getOrigSkel() const { return origSkel; }
    const Attachment &getAttachment() const { return attachment; }
    const Mesh & getMesh() { 
        updateMesh();
        return curMesh; 
    }

    // Model customization using SMPL. Beta controls parameters like
    // height, weight, muscles, etc
    void incBeta(int no)  { smpl->incBeta(no); }
    void decBeta(int no)  { smpl->decBeta(no); }
    void resetBeta() { smpl->resetBeta(); }

    mutable vector<Vector3> match;
    vector<Eigen::Quaternionf> pose;
    map<int, Eigen::Transform<float, 3, Eigen::Affine>> computeTransforms() const;
    void toggleSMPL();
    int getJointParent(int i) { return smpl->getParent(i);}
private:
    void updateMesh() const;

    Skeleton origSkel;
    Attachment attachment;
    mutable Mesh origMesh;
    mutable Mesh curMesh;
    Motion *motion;
    SMPL *smpl;
};

#endif //DEFMESH_H_INCLUDED
