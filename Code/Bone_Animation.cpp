#include "Bone_Animation.h"
#include <glm/gtx/euler_angles.hpp>


Bone_Animation::Bone_Animation()
{
}


Bone_Animation::~Bone_Animation()
{
}

void Bone_Animation::init()
{
    scale_vector =
    {
        {1.0f,1.0f,1.0f},
        {0.5f,4.0f,0.5f},
        {0.5f,3.0f,0.5f},
        {0.5f,2.0f,0.5f}
    };
    
    rotation_degree_vector =
    {
        {0.0f,0.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f}
    };
    
    colors =
    {
        {0.7f,0.0f,0.0f,1.0f},
        {0.7f,0.7f,0.0f,1.0f},
        {0.7f,0.0f,0.7f,1.0f},
        {0.0f,0.7f,0.7f,1.0f}
    };
    
    root_position = { 2.0f,0.5f,2.0f };
    target = { 3.0f,8.0f,3.0f };
    target_colors = { 0.0f, 1.0f, 0.0f, 1.0f };
    
    bone_mat = {glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f)};
    
    isMoving = false;
    tree_depth = (int)scale_vector.size();
    joint_num = tree_depth - 1;
    
    Rotation();
}

void Bone_Animation::update(float delta_time)
{
    // --> ( solution 1: Rotation Matrix
    pivot.clear();
    axisX.clear();
    axisY.clear();
    axisZ.clear();
    // ---------------
    
    // --> ( solution 2: Quaterion
//    pivot.clear();
//    axisYZX.clear();
    // ---------------
    
    Rotation();

    if (isMoving)
    {
        JacobianTranspose();
    }
}

void Bone_Animation::resetRotation()
{
    isMoving = false;
    
    rotation_degree_vector =
    {
        {0.0f,0.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f},
        {0.0f,30.0f,0.0f}
    };
    
    bone_mat = {glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f)};
}

void Bone_Animation::resetTarget()
{
    isMoving = false;
    target = { 3.0f,8.0f,3.0f };
}

void Bone_Animation::Rotation()
{
    int j;
    
    std::vector<float> angleX = { glm::radians(rotation_degree_vector[0][2]),
                                  glm::radians(rotation_degree_vector[1][2]),
                                  glm::radians(rotation_degree_vector[2][2]),
                                  glm::radians(rotation_degree_vector[3][2]) };
    
    std::vector<float> angleY = { glm::radians(rotation_degree_vector[0][0]),
                                  glm::radians(rotation_degree_vector[1][0]),
                                  glm::radians(rotation_degree_vector[2][0]),
                                  glm::radians(rotation_degree_vector[3][0]) };
    
    std::vector<float> angleZ = { glm::radians(rotation_degree_vector[0][1]),
                                  glm::radians(rotation_degree_vector[1][1]),
                                  glm::radians(rotation_degree_vector[2][1]),
                                  glm::radians(rotation_degree_vector[3][1]) };
    
    rotateX = { glm::eulerAngleX(angleX[0]), glm::eulerAngleX(angleX[1]), glm::eulerAngleX(angleX[2]), glm::eulerAngleX(angleX[3]) };
    rotateY = { glm::eulerAngleX(angleY[0]), glm::eulerAngleY(angleY[1]), glm::eulerAngleY(angleY[2]), glm::eulerAngleY(angleY[3]) };
    rotateZ = { glm::eulerAngleX(angleZ[0]), glm::eulerAngleZ(angleZ[1]), glm::eulerAngleZ(angleZ[2]), glm::eulerAngleZ(angleZ[3]) };
    
    for (int i = 0; i < tree_depth; i++)
    {
        j = i -1;
        rotateMatrix[i] = rotateX[i] * rotateZ[i] * rotateY[i];
        
        if (i == 0)
        {
            translateMatrix_origin[i] = glm::translate(glm::mat4(1.0f) , {0, scale_vector[i][1]/2.0f, 0} );
            translateMatrix_link[i] = glm::translate(glm::mat4(1.0f)  , { root_position[0], ( root_position[1] - scale_vector[i][1]/2.0f ),root_position[2] } );
            translateMatrix_end[i] = glm::translate(glm::mat4(1.0f) , {0, scale_vector[i][1], 0} );
            
            worldMatrix[i] = translateMatrix_link[i] * rotateMatrix[i];
            bone_mat[i] = glm::translate( glm::mat4(1.0f)  , root_position );
        }
        else
        {
            translateMatrix_origin[i] = glm::translate(glm::mat4(1.0f) , {0, scale_vector[i][1]/2.0f, 0} );
            translateMatrix_link[i] = glm::translate(glm::mat4(1.0f)  , {0, scale_vector[i-1][1], 0} );
            translateMatrix_end[i] = glm::translate(glm::mat4(1.0f) , {0, scale_vector[i][1], 0} );
            
            worldMatrix[i] = worldMatrix[i-1] * translateMatrix_link[i] * rotateMatrix[i];
            bone_mat[i] =  worldMatrix[i] * translateMatrix_origin[i];
            
            // Find joint pivot and rotate axis
            
            // --> ( solution 1: Rotation Matrix
            pivot.push_back( glm::vec3( worldMatrix[j] * translateMatrix_end[j] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f) ) );
            axisX.push_back( normalize(glm::vec3( worldMatrix[j] * glm::vec4(1.0f, 0.0f, 0.0f, 0.0f) )) );
            axisY.push_back( normalize(glm::vec3( worldMatrix[j] * rotateX[i] * rotateZ[i] * glm::vec4(0.0f, 1.0f, 0.0f, 0.0f) )) );
            axisZ.push_back( normalize(glm::vec3( worldMatrix[j] * rotateX[i] * glm::vec4(0.0f, 0.0f, 1.0f, 0.0f) )) );
            // ---------------
            
            // --> ( solution 2: Quaterion
            //            pivot.push_back( worldMatrix[j] * translateMatrix_end[j] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f) );
            //            axisYZX.push_back( normalize(cross(endEffector - pivot[j], target - pivot[j])) );
            // ---------------
        }
        
    }
    
    endEffector = worldMatrix[3] * translateMatrix_end[3] * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
    distance = glm::distance( target, endEffector );
}

void Bone_Animation::JacobianTranspose()
{
    // --> ( solution 1: Rotation Matrix
    std::vector<glm::mat3> iJacobian;
    std::vector<glm::mat3> iJacobianTrans;
    std::vector<glm::vec3> deltaTheta;
    std::vector<glm::vec3> umeratorRoot;
    glm::vec3 denominatorRoot;
    
    if (distance >= threshold) {
        
        for (int i = 0; i < joint_num; i++)
        {
            // Calculate Jacobian
            iJacobian.push_back( glm::mat3( cross(axisX[i], (endEffector - pivot[i])), cross(axisY[i], (endEffector - pivot[i])), cross(axisZ[i], (endEffector - pivot[i])) ) );
            
            // For alpha
            iJacobianTrans.push_back( glm::transpose(iJacobian[i]) );
            umeratorRoot.push_back( iJacobianTrans[i] * (target - endEffector) );
        }
        
        denominatorRoot = iJacobian [0] * umeratorRoot[0] + iJacobian [1] * umeratorRoot[1] + iJacobian [2] * umeratorRoot[2];
        
        // Calculate alpha
        alpha = ( dot( umeratorRoot[0], umeratorRoot[0]) + dot( umeratorRoot[1], umeratorRoot[1]) + dot( umeratorRoot[2], umeratorRoot[2]))
        / (dot( denominatorRoot, denominatorRoot ));
        
        for (int i = 0; i < joint_num; i++)
        {
            // Calculate delta theta
            deltaTheta.push_back( alpha * iJacobianTrans[i] * (target - endEffector) );
            
            // Update 9DOF bone values
            rotation_degree_vector[i+1] = rotation_degree_vector[i+1] + glm::vec3( deltaTheta[i][1], deltaTheta[i][2], deltaTheta[i][0] );
        }
    }
    // ---------------
    
    
    // --> ( solution 2: Quaterion
//    std::vector<glm::quat> boneOrientation;
//    std::vector<glm::vec3> deltaAngles;
//    glm::mat3 Jacobian;
//    glm::mat3 JacobianTrans;
//    glm::vec3 deltaTheta;
//    //std::vector<float> weight = {2, 2, 2};
//
//    if (distance >= threshold) {
//
//        // Calculate Jacobian
//        Jacobian = glm::mat3( cross(axisYZX[0], (endEffector - pivot[0]) ), cross(axisYZX[1], (endEffector - pivot[1]) ), cross(axisYZX[2], (endEffector - pivot[2]) ) );
//
//        // Calculate alpha
//        JacobianTrans = glm::transpose(Jacobian);
//        alpha = (dot(JacobianTrans*(target - endEffector), JacobianTrans*(target - endEffector)))
//                / (dot(Jacobian*JacobianTrans*(target - endEffector), Jacobian*JacobianTrans*(target - endEffector)));
//
//        // Calculate delta theta
//        deltaTheta = alpha*JacobianTrans*(target-endEffector);
//
//        // Update 9DOF bone values
//        for (int i = 0; i < joint_num; i++)
//        {
//            boneOrientation.push_back( angleAxis(deltaTheta[i], axisYZX[i]) );
//            deltaAngles.push_back( eulerAngles(boneOrientation[i]) );
//
//            rotation_degree_vector[i+1] = rotation_degree_vector[i+1] + glm::vec3(deltaAngles[i].y, deltaAngles[i].z, deltaAngles[i].x);
//        }
//    }
    // ---------------
    
}











