#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class Bone_Animation
{
public:
    Bone_Animation();
    ~Bone_Animation();
    
    void init();
    void update(float delta_time);
    
private:
    
    // for Forward Kinematics
    std::vector<glm::mat4> rotateX;
    std::vector<glm::mat4> rotateY;
    std::vector<glm::mat4> rotateZ;
    
    std::vector<glm::mat4> rotateMatrix = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f) };
    std::vector<glm::mat4> translateMatrix_origin = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f)  };
    std::vector<glm::mat4> translateMatrix_link = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f)  };
    std::vector<glm::mat4> translateMatrix_end = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f)  };
    
    std::vector<glm::mat4> worldMatrix = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f) };
    
    // for Inverse Kinematics
    
    // --> ( solution 1: Rotation Matrix
    std::vector<glm::vec3> axisX;
    std::vector<glm::vec3> axisY;
    std::vector<glm::vec3> axisZ;
    std::vector<glm::vec3> pivot;
    
    // --> ( solution 2: Quaterion
//    std::vector<glm::vec3> axisYZX;
//    std::vector<glm::vec3> pivot;
    
    int joint_num;
    float threshold = 1e-6;
    float distance;
    float alpha;
    
    // Forward Kinematics
    void Rotation();
    
    // Inverse Kinematics
    void JacobianTranspose();
    
public:
    
    // Here the head of each vector is the root bone
    std::vector<glm::vec3> scale_vector;
    std::vector<glm::vec3> rotation_degree_vector;
    std::vector<glm::vec4> colors;
    std::vector<glm::mat4> bone_mat = { glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f), glm::mat4(1.0f) };
    
    glm::vec3 root_position;
    glm::vec3 target;
    glm::vec3 endEffector;
    glm::vec4 target_colors;
    
    bool isMoving;
    int tree_depth;
    
    void resetRotation();
    void resetTarget();
    
};


