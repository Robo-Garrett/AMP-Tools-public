#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Add any other methods here...
        bool isInCollision(const Eigen::Vector2d& position, const amp::Problem2D& problem);

        bool isPointInsideObstacle(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle);

        void followBoundary(Eigen::Vector2d& current_position, const amp::Problem2D& problem, amp::Path2D& path, double& closest_distance_to_goal);

        Eigen::Vector2d findNextBoundaryPoint(const Eigen::Vector2d& current_position, const amp::Problem2D& problem);

        Eigen::Vector2d moveAlongEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);

        bool reachedStartBoundary(const Eigen::Vector2d& current_position);

        bool isNearEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);

        Eigen::Vector2d moveAlongEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2);
    
    private:
        // Add any member variables here...
};