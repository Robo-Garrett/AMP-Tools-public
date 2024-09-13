#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

// Declare your Bug Algorithm class, inheriting from BugAlgorithm
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the plan method
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;

        // Helper methods for the Bug1 algorithm
        bool isInCollision(const Eigen::Vector2d& position, const amp::Problem2D& problem) const;
        bool isPointInsideObstacle(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) const;

        // Methods to handle boundary following and goal searching
        Eigen::Vector2d followBoundary(Eigen::Vector2d& current_position, const amp::Problem2D& problem, amp::Path2D& path, double& closest_distance_to_goal);
        Eigen::Vector2d findNextBoundaryPoint(const Eigen::Vector2d& current_position, const amp::Problem2D& problem) const;
        Eigen::Vector2d moveAlongEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) const;

        // Helper methods for navigation
        bool reachedStartBoundary(const Eigen::Vector2d& current_position) const;
        bool isNearEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) const;

    private:
        // Store the initial contact point with the obstacle for boundary checking
        Eigen::Vector2d start_boundary_position;
};
