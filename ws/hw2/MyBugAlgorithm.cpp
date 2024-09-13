#include "MyBugAlgorithm.h"

// Plan method implementation (Bug1 algorithm)
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {
    amp::Path2D path;
    Eigen::Vector2d q_init = problem.q_init;
    Eigen::Vector2d q_goal = problem.q_goal;
    Eigen::Vector2d current_position = q_init;

    double distance_threshold = 0.001;
    double closest_distance_to_goal = std::numeric_limits<double>::max();
    Eigen::Vector2d closest_point;

    // Start moving towards the goal
    while ((q_goal - current_position).norm() > distance_threshold) {
        if (isInCollision(current_position, problem)) {
            // Collision detected, start boundary following
            closest_point = followBoundary(current_position, problem, path, closest_distance_to_goal);
            current_position = closest_point;
        } else {
            // Move towards the goal
            current_position += (q_goal - current_position).normalized() * distance_threshold;
        }
        path.waypoints.push_back(current_position);
    }
    
    path.waypoints.push_back(q_goal); // Ensure the goal is added
    return path;
}

// Check if the current position is in collision
bool MyBugAlgorithm::isInCollision(const Eigen::Vector2d& position, const amp::Problem2D& problem) const {
    for (const auto& obstacle : problem.obstacles) {
        if (isPointInsideObstacle(position, obstacle)) {
            return true;
        }
    }
    return false;
}

// Check if a point is inside a given obstacle (bounding box check)
bool MyBugAlgorithm::isPointInsideObstacle(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) const {
    Eigen::Vector2d min_bound = obstacle.verticesCW()[0];
    Eigen::Vector2d max_bound = obstacle.verticesCW()[0];

    for (const auto& vertex : obstacle.verticesCW()) {
        min_bound = min_bound.cwiseMin(vertex);
        max_bound = max_bound.cwiseMax(vertex);
    }

    return (point.x() >= min_bound.x() && point.x() <= max_bound.x() &&
            point.y() >= min_bound.y() && point.y() <= max_bound.y());
}

// Follow the obstacle boundary until a better position is found
Eigen::Vector2d MyBugAlgorithm::followBoundary(Eigen::Vector2d& current_position, const amp::Problem2D& problem, amp::Path2D& path, double& closest_distance_to_goal) {
    Eigen::Vector2d closest_point = current_position;
    Eigen::Vector2d q_goal = problem.q_goal;
    bool circumnavigated = false;

    // Begin boundary following
    do {
        Eigen::Vector2d next_boundary_point = findNextBoundaryPoint(current_position, problem);
        current_position = next_boundary_point;
        path.waypoints.push_back(current_position);

        // Update the closest point to the goal
        double current_distance_to_goal = (q_goal - current_position).norm();
        if (current_distance_to_goal < closest_distance_to_goal) {
            closest_point = current_position;
            closest_distance_to_goal = current_distance_to_goal;
        }

        circumnavigated = reachedStartBoundary(current_position);

    } while (!circumnavigated);

    return closest_point;
}

// Find the next point on the boundary
Eigen::Vector2d MyBugAlgorithm::findNextBoundaryPoint(const Eigen::Vector2d& current_position, const amp::Problem2D& problem) const {
    for (const auto& obstacle : problem.obstacles) {
        int num_vertices = obstacle.verticesCCW().size();
        for (int i = 0; i < num_vertices; ++i) {
            Eigen::Vector2d v1 = obstacle.verticesCCW()[i];
            Eigen::Vector2d v2 = obstacle.verticesCCW()[(i + 1) % num_vertices];
            if (isNearEdge(current_position, v1, v2)) {
                return moveAlongEdge(current_position, v1, v2);
            }
        }
    }
    return current_position;
}

// Check if the boundary has been circumnavigated
bool MyBugAlgorithm::reachedStartBoundary(const Eigen::Vector2d& current_position) const {
    double threshold = 0.001;
    double distance_to_start = (current_position - start_boundary_position).norm();
    return (distance_to_start < threshold);
}

// Check if a point is near an obstacle edge
bool MyBugAlgorithm::isNearEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) const {
    Eigen::Vector2d edge_direction = v2 - v1;
    Eigen::Vector2d to_position = position - v1;
    double t = edge_direction.dot(to_position) / edge_direction.squaredNorm();
    t = std::clamp(t, 0.0, 1.0);
    Eigen::Vector2d closest_point_on_edge = v1 + t * edge_direction;
    double distance_to_edge = (position - closest_point_on_edge).norm();
    return distance_to_edge < 0.1; // Threshold for proximity to edge
}

// Move along an edge between two vertices
Eigen::Vector2d MyBugAlgorithm::moveAlongEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) const {
    Eigen::Vector2d edge_direction = (v2 - v1).normalized();
    double step_size = 0.05;
    Eigen::Vector2d new_position = position + edge_direction * step_size;
    double t = (new_position - v1).dot(v2 - v1) / (v2 - v1).squaredNorm();
    if (t > 1.0) new_position = v2;
    else if (t < 0.0) new_position = v1;
    return new_position;
}
