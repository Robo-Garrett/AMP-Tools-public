#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    Eigen::Vector2d q_init = problem.q_init;
    Eigen::Vector2d q_goal = problem.q_goal;
    Eigen::Vector2d current_position = q_init;

    double distance_threshold = 0.001;
    Eigen::Vector2d closest_point;              // To track the closest point to the goal
    double closest_distance_to_goal = std::numeric_limits<double>::max(); // Set to a large value initially

    // Start moving towards the goal
    while ((q_goal - current_position).norm() > distance_threshold) {
        if (isInCollision(current_position, problem)) {
            // If a collision is detected, follow the obstacle boundary
            closest_point = followBoundary(current_position, problem, path, closest_distance_to_goal);
            // After boundary following, return to the closest point to resume moving toward the goal
            current_position = closest_point;
            path.waypoints.push_back(current_position);
        } else {
            // Move directly toward the goal
            current_position += (q_goal - current_position).normalized() * distance_threshold;
            path.waypoints.push_back(current_position);
        }
    }

    // Add the final goal point
    path.waypoints.push_back(q_goal);


    return path;
}

MyBugAlgorithm::isInCollision(const Eigen::Vector2d& position, const amp::Problem2D& problem){
    for (const auto& obstacle : problem.obstacles) {
        if (isPointInsideObstacle(position, obstacle)) {
            return true;
        }
    }
    return false;
}

MyBugAlgorithm::isPointInsideObstacle(const Eigen::Vector2d& point, const amp::Obstacle2D& obstacle) {
    Eigen::Vector2d min_bound = obstacle.vertices[0];
    Eigen::Vector2d max_bound = obstacle.vertices[0];

    // Find the bounding box for the obstacle
    for (const auto& vertex : obstacle.vertices) {
        min_bound = min_bound.cwiseMin(vertex);
        max_bound = max_bound.cwiseMax(vertex);
    }

    // Check if the point is within the bounding box
    return (point.x() >= min_bound.x() && point.x() <= max_bound.x() &&
            point.y() >= min_bound.y() && point.y() <= max_bound.y());
}

Eigen::Vector2d MyBugAlgorithm::followBoundary(Eigen::Vector2d& current_position, const amp::Problem2D& problem, amp::Path2D& path, double& closest_distance_to_goal) {
    Eigen::Vector2d closest_point = current_position;  // Track the closest point to the goal
    Eigen::Vector2d q_goal = problem.q_goal;           // Goal position
    double initial_distance_to_goal = (q_goal - current_position).norm(); // Distance when first encountering the obstacle

    bool circumnavigated = false;                      // Flag to track if we've circled the obstacle

    // Start following the boundary
    do {
        // Move along the boundary in a left-turn manner
        Eigen::Vector2d next_boundary_point = findNextBoundaryPoint(current_position, problem);
        current_position = next_boundary_point;
        path.waypoints.push_back(current_position);

        // Update the closest point to the goal during boundary following
        double current_distance_to_goal = (q_goal - current_position).norm();
        if (current_distance_to_goal < closest_distance_to_goal) {
            closest_point = current_position;
            closest_distance_to_goal = current_distance_to_goal;
        }

        // Check if we've returned to the starting boundary point (circumnavigated the obstacle)
        circumnavigated = reachedStartBoundary(current_position);

    } while (!circumnavigated);

    // Return the closest point encountered during boundary following
    return closest_point;
}

Eigen::Vector2d MyBugAlgorithm::findNextBoundaryPoint(const Eigen::Vector2d& current_position, const amp::Problem2D& problem) {
    // Loop through each obstacle
    for (const auto& obstacle : problem.obstacles) {
        // Get the number of vertices in the obstacle
        int num_vertices = obstacle.vertices.size();

        // Loop through each edge of the obstacle
        for (int i = 0; i < num_vertices; ++i) {
            // Get the current edge vertices
            Eigen::Vector2d v1 = obstacle.verticesCCW[i];
            Eigen::Vector2d v2 = obstacle.verticesCCW[(i + 1) % num_vertices];  // Wrap around to the first vertex

            // Check if the robot is close to this edge (within some small threshold)
            if (isNearEdge(current_position, v1, v2)) {
                // Move along this edge using the left-hand rule
                return moveAlongEdge(current_position, v1, v2);
            }
        }
    }

    // If no movement is possible, return the current position
    return current_position;
}

Eigen::Vector2d MyBugAlgorithm::moveAlongEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    // Move along the line segment between v1 and v2 in small steps
    Eigen::Vector2d direction = (v2 - v1).normalized();
    double step_size = 0.001;  // Step size for moving along the edge

    // Move the robot along the edge in the direction of v2
    return position + direction * step_size;
}

MyBugAlgorithm::reachedStartBoundary(const Eigen::Vector2d& current_position) {
    static Eigen::Vector2d start_boundary_position = current_position;  // Store the first contact point with the obstacle
    double threshold = 0.001;  // Small threshold for determining when we're "close enough" to the start point

    // Check if the current position is close enough to the start boundary position
    double distance_to_start = (current_position - start_boundary_position).norm();
    return (distance_to_start < threshold);
}
bool MyBugAlgorithm::isNearEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    // Calculate the vector along the edge
    Eigen::Vector2d edge_direction = v2 - v1;
    
    // Calculate the vector from the position to one of the vertices
    Eigen::Vector2d to_position = position - v1;
    
    // Project the position onto the edge to find the closest point on the edge
    double t = edge_direction.dot(to_position) / edge_direction.squaredNorm();
    t = std::clamp(t, 0.0, 1.0); // Clamp to the range [0, 1] to ensure the closest point is on the edge segment

    // Closest point on the edge
    Eigen::Vector2d closest_point_on_edge = v1 + t * edge_direction;

    // Check if the distance between the position and the closest point is within a small threshold
    double distance_to_edge = (position - closest_point_on_edge).norm();
    double edge_threshold = 0.1; // Define how close we need to be to consider it "near" the edge

    return distance_to_edge < edge_threshold;
}

Eigen::Vector2d MyBugAlgorithm::moveAlongEdge(const Eigen::Vector2d& position, const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
    // Calculate the direction along the edge
    Eigen::Vector2d edge_direction = (v2 - v1).normalized();
    
    // Define a step size for how far we move along the edge at each iteration
    double step_size = 0.05; // You can adjust this based on how fast you want to move along the edge

    // Move the position along the edge by the step size
    Eigen::Vector2d new_position = position + edge_direction * step_size;

    // Ensure the new position does not go past the edge (i.e., stays within the segment defined by v1 and v2)
    double t = (new_position - v1).dot(v2 - v1) / (v2 - v1).squaredNorm();
    if (t > 1.0) {
        new_position = v2;  // If we've reached or passed v2, clamp the position to v2
    } else if (t < 0.0) {
        new_position = v1;  // If we've reached or passed v1, clamp the position to v1
    }

    return new_position;
}
