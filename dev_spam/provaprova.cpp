#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

// Structure to hold joint angles
struct JointAngles {
    double shoulder_1;
    double shoulder_2;
    double knee;
};

// Function to calculate the inverse kinematics for one leg
JointAngles inverseKinematics(const Eigen::Vector3d& shoulder_position, const Eigen::Vector3d& foot_position) {
    JointAngles angles;

    // Calculate the distance between the shoulder and the foot
    Eigen::Vector3d vector_to_foot = foot_position - shoulder_position;
    double distance = vector_to_foot.norm();

    // Calculate the angles
    angles.knee = acos((distance * distance - 2 * 0.5 * 0.5) / (2 * 0.5 * 0.5)); // Assuming leg length is 1 unit
    angles.shoulder_1 = atan2(vector_to_foot.y(), vector_to_foot.x());
    angles.shoulder_2 = atan2(vector_to_foot.z(), sqrt(vector_to_foot.x() * vector_to_foot.x() + vector_to_foot.y() * vector_to_foot.y()));

    return angles;
}

// Function to calculate the foot positions based on COM pose
std::vector<Eigen::Vector3d> calculateFootPositions(const Eigen::Vector3d& com_position, const Eigen::Vector3d& com_orientation) {
    std::vector<Eigen::Vector3d> foot_positions(4);

    // Assuming the feet are symmetrically placed around the COM
    double leg_length = 1.0; // Example leg length
    double shoulder_offset = 0.5; // Example shoulder offset

    foot_positions[0] = com_position + Eigen::Vector3d(shoulder_offset, shoulder_offset, -leg_length);
    foot_positions[1] = com_position + Eigen::Vector3d(shoulder_offset, -shoulder_offset, -leg_length);
    foot_positions[2] = com_position + Eigen::Vector3d(-shoulder_offset, shoulder_offset, -leg_length);
    foot_positions[3] = com_position + Eigen::Vector3d(-shoulder_offset, -shoulder_offset, -leg_length);

    return foot_positions;
}

int main() {
    // Example COM pose
    Eigen::Vector3d com_position(0.0, 0.0, 0.5);
    Eigen::Vector3d com_orientation(0.0, 0.0, 0.0); // Yaw, Pitch, Roll

    // Calculate foot positions
    std::vector<Eigen::Vector3d> foot_positions = calculateFootPositions(com_position, com_orientation);

    // Example shoulder positions (assuming they are fixed relative to the COM)
    std::vector<Eigen::Vector3d> shoulder_positions = {
        Eigen::Vector3d(0.5, 0.5, 0.0),
        Eigen::Vector3d(0.5, -0.5, 0.0),
        Eigen::Vector3d(-0.5, 0.5, 0.0),
        Eigen::Vector3d(-0.5, -0.5, 0.0)
    };

    // Calculate joint angles for each leg
    for (size_t i = 0; i < 4; ++i) {
        JointAngles angles = inverseKinematics(shoulder_positions[i], foot_positions[i]);
        std::cout << "Leg " << i + 1 << " Joint Angles: "
                  << "Shoulder 1: " << angles.shoulder_1 << ", "
                  << "Shoulder 2: " << angles.shoulder_2 << ", "
                  << "Knee: " << angles.knee << std::endl;
    }


    return 0;
}