import numpy as np
import matplotlib.pyplot as plt

# Define the leg's geometry and desired end-effector position
# Lengths of the upper and lower leg segments
L1 = 0.2  # Hip to knee length
L2 = 0.2  # Knee to foot length

# Desired end-effector position (x, y, z) relative to the hip joint
end_effector_pos = np.array([0.2, 0.2, -0.3])

# Maximum iterations and tolerance for convergence
max_iterations = 100
tolerance = 0.001

def forward_kinematics(theta1, theta2):
    # Compute the end-effector position using forward kinematics
    x = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
    y = L1 * np.sin(theta1) + L2 * np.sin(theta1 + theta2)
    z = 0  # Assume the leg moves only in the x-y plane (z = 0)
    return np.array([x, y, z])

def jacobian_transpose_inverse_kinematics_with_path(current_joint_angles, end_effector_pos):
    # Create lists to store the end-effector positions for plotting
    path_x, path_y, path_z = [], [], []

    theta1, theta2 = current_joint_angles

    for iteration in range(max_iterations):
        # Compute the current end-effector position using forward kinematics
        current_end_effector_pos = forward_kinematics(theta1, theta2)

        # Append the current end-effector position to the path lists
        path_x.append(current_end_effector_pos[0])
        path_y.append(current_end_effector_pos[1])
        path_z.append(current_end_effector_pos[2])

        # Calculate the error between the current and desired end-effector position
        error = end_effector_pos - current_end_effector_pos

        # Check if the error is within the tolerance
        if np.linalg.norm(error) < tolerance:
            print("Inverse kinematics converged in {} iterations".format(iteration + 1))
            return theta1, theta2, path_x, path_y, path_z

        # Compute the Jacobian matrix (partial derivatives of the end-effector position w.r.t joint angles)
        J11 = -L1 * np.sin(theta1) - L2 * np.sin(theta1 + theta2)
        J12 = -L2 * np.sin(theta1 + theta2)
        J21 = L1 * np.cos(theta1) + L2 * np.cos(theta1 + theta2)
        J22 = L2 * np.cos(theta1 + theta2)

        # Use Jacobian Transpose method to update joint angles
        J = np.array([[J11, J12], [J21, J22]])
        J_transpose = np.transpose(J)
        delta_theta = np.dot(J_transpose, error)
        theta1 += delta_theta[0]
        theta2 += delta_theta[1]

    print("Inverse kinematics did not converge!")
    return None

# Initial guess for joint angles (radians)
initial_joint_angles = np.array([np.pi / 4, np.pi / 4])

# Calculate inverse kinematics with the path
result, path_x, path_y, path_z = jacobian_transpose_inverse_kinematics_with_path(initial_joint_angles, end_effector_pos)
if result:
    theta1, theta2 = result
    print("Joint angles: Theta1 = {:.2f} rad, Theta2 = {:.2f} rad".format(theta1, theta2))

    # Plot the path traversed by the end-effector
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(path_x, path_y, path_z, label='End-effector Path')
    ax.scatter(path_x[-1], path_y[-1], path_z[-1], color='red', label='End Position')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Path Traversed by End-effector')
    ax.legend()
    plt.show()
