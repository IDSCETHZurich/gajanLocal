#ifndef QUATERNIONS_HPP_
#define QUATERNIONS_HPP_


#include <Eigen/Eigen>

// Make a skew symmetric matrix out of a vector
Eigen::Matrix3d skew (Eigen::Vector3d vector);

// Quaternion multiplication
Eigen::Vector4d quaternionMultiplication (Eigen::Vector4d p, Eigen::Vector4d q);

// Make an imaginary quaternion out of a vector
Eigen::Vector4d vectorToImaginaryQuaternion (Eigen::Vector3d vector);

// Make a vector out of an imaginary quaternion
Eigen::Vector3d imaginaryQuaternionToVector (Eigen::Vector4d imaginaryQuaternion);

// Calculate the inverse quaternion
Eigen::Vector4d inverseQuaternion (Eigen::Vector4d quaternion);

// Rotate a vector
Eigen::Vector3d rotateVector (Eigen::Vector3d vector, Eigen::Vector4d quaternion);

// Get the vector in different coordinates
Eigen::Vector3d transformVector (Eigen::Vector3d vector, Eigen::Vector4d quaternion);

#endif /* QUATERNIONS_HPP_ */
