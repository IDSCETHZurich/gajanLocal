#include "quaternions.hpp"

// Make a skew symmetric matrix out of a vector
Eigen::Matrix3d skew (Eigen::Vector3d vector)
{
	Eigen::Matrix3d tempMatrix;
	tempMatrix << 0.0, -vector(2), vector(1), vector(2), 0.0, -vector(0), -vector(1), vector(0), 0.0;

	return tempMatrix;
}

// Quaternion multiplication
Eigen::Vector4d quaternionMultiplication (Eigen::Vector4d p, Eigen::Vector4d q)
{
	Eigen::Matrix4d tempMatrix;
	tempMatrix << p(1), -p.block<3, 1>(1, 0).transpose (), p.block<3, 1>(1, 0), p(1) * Eigen::Matrix3d::Identity () + skew (p.block<3, 1>(1, 0));

	return tempMatrix * q;
}

// Make an imaginary quaternion out of a vector
Eigen::Vector4d vectorToImaginaryQuaternion (Eigen::Vector3d vector)
{
	Eigen::Vector4d tempVector;
	tempVector << 0, vector;

	return tempVector;
}

// Make a vector out of an imaginary quaternion
Eigen::Vector3d imaginaryQuaternionToVector (Eigen::Vector4d imaginaryQuaternion)
{
	return imaginaryQuaternion.bottomRows (3);
}

// Calculate the inverse quaternion
Eigen::Vector4d inverseQuaternion (Eigen::Vector4d quaternion)
{
	Eigen::Vector4d tempQuaternion;
	tempQuaternion << quaternion(1), -quaternion.block<3, 1>(1, 0);

	return tempQuaternion;
}

// Rotate a vector
Eigen::Vector3d rotateVector (Eigen::Vector3d vector, Eigen::Vector4d quaternion)
{
	return imaginaryQuaternionToVector (quaternionMultiplication (quaternion, quaternionMultiplication (vectorToImaginaryQuaternion (vector), inverseQuaternion (quaternion))));
}

// Get the vector in different coordinates
Eigen::Vector3d transformVector (Eigen::Vector3d vector, Eigen::Vector4d quaternion)
{
	return imaginaryQuaternionToVector (quaternionMultiplication (inverseQuaternion (quaternion), quaternionMultiplication (vectorToImaginaryQuaternion (vector), quaternion)));
}
