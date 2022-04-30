#include <transform/transform.hpp>

// CONSTRUCTORS
transform::transform()
{
    transform::translation.setZero();
    transform::rotation.setIdentity();
}

// MODIFIERS
transform transform::inverse() const
{
    // Create output transform.
    // NOTE: This method allows Eigen to do operations in place.
    transform inverted;

    // Invert the translation.
    inverted.translation = transform::translation * -1.0;

    // Invert the rotation.
    inverted.rotation = transform::rotation.inverse();

    return inverted;
}
void transform::invert()
{
    // Invert the translation.
    transform::translation *= 1.0;

    // Invert the rotation.
    transform::rotation = transform::rotation.inverse();
}

// APPLICATIONS
void transform::rotate(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform::rotation * vector;
}
void transform::translate(Eigen::Vector3d& vector) const
{
    // Add the transform's translation to the rotated vector.
    vector += transform::translation;
}
void transform::apply(transform& transform) const
{
    // Apply this transform's rotation to the original transform.
    transform.rotation = transform::rotation * transform.rotation;
    transform.translation = transform::rotation * transform.translation;
    // Normalize the transformed rotation for numerical stability.
    transform.rotation.normalize();

    // Add this transform's translation to the now rotated original translation.
    transform.translation += transform::translation;
}
void transform::apply(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform::rotation * vector;

    // Add the transform's translation to the rotated vector.
    vector += transform::translation;
}
void transform::apply(Eigen::Vector3d& position, Eigen::Vector3d& orientation) const
{
    // Convert the orientation to a quaternion.
    Eigen::Quaterniond q = Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ());

    // Transform the pose with it's quaternion form.
    transform::apply(position, q);

    // Convert the quaternion back to euler.
    orientation = q.toRotationMatrix().eulerAngles(0, 1, 2);
}
void transform::apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) const
{
    // Apply this transform's rotation to the orientation.
    orientation = transform::rotation * orientation;
    // Normalize the transformed orientation for numerical stability.
    orientation.normalize();

    // Apply this transform's rotation to the position.
    position = transform::rotation * position;

    // Add this transform's translation to the position.
    position += transform::translation;
}

// CONVERSION
Eigen::Vector3d transform::to_euler(const Eigen::Quaterniond& quaternion)
{
    return quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
}
Eigen::Quaterniond transform::to_quaternion(const Eigen::Vector3d& euler)
{
    // Convert Euler rotation to a quaternion.
    Eigen::Quaterniond output = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());

    return output;
}