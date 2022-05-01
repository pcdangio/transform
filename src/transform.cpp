#include <transform/transform.hpp>

using namespace transform;

// CONSTRUCTORS
transform_t::transform_t()
{
    transform_t::translation.setZero();
    transform_t::rotation.setIdentity();
}

// MODIFIERS
transform_t transform_t::inverse() const
{
    // Create output transform.
    // NOTE: This method allows Eigen to do operations in place.
    transform_t inverted;

    // Invert the translation.
    inverted.translation = transform_t::translation * -1.0;

    // Invert the rotation.
    inverted.rotation = transform_t::rotation.inverse();

    return inverted;
}
void transform_t::invert()
{
    // Invert the translation.
    transform_t::translation *= 1.0;

    // Invert the rotation.
    transform_t::rotation = transform_t::rotation.inverse();
}

// APPLICATIONS
void transform_t::rotate(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform_t::rotation * vector;
}
void transform_t::translate(Eigen::Vector3d& vector) const
{
    // Add the transform's translation to the rotated vector.
    vector += transform_t::translation;
}
void transform_t::apply(transform_t& transform) const
{
    // Apply this transform's rotation to the original transform.
    transform.rotation = transform_t::rotation * transform.rotation;
    transform.translation = transform_t::rotation * transform.translation;
    // Normalize the transformed rotation for numerical stability.
    transform.rotation.normalize();

    // Add this transform's translation to the now rotated original translation.
    transform.translation += transform_t::translation;
}
void transform_t::apply(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform_t::rotation * vector;

    // Add the transform's translation to the rotated vector.
    vector += transform_t::translation;
}
void transform_t::apply(Eigen::Vector3d& position, Eigen::Vector3d& orientation) const
{
    // Convert the orientation to a quaternion.
    Eigen::Quaterniond q = Eigen::AngleAxisd(orientation.x(), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(orientation.y(), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(orientation.z(), Eigen::Vector3d::UnitZ());

    // Transform the pose with it's quaternion form.
    transform_t::apply(position, q);

    // Convert the quaternion back to euler.
    orientation = q.toRotationMatrix().eulerAngles(0, 1, 2);
}
void transform_t::apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) const
{
    // Apply this transform's rotation to the orientation.
    orientation = transform_t::rotation * orientation;
    // Normalize the transformed orientation for numerical stability.
    orientation.normalize();

    // Apply this transform's rotation to the position.
    position = transform_t::rotation * position;

    // Add this transform's translation to the position.
    position += transform_t::translation;
}

// CONVERSION
Eigen::Vector3d transform_t::to_euler(const Eigen::Quaterniond& quaternion)
{
    return quaternion.toRotationMatrix().eulerAngles(0, 1, 2);
}
Eigen::Quaterniond transform_t::to_quaternion(const Eigen::Vector3d& euler)
{
    // Convert Euler rotation to a quaternion.
    Eigen::Quaterniond output = Eigen::AngleAxisd(euler.x(), Eigen::Vector3d::UnitX()) *
                                Eigen::AngleAxisd(euler.y(), Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(euler.z(), Eigen::Vector3d::UnitZ());

    return output;
}