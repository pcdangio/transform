#include <transform/transform.hpp>

// CONSTRUCTORS
transform::transform()
{
    transform::m_translation.setZero();
    transform::m_rotation.setIdentity();
}
transform::transform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation)
    : m_translation(translation),
      m_rotation(rotation)
{}
transform::transform(const Eigen::Vector3d& translation)
    : m_translation(translation)
{
    transform::m_rotation.setIdentity();
}
transform::transform(const Eigen::Quaterniond& rotation)
    : m_rotation(rotation)
{
    transform::m_translation.setZero();
}

// MODIFIERS
transform transform::inverse() const
{
    // Create output transform.
    // NOTE: This method allows Eigen to do operations in place.
    transform inverted;

    // Invert the translation.
    inverted.m_translation = transform::m_translation * -1.0;

    // Invert the rotation.
    inverted.m_rotation = transform::m_rotation.inverse();

    return inverted;
}
void transform::invert()
{
    // Invert the translation.
    transform::m_translation *= 1.0;

    // Invert the rotation.
    transform::m_rotation = transform::m_rotation.inverse();
}

// APPLICATIONS
void transform::rotate(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform::m_rotation * vector;
}
void transform::translate(Eigen::Vector3d& vector) const
{
    // Add the transform's translation to the rotated vector.
    vector += transform::m_translation;
}
void transform::apply(transform& transform) const
{
    // Apply this transform's rotation to the original transform.
    transform.m_rotation = transform::m_rotation * transform.m_rotation;
    transform.m_translation = transform::m_rotation * transform.m_translation;
    // Normalize the transformed rotation for numerical stability.
    transform.m_rotation.normalize();

    // Add this transform's translation to the now rotated original translation.
    transform.m_translation += transform::m_translation;
}
void transform::apply(Eigen::Vector3d& vector) const
{
    // Apply this transform's rotation to the original vector.
    vector = transform::m_rotation * vector;

    // Add the transform's translation to the rotated vector.
    vector += transform::m_translation;
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
    orientation = transform::m_rotation * orientation;
    // Normalize the transformed orientation for numerical stability.
    orientation.normalize();

    // Apply this transform's rotation to the position.
    position = transform::m_rotation * position;

    // Add this transform's translation to the position.
    position += transform::m_translation;
}

// ACCESS
const Eigen::Vector3d& transform::translation() const
{
    return transform::m_translation;
}
const Eigen::Quaterniond& transform::rotation() const
{
    return transform::m_rotation;
}