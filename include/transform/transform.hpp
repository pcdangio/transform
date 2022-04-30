/// \file transform/transform.hpp
/// \brief Defines the transform class.
#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <eigen3/Eigen/Dense>

/// \brief Represents a 3D transformation between coordinate frames.
class transform
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new identity transform instance.
    transform();
    /// \brief Creates a new transform instance from a translation and Euler rotation.
    /// \param translation The translation component of the transform.
    /// \param rotation The Euler angle rotation of the transform.
    transform(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation);
    /// \brief Creates a new transform instance from a translation and quaternion rotation.
    /// \param translation The translation component of the transformation.
    /// \param rotation The rotation component of the transformation.
    transform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& rotation);
    /// \brief Creates a new transform instance from a translation.
    /// \param translation The translation component of the transformation.
    transform(const Eigen::Vector3d& translation);
    /// \brief Creates a new transform instance from a rotation.
    /// \param rotation The rotation component of the transformation.
    transform(const Eigen::Quaterniond& rotation);

    // MODIFIERS
    /// \brief Calculates the inverse of the transform.
    /// \returns The inverted transform.
    transform inverse() const;
    /// \brief Inverts this transform in place.
    void invert();

    // APPLICATIONS
    /// \brief Rotates a 3D point or vector in place.
    /// \param vector The 3D point or vector to rotate.
    void rotate(Eigen::Vector3d& vector) const;
    /// \brief Translates a 3D point or vector in place.
    /// \param vector The 3D point or vector to translate.
    void translate(Eigen::Vector3d& vector) const;
    /// \brief Performs an in-place chain on top of another transform.
    /// \param transform The transform to chain in place.
    void apply(transform& transform) const;
    /// \brief Transforms a 3D point or vector in place.
    /// \param vector The 3D point or vector to transform.
    void apply(Eigen::Vector3d& vector) const;
    /// \brief Transforms a 3D pose in place.
    /// \param position The 3D position of the pose.
    /// \param orientation The 3D euler angle representation of the pose orientation.
    void apply(Eigen::Vector3d& position, Eigen::Vector3d& orientation) const;
    /// \brief Transforms a 3D pose in place.
    /// \param position The 3D position of the pose.
    /// \param orientation The 3D quaternion representation of the pose orientation.
    void apply(Eigen::Vector3d& position, Eigen::Quaterniond& orientation) const;

    // ACCESS
    /// \brief Gets the translation component of this transform.
    /// \returns A const reference to the translation component.
    const Eigen::Vector3d& translation() const;
    /// \brief Gets the rotation component of this transform.
    /// \returns A const reference to the rotation component.
    const Eigen::Quaterniond& rotation() const;
    
private:
    // VARIABLES
    /// \brief The transform's translation component.
    Eigen::Vector3d m_translation;
    /// \brief The transform's rotation component.
    Eigen::Quaterniond m_rotation;
};

#endif