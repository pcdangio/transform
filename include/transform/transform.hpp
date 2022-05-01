/// \file transform/transform.hpp
/// \brief Defines the transform::transform_t class.
#ifndef TRANSFORM___TRANSFORM_H
#define TRANSFORM___TRANSFORM_H

#include <eigen3/Eigen/Dense>

/// \brief Represents a 3D transformation between coordinate frames.
struct transform_t
{
public:
    // CONSTRUCTORS
    /// \brief Creates a new identity transform instance.
    transform_t();

    // MODIFIERS
    /// \brief Calculates the inverse of the transform.
    /// \returns The inverted transform.
    transform_t inverse() const;
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
    void apply(transform_t& transform) const;
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
    
    // VARIABLES
    /// \brief The transform's translation component.
    Eigen::Vector3d translation;
    /// \brief The transform's rotation component.
    Eigen::Quaterniond rotation;

    // CONVERSIONS
    /// \brief Converts a quaternion rotation to a Euler rotation.
    /// \param quaternion The quaternion to convert.
    /// \returns The converted Euler rotation.
    static Eigen::Vector3d to_euler(const Eigen::Quaterniond& quaternion);
    /// \brief Converts a Euler rotation to a quaternion rotation.
    /// \param euler The Euler rotation to convert.
    /// \returns The converted quaternion.
    static Eigen::Quaterniond to_quaternion(const Eigen::Vector3d& euler);
};

#endif