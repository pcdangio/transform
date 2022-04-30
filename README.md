# transform

A C++ library for applying 3D geometric transforms to points, vectors, and poses.

## Overview:
This library provides a ```transform``` class that allows the user to create and apply 3D geometric transforms to points, vectors, and poses. A ```transform``` is first constructed with standard translation / rotation values, and can then be applied to a series of geometries. The majority of operations are performed in-place for higher performance.

## Dependencies:
- [Eigen3 (libeigen3-dev)](https://eigen.tuxfamily.org/index.php?title=Main_Page)

## Usage:
```cpp
#include <transform/transform.hpp>
#include <iostream>

int32_t main(int32_t argc, char** argv)
{
    // Create a transform instance.
    Eigen::Vector3d translation = {1.0, 2.0, 3.0};
    Eigen::Vector3d rotation = {0.02, -0.1, 0.0};
    transform tf(translation, rotation);

    // Create a point.
    Eigen::Vector3d point = {4.0, 5.0, 6.0};

    // Apply the transform to the point.
    // NOTE: This changes the point in place.
    tf.apply(point);

    // Print the newly transformed point.
    std::cout << point << std::endl;
}
```