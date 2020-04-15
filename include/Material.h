#ifndef MATERIAL_H
#define MATERIAL_H
#include <Eigen/Core>

// Blinn-Phong Approximate Shading Material Parameters
struct Material
{
  // Ambient, Diffuse, Specular, Mirror Color
  Eigen::Vector3d ka,kd,ks,km;
  // Phong exponent
  double phong_exponent;
  /* 
  One component for red, green, and blue light. In a given component, 1.0
  means that the material is completely opaque to the corresponding colour
  0.0 in a component means that colour passes through the object unabsorbed.
  */
  Eigen::Vector3d opacity;
  // -1 if material is opaque (opacity is 1-vector). Otherwise, at least 1.
  double refractive_index;
};
#endif
