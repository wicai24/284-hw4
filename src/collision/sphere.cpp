#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  Vector3D diff = pm.position - origin;
  double dist = diff.norm();

  if (dist <= radius) {
    // Tangent point on sphere surface
    Vector3D tangent = origin + diff.unit() * radius;

    // Correction vector from last_position to tangent point
    Vector3D correction = tangent - pm.last_position;

    // Apply with friction
    pm.position = pm.last_position + correction * (1.0 - friction);
  }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
