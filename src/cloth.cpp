#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // Create point masses in row-major order
  for (int row = 0; row < num_height_points; row++) {
    for (int col = 0; col < num_width_points; col++) {
      double x = (double)col * width / (num_width_points - 1);
      Vector3D pos;

      if (orientation == HORIZONTAL) {
        double z = (double)row * height / (num_height_points - 1);
        pos = Vector3D(x, 1.0, z);
      } else {
        double y = (double)row * height / (num_height_points - 1);
        double z = (rand() / (double)RAND_MAX) * 2.0 / 1000.0 - 1.0 / 1000.0;
        pos = Vector3D(x, y, z);
      }

      // Check if this point is pinned
      bool is_pinned = false;
      for (auto &p : pinned) {
        if (p[0] == col && p[1] == row) {
          is_pinned = true;
          break;
        }
      }

      point_masses.emplace_back(pos, is_pinned);
    }
  }

  // Create springs
  for (int row = 0; row < num_height_points; row++) {
    for (int col = 0; col < num_width_points; col++) {
      int idx = row * num_width_points + col;

      // Structural: left neighbor and upper neighbor
      if (col > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - 1], STRUCTURAL);
      }
      if (row > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - num_width_points], STRUCTURAL);
      }

      // Shearing: upper-left and upper-right diagonal
      if (row > 0 && col > 0) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - num_width_points - 1], SHEARING);
      }
      if (row > 0 && col < num_width_points - 1) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - num_width_points + 1], SHEARING);
      }

      // Bending: two left and two above
      if (col > 1) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - 2], BENDING);
      }
      if (row > 1) {
        springs.emplace_back(&point_masses[idx], &point_masses[idx - 2 * num_width_points], BENDING);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // Part 2: Compute total force acting on each point mass.

  // Reset forces
  for (PointMass &pm : point_masses) {
    pm.forces = Vector3D(0, 0, 0);
  }

  // Apply external forces (F = ma)
  for (PointMass &pm : point_masses) {
    for (Vector3D &a : external_accelerations) {
      pm.forces += mass * a;
    }
  }

  // Apply spring correction forces (Hooke's law)
  for (Spring &s : springs) {
    if ((s.spring_type == STRUCTURAL && !cp->enable_structural_constraints) ||
        (s.spring_type == SHEARING && !cp->enable_shearing_constraints) ||
        (s.spring_type == BENDING && !cp->enable_bending_constraints)) {
      continue;
    }

    Vector3D diff = s.pm_b->position - s.pm_a->position;
    double dist = diff.norm();
    Vector3D force_dir = diff / dist;

    double ks = cp->ks;
    if (s.spring_type == BENDING) {
      ks *= 0.2;
    }

    double force_mag = ks * (dist - s.rest_length);
    Vector3D force = force_mag * force_dir;

    s.pm_a->forces += force;
    s.pm_b->forces -= force;
  }

  // Part 2: Use Verlet integration to compute new point mass positions
  double d = cp->damping / 100.0;
  for (PointMass &pm : point_masses) {
    if (pm.pinned) continue;

    Vector3D accel = pm.forces / mass;
    Vector3D new_pos = pm.position + (1.0 - d) * (pm.position - pm.last_position)
                       + accel * delta_t * delta_t;
    pm.last_position = pm.position;
    pm.position = new_pos;
  }

  // Part 4: Handle self-collisions.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
    self_collide(pm, simulation_steps);
  }

  // Part 3: Handle collisions with other primitives.
  for (PointMass &pm : point_masses) {
    for (CollisionObject *co : *collision_objects) {
      co->collide(pm);
    }
  }

  // Part 2: Constrain spring lengths (Provot 1995) - no more than 10% stretch.
  for (Spring &s : springs) {
    Vector3D diff = s.pm_b->position - s.pm_a->position;
    double dist = diff.norm();
    double max_len = s.rest_length * 1.1;

    if (dist > max_len) {
      Vector3D correction = diff.unit() * (dist - max_len);

      if (s.pm_a->pinned && s.pm_b->pinned) {
        continue;
      } else if (s.pm_a->pinned) {
        s.pm_b->position -= correction;
      } else if (s.pm_b->pinned) {
        s.pm_a->position += correction;
      } else {
        s.pm_a->position += correction / 2.0;
        s.pm_b->position -= correction / 2.0;
      }
    }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // Part 4: Build a spatial map out of all of the point masses.
  for (PointMass &pm : point_masses) {
    float h = hash_position(pm.position);
    if (map.find(h) == map.end()) {
      map[h] = new vector<PointMass *>();
    }
    map[h]->push_back(&pm);
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  float h = hash_position(pm.position);

  if (map.find(h) == map.end()) return;

  vector<PointMass *> *candidates = map[h];
  Vector3D correction(0, 0, 0);
  int count = 0;

  for (PointMass *candidate : *candidates) {
    if (candidate == &pm) continue;

    Vector3D diff = pm.position - candidate->position;
    double dist = diff.norm();

    if (dist < 2.0 * thickness) {
      Vector3D corr = diff.unit() * (2.0 * thickness - dist);
      correction += corr;
      count++;
    }
  }

  if (count > 0) {
    pm.position += correction / (double)count / simulation_steps;
  }
}

float Cloth::hash_position(Vector3D pos) {
  double w = 3.0 * width / num_width_points;
  double h = 3.0 * height / num_height_points;
  double t = max(w, h);

  int box_x = (int)floor(pos.x / w);
  int box_y = (int)floor(pos.y / h);
  int box_z = (int)floor(pos.z / t);

  // Combine with large primes for a unique hash
  return (float)(box_x * 73856093 ^ box_y * 19349663 ^ box_z * 83492791);
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
