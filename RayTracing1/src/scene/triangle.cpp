#include "triangle.h"

#include "CGL/CGL.h"
#include "GL/glew.h"

namespace CGL {
namespace SceneObjects {

Triangle::Triangle(const Mesh *mesh, size_t v1, size_t v2, size_t v3) {
  p1 = mesh->positions[v1];
  p2 = mesh->positions[v2];
  p3 = mesh->positions[v3];
  n1 = mesh->normals[v1];
  n2 = mesh->normals[v2];
  n3 = mesh->normals[v3];
  bbox = BBox(p1);
  bbox.expand(p2);
  bbox.expand(p3);

  bsdf = mesh->get_bsdf();
}

BBox Triangle::get_bbox() const { return bbox; }

bool Triangle::has_intersection(const Ray &r) const {
  // Part 1, Task 3: implement ray-triangle intersection
  // The difference between this function and the next function is that the next
  // function records the "intersection" while this function only tests whether
  // there is a intersection.
  //cout << "Triangle::has_intersection" <<endl;

  Vector3D O, D, E1, E2, S, S1, S2;

  O = r.o;
  D = r.d;

  E1 = p2 - p1;
  E2 = p3 - p1;

  S = O - p1;
  S1 = cross(D,E2);
  S2 = cross(S,E1);

  double t,b1,b2;

  t = dot(S2,E2) / dot(S1,E1);
  b1 = dot(S1,S) / dot(S1,E1);
  b2 = dot(S2,D) / dot(S1,E1);

  if(b1 > 0 && b2 > 0 && (1-b1-b2) > 0){
    if (t > r.min_t && t < r.max_t){
      r.max_t = t;
      return true;
    }
  }

  return false;

}

bool Triangle::intersect(const Ray &r, Intersection *isect) const {
  // Part 1, Task 3:
  // implement ray-triangle intersection. When an intersection takes
  // place, the Intersection data should be updated accordingly

  //cout << "Triangle::intersect" <<endl;

  Vector3D O, D, E1, E2, S, S1, S2;

  O = r.o;
  D = r.d;

  E1 = p2 - p1;
  E2 = p3 - p1;

  S = O - p1;
  S1 = cross(D,E2);
  S2 = cross(S,E1);

  double t,b1,b2;

  t = dot(S2,E2) / dot(S1,E1);
  b1 = dot(S1,S) / dot(S1,E1);
  b2 = dot(S2,D) / dot(S1,E1);

  if(b1 > 0 && b2 > 0 && (1-b1-b2) > 0){
    if (t > r.min_t && t < r.max_t){
      r.max_t = t;

      isect->t = t;

      Vector3D normal = (1-b1-b2) * n1 + b1 * n2 + b2 * n3;
      normal.normalize();
      isect->n = normal;

      isect->primitive = this;

      isect->bsdf = get_bsdf();

      return true;
    }
  }
  

  return false;



}

void Triangle::draw(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_TRIANGLES);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

void Triangle::drawOutline(const Color &c, float alpha) const {
  glColor4f(c.r, c.g, c.b, alpha);
  glBegin(GL_LINE_LOOP);
  glVertex3d(p1.x, p1.y, p1.z);
  glVertex3d(p2.x, p2.y, p2.z);
  glVertex3d(p3.x, p3.y, p3.z);
  glEnd();
}

} // namespace SceneObjects
} // namespace CGL
