#include "sphere.h"

#include <cmath>

#include "pathtracer/bsdf.h"
#include "util/sphere_drawing.h"

namespace CGL {
namespace SceneObjects {

bool Sphere::test(const Ray &r, double &t1, double &t2) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection test.
  // Return true if there are intersections and writing the
  // smaller of the two intersection times in t1 and the larger in t2.


  return true;

}

bool Sphere::has_intersection(const Ray &r) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note that you might want to use the the Sphere::test helper here.


  //cout << "Sphere::has_intersection" <<endl;

  Vector3D O,D,center;

  O = r.o;
  D = r.d;

  center = o;

  double a,b,c,temp,t1,t2;

  a = dot(D,D);
  b = 2 * dot((O-center),D);
  c = dot((O-center),(O-center)) - pow(this->r,2);

  temp = pow(b,2)-4*a*c;

  if(temp >= 0){
    t1 = (-b - pow(temp,0.5)) / (2*a);
    t2 = (-b + pow(temp,0.5)) / (2*a);
    if (t1 > r.min_t && t1 < r.max_t){
      r.max_t = t1;
      return true;
    }else if(t2 > r.min_t && t2 < r.max_t){
      r.max_t = t2;
      return true;
    }

  }

  return false;
}

bool Sphere::intersect(const Ray &r, Intersection *i) const {

  // TODO (Part 1.4):
  // Implement ray - sphere intersection.
  // Note again that you might want to use the the Sphere::test helper here.
  // When an intersection takes place, the Intersection data should be updated
  // correspondingly.

  //cout << "Sphere::intersect" <<endl;

  Vector3D O,D,center;

  O = r.o;
  D = r.d;

  center = o;

  double a,b,c,temp,t1,t2,t;

  a = dot(D,D);
  b = 2 * dot((O-center),D);
  c = dot((O-center),(O-center)) - pow(this->r,2);

  temp = pow(b,2)-4*a*c;

  if(temp >= 0){
    t1 = (-b - pow(temp,0.5)) / (2*a);
    t2 = (-b + pow(temp,0.5)) / (2*a);
    if (t1 > r.min_t && t1 < r.max_t){
      t = t1;

      r.max_t = t;
      i->t = t;

      Vector3D p,normal;
      p = O + t * D;
      normal = p - center;
      normal.normalize();
      i->n = normal;

      i->primitive = this;

      i->bsdf = get_bsdf();
      return true;
    }else if(t2 > r.min_t && t2 < r.max_t){
      t = t2;

      r.max_t = t;
      i->t = t;

      Vector3D p,normal;
      p = O + t * D;
      normal = p - center;
      normal.normalize();
      i->n = normal;

      i->primitive = this;

      i->bsdf = get_bsdf();
      return true;
    }

  }

  return false;

}

void Sphere::draw(const Color &c, float alpha) const {
  Misc::draw_sphere_opengl(o, r, c);
}

void Sphere::drawOutline(const Color &c, float alpha) const {
  // Misc::draw_sphere_opengl(o, r, c);
}

} // namespace SceneObjects
} // namespace CGL
