#include "pathtracer.h"

#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"


using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();

  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // For this function, sample uniformly in a hemisphere.

  // Note: When comparing Cornel Box (CBxxx.dae) results to importance sampling, you may find the "glow" around the light source is gone.
  // This is totally fine: the area lights in importance sampling has directionality, however in hemisphere sampling we don't model this behaviour.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);

  // This is the same number of total samples as
  // estimate_direct_lighting_importance (outside of delta lights). We keep the
  // same number of samples for clarity of comparison.
  int num_samples = scene->lights.size() * ns_area_light;
  Vector3D L_out(0.);

  // TODO (Part 3): Write your sampling loop here
  // TODO BEFORE YOU BEGIN
  // UPDATE `est_radiance_global_illumination` to return direct lighting instead of normal shading 

  for(int i=0; i<num_samples; i++){
    Vector3D w_in = hemisphereSampler->get_sample(); //generate random sample direction
    w_in = o2w * w_in;
    Intersection isect_s;    //intsesection of sample with light source
    Ray r_i(hit_p,w_in);    //generate ray from the hit point, this ray samples the light
    r_i.min_t = EPS_F;

    //Monte Carlo
    if(bvh->intersect(r_i,&isect_s)){
      Vector3D L_i = isect_s.bsdf->get_emission();
      Vector3D f_r = isect.bsdf->f(w_out,w_in);
      double pdf = 1./(M_PI * 2);

      L_out += (1./num_samples) * L_i * f_r * dot(Vector3D(0.0, 0.0, 1.0), w_in) / pdf; //Monte Carlo

    }
  }

  return L_out;

}

Vector3D PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  // Estimate the lighting from this intersection coming directly from a light.
  // To implement importance sampling, sample only from lights, not uniformly in
  // a hemisphere.

  // make a coordinate system for a hit point
  // with N aligned with the Z direction.
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  // w_out points towards the source of the ray (e.g.,
  // toward the camera if this is a primary ray)
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0.);

  for(auto light : scene->lights){
    Vector3D L_temp(0.);
    int sample_n;
    if(light->is_delta_light()){
      sample_n = 1;
    }else{
      sample_n = ns_area_light;
    }

    for(int i=0; i<sample_n; i++){
      double pdf,d;
      Vector3D w_in,L_i;

      L_i = light->sample_L(hit_p,&w_in,&d,&pdf);

      Ray r_i(hit_p,w_in);
      Intersection isect_s;
      r_i.min_t = EPS_F;
      r_i.max_t = d - EPS_F;
      w_in = w2o * w_in;
      
      if(!bvh->intersect(r_i,&isect_s)){
        Vector3D f_r = isect.bsdf->f(w_out,w_in);

        L_temp += 1./(sample_n) * f_r * L_i * dot(Vector3D(0.,0.,1.),w_in) / pdf;
      }

    }

    L_out += L_temp;

    
  }
  
  return L_out;

}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  // TODO: Part 3, Task 2
  // Returns the light that results from no bounces of light



  return isect.bsdf->get_emission();


}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  // TODO: Part 3, Task 3
  // Returns either the direct illumination by hemisphere or importance sampling
  // depending on `direct_hemisphere_sample`

  //cout << "PathTracer::one_bounce_radiance" <<endl;
  if(direct_hemisphere_sample){
    //cout << "he" <<endl;
    return estimate_direct_lighting_hemisphere(r,isect);
  }else{
    //cout << "imp" <<endl;
    return estimate_direct_lighting_importance(r,isect);
  }


  


}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();

  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);

  Vector3D L_out(0, 0, 0);

  // TODO: Part 4, Task 2
  // Returns the one bounce radiance + radiance from extra bounces at this point.
  // Should be called recursively to simulate extra bounces.

  //cout << "PathTracer::at_least_one_bounce_radiance" <<endl;


  double die_pos = 0.3;

  if (r.depth <= 0 || coin_flip(die_pos)){
    return L_out;
  }

  L_out = one_bounce_radiance(r,isect);

  double pdf;
  Vector3D w_in,f_i;

  f_i = isect.bsdf->sample_f(w_out, &w_in, &pdf);
  w_in = o2w * w_in;

  Ray r_i(hit_p,w_in);
  r_i.min_t = EPS_F;
  r_i.depth = r.depth - 1;

  Intersection isect_s;

  if(bvh->intersect(r_i,&isect_s)){
    Vector3D f_r = isect.bsdf->f(w_out,w_in);

    L_out += f_r * at_least_one_bounce_radiance(r_i,isect_s) * dot(Vector3D(0.,0.,1.),w_in) / pdf / (1 - die_pos);
  }

  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // You will extend this in assignment 3-2.
  // If no intersection occurs, we simply return black.
  // This changes if you implement hemispherical lighting for extra credit.

  // The following line of code returns a debug color depending
  // on whether ray intersection with triangles or spheres has
  // been implemented.
  //
  // REMOVE THIS LINE when you are ready to begin Part 3.
  /*
  if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;


  L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  */
  // TODO (Part 3): Return the direct illumination.
  
  //cout << "PathTracer::est_radiance_global_illumination" <<endl;
  

  if(PART == 3){
    if (!bvh->intersect(r, &isect)){
      return envLight ? envLight->sample_dir(r) : L_out;
    }

    L_out = (isect.t == INF_D) ? debug_shading(r.d) : estimate_direct_lighting_importance(r,isect) + zero_bounce_radiance(r,isect);
  }else if (PART_4){
    if (!bvh->intersect(r, &isect)){
      return envLight ? envLight->sample_dir(r) : L_out;
    }

    L_out = (isect.t == INF_D) ? debug_shading(r.d) : at_least_one_bounce_radiance(r,isect) + zero_bounce_radiance(r,isect);
  }else{
    if (!bvh->intersect(r, &isect))
    return envLight ? envLight->sample_dir(r) : L_out;


    L_out = (isect.t == INF_D) ? debug_shading(r.d) : normal_shading(isect.n);
  }
  


  // TODO (Part 4): Accumulate the "direct" and "indirect"
  // parts of global illumination into L_out rather than just direct

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  // TODO (Part 1.2):
  // Make a loop that generates num_samples camera rays and traces them
  // through the scene. Return the average Vector3D.
  // You should call est_radiance_global_illumination in this function.

  // TODO (Part 5):
  // Modify your implementation to include adaptive sampling.
  // Use the command line parameters "samplesPerBatch" and "maxTolerance"

  //cout << "PathTracer::raytrace_pixel" <<endl;
  

  int num_samples = ns_aa;          // total samples to evaluate
  Vector2D origin = Vector2D(x, y); // bottom left corner of the pixel


  Vector3D pixel,pixel_temp;

  double x_norm, y_norm;

  double s1 = 0,s2 = 0;

  for(int i=1; i<=num_samples; i++){
    x_norm = x + random_uniform();
    y_norm = y + random_uniform();

    x_norm /= sampleBuffer.w;
    y_norm /= sampleBuffer.h;

    Ray r = camera->generate_ray(x_norm,y_norm);
    r.depth = max_ray_depth;

    pixel_temp = est_radiance_global_illumination(r);
    double illum = pixel_temp.illum();
    
    s1 += illum;
    s2 += illum * illum;

    //cout << "illum " << illum << endl;

    pixel += pixel_temp;

    if(i % samplesPerBatch == 0){
      double mean = 0,var = 0,I = 0;
      mean = s1 / double(i);
      var = (s2 - (s1 * s1 / double(i))) / (double(i) - 1);
      I = 1.96 * sqrt(var) / sqrt(double(i));

      //cout  << s1 << " " << s2 << endl;
      //cout << "I" << I << endl;
      //cout << sqrt(var) << " "<<sqrt(double(i))  << " "<< var <<   " "<<double(i) << endl;
      //cout << "maxTolerance * mean "<< maxTolerance * mean << endl;
      if(I <= maxTolerance * mean){
        //cout << "break"<<endl;
        num_samples = i;
        break;
      }
    }

  }

  //cout << num_samples <<endl;

  pixel /= num_samples;

  sampleBuffer.update_pixel(pixel, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;


}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;

  bvh->intersect(r, &isect);

  camera->focalDistance = isect.t;
}

} // namespace CGL
