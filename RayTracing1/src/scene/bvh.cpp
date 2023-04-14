#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}



BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {

  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.

  //cout << "BVHAccel::construct_bvh" << endl;
  
  if (PART <= 2){
    BBox bbox;

    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      bbox.expand(bb);
    }

    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;

    return node;
  }



  BBox bbox;
  int count = 0;

  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
    count ++;
  }

  BVHNode *node = new BVHNode(bbox);
  node->start = start;
  node->end = end;


  if(count > max_leaf_size){
    vector<Primitive*> left_arr;
    vector<Primitive*> right_arr;

    int axis = 0;

    for(int i=1; i<3; i++){
      if (bbox.extent[i] > bbox.extent[axis]){
        axis = i;
      }
    }
    
    double mid = bbox.centroid()[axis];

    //cout << "axis " << axis << endl;

    //cout << "mid = " << mid << endl;


    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();

      //cout << "mid = " << mid << endl;

      //cout << "centroid = " << bb.centroid()[axis]<<endl;


      if(bb.centroid()[axis] <= mid){
        left_arr.push_back(*p);
      }else{
        right_arr.push_back(*p);
      }
    }

    if (left_arr.empty()){
      left_arr.push_back(right_arr.back());
      right_arr.pop_back();
    }

    if (right_arr.empty()){
      right_arr.push_back(left_arr.back());
      left_arr.pop_back();
    }


    std::vector<Primitive *>::iterator temp = start;

    for(int i=0; i<left_arr.size(); i++){
      *temp = left_arr[i];
      temp ++;
    }

    for(int i=0; i<right_arr.size(); i++){
      *temp = right_arr[i];
      temp ++;
    }


    node->l = construct_bvh(start, start + left_arr.size(), max_leaf_size);
    node->r = construct_bvh(start + left_arr.size(), end, max_leaf_size);

  }

  return node;


}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.

  //cout << "BVHAccel::has_intersection" << endl;

  if(PART <= 2){
    for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
    }
    return false;
  }



  double t0 = ray.min_t;
  double t1 = ray.max_t;

  if(!node->bb.intersect(ray,t0,t1)){
    return false;
  }

  if(node->isLeaf()){
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if((*p)->has_intersection(ray)){
        return true;
      }
    }
    return false;
  }

  bool res = has_intersection(ray,node->l) || has_intersection(ray,node->r);

  return res;


}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.

  //cout << "BVHAccel::intersect" << endl;

  if(PART <= 2){
    bool hit = false;
    for (auto p : primitives) {
      total_isects++;
      hit = p->intersect(ray, i) || hit;
    }
    return hit;
  }


  double t0 = ray.min_t;
  double t1 = ray.max_t;

  if(!node->bb.intersect(ray,t0,t1)){
    return false;
  }

 

  if(node->isLeaf()){
    bool found = false;
    for (auto p = node->start; p != node->end; p++) {
      total_isects++;
      if((*p)->intersect(ray,i)){
        found = true;
      }
    }
    return found;
  }

  bool left_found = intersect(ray, i, node->l);
  bool right_found = intersect(ray, i, node->r);

  return left_found || right_found;


}

} // namespace SceneObjects
} // namespace CGL
