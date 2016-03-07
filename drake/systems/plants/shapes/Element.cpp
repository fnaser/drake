#include "Element.h"

#include <iostream>

using namespace Eigen;

namespace DrakeShapes {
Element::Element(const Element& other)
    : T_element_to_world(other.getWorldTransform()),
      T_element_to_local(other.getLocalTransform()),
      geometry(other.getGeometry().clone()) {}

Element* Element::clone() const { return new Element(*this); }

const Isometry3d& Element::getWorldTransform() const {
  return T_element_to_world;
}

const Isometry3d& Element::getLocalTransform() const {
  return T_element_to_local;
}

const Shape Element::getShape() const { return geometry->getShape(); }

void Element::setGeometry(const Geometry& geometry) {
  this->geometry = std::unique_ptr<Geometry>(geometry.clone());
}

const Geometry& Element::getGeometry() const { return *geometry; }

bool Element::hasGeometry() const { return geometry != nullptr; }

void Element::getTerrainContactPoints(Eigen::Matrix3Xd& local_points) {
  if (!hasGeometry()) {
    local_points = Eigen::Matrix3Xd();
    return;
  }

  Eigen::Matrix3Xd points;
  geometry->getTerrainContactPoints(points);

  local_points = T_element_to_local * points;
}

void Element::setLocalTransform(const Eigen::Isometry3d& T_element_to_local) {
  this->T_element_to_local = T_element_to_local;
}

void Element::updateWorldTransform(const Eigen::Isometry3d& T_local_to_world) {
  setWorldTransform(T_local_to_world * (this->T_element_to_local));
}

void Element::setWorldTransform(const Isometry3d& T_element_to_world) {
  this->T_element_to_world = T_element_to_world;
}

std::ostream& operator<<(std::ostream& os, const Element& e) {
  os << "Element:\n"
            << " - world transform:\n" << e.getWorldTransform().matrix() << "\n"
            << " - local transform:\n" << e.getLocalTransform().matrix()
            << std::endl;
  return os;
}

}
