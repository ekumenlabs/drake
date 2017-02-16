#include <iostream>
#include <ignition/math.hh>
#include <manifold/RoadNetwork.hh>
#include <manifold/rndf/RNDF.hh>

int main (int argc, char **argv) {
  ignition::math::Vector3d p(0.0, 1.0, 2.0);
  std::cout << "Hello world!" << std::endl;
  std::cout << "My vector is: " << p << std::endl;

  manifold::rndf::RNDF info("/tmp/manifold/test/rndf/sample1.rndf");
  manifold::RoadNetwork network(info);

  for (auto vertex : network.Graph().Vertexes()) {
    std::cout << vertex->Name() << std::endl;
  }

  return 0;
}
