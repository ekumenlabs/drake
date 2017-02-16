#include <iostream>
#include <ignition/math.hh>

int main (int argc, char **argv) {
  ignition::math::Vector3d p(0.0, 1.0, 2.0);
  std::cout << "Hello world!" << std::endl;
  std::cout << "My vector is: " << p << std::endl;
  return 0;
}
