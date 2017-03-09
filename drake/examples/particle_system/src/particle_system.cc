





DEFINE_double(num_of_particles, 0.0, );
DEFINE_double(initial_vel, 0.0, );
DEFINE_double(initial_accel, 1.0, );

int main (int argc, char **argv) {
  return drake::particle::main(argc, argv);
}
