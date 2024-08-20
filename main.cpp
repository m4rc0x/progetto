#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "position.hpp"
#include "speed.hpp"

int main() {
  // parametri del modello di volo
  long unsigned int n;
  double d;
  double ds;
  double s;
  double a;
  double c;
  double time;

  std::cout << "Insert parameters n, d, ds, s, a, c, time  \n";
  std::cin >> n >> d >> ds >> s >> a >> c >> time;
  double delta_t{time / 20};  // TEMPO!!
  // condizioni di posizione e velocita
  const double range_x{80.0};
  const double range_y{40.0};
  const double sx_max{40.0};
  const double sy_max{40.0};

  std::vector<pf::boid> boids(n);

  // generazione di numeri pseudocasuali
  pf::fill(boids, range_x, range_y, sx_max, sy_max);

  // near boids
  std::vector<std::vector<pf::boid>> all_near(n);  // NO VETTORE DI VETTORI

  // std::cout <<
  // auto t0 = std::chrono::system_clock::now();

  pf::vector2d v1;
  pf::vector2d v2;
  pf::vector2d v3;
  pf::vector2d v{0., 0.};
  pf::vector2d p{0., 0.};

  std::vector<pf::Statistics> result;
  /* std::cout << all_near.size() << "\n";
   std::cout << boids.size() << "\n";  */

  for (long unsigned int i{0}; i < static_cast<long unsigned int>((time / delta_t)); ++i) {
    auto it1 = all_near.begin();
    auto it2 = all_near.end();
    auto it = boids.begin();

    for (; it1 != it2; ++it1) {
      (*it1) = pf::near_boids(boids, d, *it);
      v1 = pf::separation(s, ds, *it, *it1);
      v2 = pf::alignment(a, *it, *it1);
      v3 = pf::cohesion(c, *it, *it1);
      v = pf::speed_now((*it).speed, v1, v2, v3);
      p = pf::position_now((*it).position, v, delta_t);

      (*it).position = p;
      (*it).speed = v;
      ++it;
    }

    result.push_back(pf::statistics(boids));
    std::cout << "mean distance:" << result[i].mean_distance << "+-"
              << result[i].dev_distance
              << "    mean speed:" << result[i].mean_speed << "+-"
              << result[i].dev_speed << "\n";
  }
}