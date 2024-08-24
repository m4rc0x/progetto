#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "boids.hpp"

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

  // condizioni di posizione e velocita
  const double range_x{80.0};
  const double range_y{80.0};
  const double sx_max{5.0};
  const double sy_max{5.0};

  std::vector<pf::boid> boids(n);

  // generazione di numeri pseudocasuali
  pf::fill(boids, range_x, range_y, sx_max, sy_max);
  /*
  for (std::size_t i{0}; i != n; ++i){
    std::cout << " px: " << boids[i].position.get_x() << '\n'; 
    std::cout << " py: " << boids[i].position.get_y() << '\n'; 
    std::cout << " sx: " << boids[i].speed.get_x() << '\n'; 
    std::cout << " sy: " << boids[i].speed.get_y() << '\n'; 
  }
  */
  // near boids
  std::vector<std::vector<pf::boid>> all_near(n);  // NO VETTORE DI VETTORI

  // std::cout <<
  // auto t0 = std::chrono::system_clock::now();

  pf::vector2d v1{0., 0.};;     
  pf::vector2d v2{0., 0.};
  pf::vector2d v3{0., 0.};
  pf::vector2d v{0., 0.};
  pf::vector2d p{0., 0.};

  std::vector<pf::Statistics> result;
  /* std::cout << all_near.size() << "\n";
   std::cout << boids.size() << "\n";  */

  for (long unsigned int i{0}; i < static_cast<long unsigned int>(time); ++i) {
    auto it1 = all_near.begin();
    auto it2 = all_near.end();
    auto it = boids.begin();

    for (; it1 != it2; ++it1, ++it) {
      
      (*it1) = pf::near_boids(boids, d, *it);

      v1 = pf::separation(s, ds, *it, *it1); //MODIFICATO
      v2 = pf::alignment(a, *it, *it1);
      v3 = pf::cohesion(c, *it, *it1);

      v = pf::speed_now(it->speed, v1, v2, v3); 
      p = pf::position_now(it->position, v, 1);

      pf::Weierstrass(*it, range_x, range_y);
      
      //assert(v.norm() < .0); 
      assert(std::abs(p.get_x()) < range_x / 2);
      assert(std::abs(p.get_y()) < range_y / 2);


      
      it->position = p;   //MODIFICATO
      it->speed = v;
    
    }

    result.push_back(pf::statistics(boids));
    std::cout << "mean distance:" << result[i].mean_distance << "+-" << result[i].dev_mean_distance << 
    "    mean speed:" << result[i].mean_speed << "+-" << result[i].dev_mean_speed << "\n";
  }
}