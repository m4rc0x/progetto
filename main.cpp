#include <cassert>
#include <cmath>
#include <iostream>
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

  std::cout
      << "Our program simulates the evolution of a flock according to the "
         "boids model\n"
      << "Mathematical rules that repeatedly define boids' speed and position "
         "are based on the following parameters:\n"
      << "n: number of boids (lower bound: 3, suggested range: 100-1000)\n"
      << "d: distance of near boids expressed in metres (suggested at least "
         "100)\n"
      << "ds: distance used for the separation rule expressed in metres "
         "(suggested range: [0, 1]) \n"
      << "s: factor of separation (allowed range: [0,1], suggested range : [0, "
         "0.2]) \n"
      << "a: factor of alignment (allowed range: [0,1], suggested range : [0, "
         "0.2]) \n"
      << "c: factor of cohesion (allowed range: [0,1], suggested range : [0, "
         "0.2]) \n"
      << "time: duration of the simulation expressed in seconds\n"
      << "Please insert them\n";
  std::cin >> n >> d >> ds >> s >> a >> c >> time;
  assert(n > 2);
  assert(d > 0);
  assert(ds > 0);
  assert(s >= 0 && s <= 1);
  assert(a >= 0 && a <= 1);
  assert(c >= 0 && c <= 1);
  assert(time > 0);

  // condizioni massime di posizione e velocità
  const double range_px{1000.0};
  const double range_py{1000.0};
  const double range_sx{200.0};
  const double range_sy{200.0};

  // vettore dello stormo
  std::vector<pj::boid> flock(n);

  // generazione di numeri pseudocasuali
  pj::fill(flock, range_px, range_py, range_sx, range_sy);

  // il vettore accumula le informazioni relative ai boids vicini
  std::vector<std::vector<pj::boid>> all_near(n);

  // velocità per le regole

  pj::vector2d speed{0., 0.};
  pj::vector2d position{0., 0.};

  // il vettore raccoglie i dati statistici ad ogni unità di tempo
  std::vector<pj::Statistics> result;

  // il ciclo compie un numero di iterazioni pari al valore di time
  for (long unsigned int i{0}; i < static_cast<long unsigned int>(time); ++i) {
    auto it1 = all_near.begin();
    auto it2 = all_near.end();
    auto it = flock.begin();

    for (; it1 != it2; ++it1, ++it) {
      (*it1) = pj::near_boids(flock, d, *it);
      speed = it->speed_;
      // ad ogni iterazione vengono aggiornate le posizioni e le velocità dei
      // boids
      speed = speed + pj::separation(s, ds, *it, *it1);
      speed = speed + pj::alignment(a, *it, *it1);
      speed = speed + pj::cohesion(c, *it, *it1);

      position = pj::position_now(it->position_, speed, 1);

      pj::pacman(*it, range_px, range_py);

      if (std::abs(position.get_x()) > range_px / 2 ||
          std::abs(position.get_y()) > range_py / 2) {
        throw std::runtime_error{"Boundaries have been crossed"};
      }

      if (speed.get_x() > range_sx || speed.get_y() > range_sy) {
        throw std::runtime_error{"Speed limit reached "};
      }

      it->position_ = position;
      it->speed_ = speed;
    }
    result.push_back(pj::statistics(flock));
    std::cout << "mean distance:(" << result[i].mean_distance << "+-"
              << result[i].dev_mean_distance
              << ") m"  // st>d::setprecision()
                 "    mean speed: ("
              << result[i].mean_speed << "+-" << result[i].dev_mean_speed
              << ") m/s\n";
  }
}