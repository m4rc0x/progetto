#include <cassert>
#include <cmath>
#include <random>
#include <vector>
#include <stdexcept>

#include "boids.hpp"

// #include <stdexpet> per introdurre throw std::runtime_error (vedi data
// abstraction) cerr e .what()

namespace pf {

vector2d::vector2d(double x, double y) : x_{x}, y_{y} {  // l'assert si deve trovare alla fine del costruttore o all'inizio di un metodo
  
  }  // quando l'assert non è verificato, utilizzare l'istruzione terminate
double vector2d::get_x() const { return x_;}  
double vector2d::get_y() const { return y_; }
void vector2d::set_x(double x) { x_ = x; }  
void vector2d::set_y(double y) { y_ = y; }
double vector2d::norm() const { 
  return std::sqrt(x_ * x_ + y_ * y_);
}

// metodi per operazioni vettoriali: somma e prodotto scalare
// assert(std::abs(x_) < 40.0 && std::abs(y_) < 30;

/*if(std::abs(x_) < 40.0){
    throw std::runtime_error
}
//griglia 80 x 60, condizioni di range per x e y */

vector2d operator+(vector2d const &v1, vector2d const &v2) { 

  vector2d v;
  v.set_x(v1.get_x() + v2.get_x());
  v.set_y(v1.get_y() + v2.get_y());

  return v;
}

vector2d operator-(vector2d const &v1, vector2d const &v2) {
  vector2d v;
  v.set_x(v1.get_x() - v2.get_x());
  v.set_y(v1.get_y() - v2.get_y());

  return v;
}

vector2d operator*(vector2d &v, double f) {       //prodotto di un vettore per uno scalare
  v.set_x(v.get_x() * f);
  v.set_y(v.get_y() * f);
  return v;
}

bool operator!=(vector2d const &v1, vector2d const &v2) {  // controlla bj != bi
  return (v1.get_x() != v2.get_x()) || (v1.get_y() != v2.get_y());
}

void fill(std::vector<boid> &b, double range_x, double range_y, double sx, double sy) {

  std::random_device rd;
  std::mt19937 gen(rd());

  std::uniform_real_distribution<> dis1(-(range_x / 8), (range_x / 8));
  for (auto &it : b) {
    it.position.set_x(dis1(gen));
  }

  std::uniform_real_distribution<> dis2(-(range_y / 8), (range_y / 8));
  for (auto &it : b) {  
    it.position.set_y(dis2(gen));
  }

  std::uniform_real_distribution<> dis3(-sx, sx);
  for (auto &it : b) {
    it.speed.set_x(dis3(gen));
  }

  std::uniform_real_distribution<> dis4(-sy, sy);
  for (auto &it : b) {
    it.speed.set_y(dis4(gen));
  }
}

std::vector<boid> near_boids(std::vector<boid> const &b, double d, boid const &b_i) {
  std::vector<boid> near;
  vector2d b_ij;
  for (auto &b_j : b) {
    b_ij = b_i.position - b_j.position;

    if (b_ij.norm() < d) {
      near.push_back(b_j);
    }
  }
  if(near.size() < 2){
    throw std::runtime_error("not enough near boids to define some operations");
  }
  return near;
}

vector2d speed_now(vector2d &v, vector2d const &v1, vector2d const &v2, vector2d const &v3) {
  v = v + v1;
  v = v + v2;
  v = v + v3;
  
  return v;
}

vector2d position_now(vector2d &s, vector2d &v, double delta_t) {
  vector2d delta_p = v * (1 / delta_t);  
  s = s + delta_p;

  return s;
}

vector2d separation(double s, double ds, boid const &b, std::vector<boid> const &near) {
  vector2d v1{0., 0.};
    vector2d delta_s{0., 0.};
  for (auto b1 : near) {
      delta_s = b.position - b1.position;

    if (delta_s.norm() < ds) {
          v1 = v1 + delta_s; 
    }
  }
  v1 = v1 * (-s);
  return v1;
}

vector2d alignment(double a, boid const &b, std::vector<boid> const &near) {
  vector2d v2{0., 0.};
  for (auto b1 : near) {
    if (b1.speed != b.speed) {
      v2 = v2 + b1.speed;
    }
  }
 
  v2 = v2 * (1 / (static_cast<double>(near.size()) - 1));
  v2 = v2 - b.speed;
  v2 = v2 * a;

  return v2;
}

vector2d cohesion(double c, boid const &b, std::vector<boid> const &near) {
  vector2d x_cm{0., 0.};
  for (auto b1 : near) {
    if (b1.speed != b.speed) {
      x_cm = x_cm + b1.position;
    }
  }
  x_cm = x_cm * (1 / static_cast<double>((near.size()) - 1));
  vector2d v3 = x_cm - b.position;
  v3 = v3 * c;
  return v3;
}

void Weierstrass(boid &b_e, double range_x, double range_y) {
  
  if (std::abs(b_e.position.get_x()) > (range_x / 2 - 1)) {
    b_e.speed.set_x(b_e.speed.get_x() * (-1));
  }

  if (std::abs(b_e.position.get_y()) > (range_y / 2 - 1)) {
    b_e.speed.set_y(b_e.speed.get_y() * (-1));
  }

  
}

Statistics statistics(std::vector<boid> const &boids) {  

  
  double sum_distance{};
  double sum_speed{};
  double sum_distance2{};
  double sum_speed2{};
  long unsigned int size = boids.size(); 
  long unsigned int n = size * (size - 1) / 2; 

  auto it1 = boids.begin();
  for (; it1 != boids.end(); ++it1) {
    sum_speed += it1->speed.norm();  // accumula i moduli delle velocità 
    sum_speed2 += std::pow(it1->speed.norm(),2);

    auto it2 = it1 + 1;

    for (; it2 != boids.end(); ++it2) {
      sum_distance += (it1->position - it2->position).norm();       
      sum_distance2 += std::pow((it1->position - it2->position).norm(), 2);
    }
    
  }

  double mean_distance = sum_distance / static_cast<double>(n);   //calcolo della distanza media DELLO STORMO!
  double mean_speed = sum_speed / static_cast<double>(size); 
  double dev_mean_distance =  std::sqrt(std::abs((sum_distance2 - static_cast<double>(n) * mean_distance * mean_distance)) / static_cast<double>((n - 1) * n));
  double dev_mean_speed = std::sqrt(std::abs((sum_speed2 - static_cast<double>(size) * mean_speed * mean_speed)) / static_cast<double>((size - 1) * size));

  return Statistics{mean_distance, mean_speed, dev_mean_distance, dev_mean_speed};
}



}  // namespace pf
