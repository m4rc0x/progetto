#include "boids.hpp"

#include <cassert>
#include <cmath>
#include <random>
#include <stdexcept>

namespace pj {

vector2d::vector2d(double x, double y) : x_{x}, y_{y} {}
double vector2d::get_x() const { return x_; }
double vector2d::get_y() const { return y_; }
void vector2d::set_x(double x) { x_ = x; }
void vector2d::set_y(double y) { y_ = y; }
double vector2d::norm() const { return std::sqrt(x_ * x_ + y_ * y_); }

vector2d operator+(vector2d const &v1, vector2d const &v2) {
  return vector2d{v1.get_x() + v2.get_x(), v1.get_y() + v2.get_y()};
}

vector2d operator-(vector2d const &v1, vector2d const &v2) {
  return vector2d{v1.get_x() - v2.get_x(), v1.get_y() - v2.get_y()};
}

vector2d operator*(vector2d &v, double f) {
  return vector2d{v.get_x() * f, v.get_y() * f};
}

bool operator!=(vector2d const &v1, vector2d const &v2) {
  return (v1.get_x() != v2.get_x()) || (v1.get_y() != v2.get_y());
}

void fill(std::vector<boid> &flock, double range_px, double range_py,
          double range_sx, double range_sy) {
  std::random_device rd;
  std::default_random_engine eng(rd());

  std::uniform_real_distribution<> dis1(-(range_px / 10), (range_px / 10));
  for (auto &boid : flock) {
    boid.position_.set_x(dis1(eng));
  }

  std::uniform_real_distribution<> dis2(-(range_py / 10), (range_py / 10));
  for (auto &boid : flock) {
    boid.position_.set_y(dis2(eng));
  }

  std::uniform_real_distribution<> dis3(-range_sx / 10, range_sx / 10);
  for (auto &boid : flock) {
    boid.speed_.set_x(dis3(eng));
  }

  std::uniform_real_distribution<> dis4(-range_sy / 10, range_sy / 10);
  for (auto &boid : flock) {
    boid.speed_.set_y(dis4(eng));
  }
}

std::vector<boid> near_boids(std::vector<boid> const &boids, double d,
                             boid const &boid_i) {
  std::vector<boid> near;

  for (auto &boid_j : boids) {
    if (((boid_i.position_ - boid_j.position_).norm() < d) &&
        (boid_i.position_ != boid_j.position_)) {
      near.push_back(boid_j);
    }
  }
  if (near.size() < 2) {
    throw std::runtime_error{"not enough near boids to define some operations"};
  }
  return near;
}

vector2d separation(double s, double ds, boid const &boid_i,
                    std::vector<boid> const &near) {
  vector2d sp1{0., 0.};
  vector2d delta_p{0., 0.};
  for (auto &boid_j : near) {
    delta_p = boid_i.position_ - boid_j.position_;

    if (delta_p.norm() < ds) {
      sp1 = sp1 + delta_p;
    }
  }
  return sp1 * (-s);
}

vector2d alignment(double a, boid const &boid_i,
                   std::vector<boid> const &near) {
  vector2d sp2{0., 0.};
  for (auto &boid_j : near) {
    if (boid_j.speed_ != boid_i.speed_) {
      sp2 = sp2 + boid_j.speed_;
    }
  }

  sp2 = ((sp2 * (1 / (static_cast<double>(near.size()) - 1))) - boid_i.speed_);
  return sp2 * a;
}

vector2d cohesion(double c, boid const &boid_i, std::vector<boid> const &near) {
  vector2d pos_cm{0., 0.};
  for (auto &boid_j : near) {
    if (boid_j.speed_ != boid_i.speed_) {
      pos_cm = pos_cm + boid_j.position_;
    }
  }
  vector2d sp3 = (pos_cm * (1 / static_cast<double>((near.size()) - 1))) -
                 boid_i.position_;
  return sp3 * c;
}

vector2d position_now(vector2d &pos, vector2d &sp, double delta_t) {
  pos = pos + (sp * (1 / delta_t));

  assert(pos.get_x() < 1000 && pos.get_y() < 1000);
  return pos;
}

void pacman(boid &boid_ext, double range_px, double range_py) {
  if (std::abs(boid_ext.position_.get_x()) > (range_px / 2)) {
    double sign_x = boid_ext.position_.get_x() * (-1);
    boid_ext.position_.set_x(std::copysign(range_px / 2, sign_x));
  }

  if (std::abs(boid_ext.position_.get_y()) > (range_py / 2)) {
    double sign_y = boid_ext.position_.get_y() * -1;
    boid_ext.position_.set_y(std::copysign(range_py / 2, sign_y));
  }
}

Statistics statistics(std::vector<boid> const &flock) {
  double sum_distance{};
  double sum_speed{};
  double sum_distance2{};
  double sum_speed2{};
  long unsigned int size = flock.size();
  long unsigned int n = size * (size - 1) / 2;

  auto it1 = flock.begin();
  for (; it1 != flock.end(); ++it1) {
    sum_speed += it1->speed_.norm();  // accumula i moduli delle velocità
    sum_speed2 += std::pow(it1->speed_.norm(), 2);

    auto it2 = it1 + 1;

    for (; it2 != flock.end(); ++it2) {
      sum_distance += (it1->position_ - it2->position_).norm();
      sum_distance2 += std::pow((it1->position_ - it2->position_).norm(), 2);
    }
  }

  double mean_distance = sum_distance / static_cast<double>(n);
  double mean_speed = sum_speed / static_cast<double>(size);
  double dev_mean_distance =
      std::sqrt(std::abs((sum_distance2 - static_cast<double>(n) *
                                              mean_distance * mean_distance)) /
                static_cast<double>((n - 1) * n));
  double dev_mean_speed =
      std::sqrt(std::abs((sum_speed2 - static_cast<double>(size) * mean_speed *
                                           mean_speed)) /
                static_cast<double>((size - 1) * size));

  return Statistics{mean_distance, mean_speed, dev_mean_distance,
                    dev_mean_speed};
}

}  // namespace pj