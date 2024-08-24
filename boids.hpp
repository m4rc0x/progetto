#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <vector>
#include <cmath>
#include <random>

namespace pf{

 class vector2d{
    
    double x_;      //default inizialitation to 0???
    double y_;
    double sum_norm_; //underscore alla fine dei data members!!!

    public:
    vector2d(double x = 0., double y = 0.); 
    double get_x() const;
    double get_y() const;
    void set_x(double);
    void set_y(double);
    double norm () const;
   

    //costruttore di default
    //metodi per operazioni vettoriali: somma e prodotto scalare
    //assert(std::abs(x_) < 40.0 && std::abs(y_) < 30;

    /*if(std::abs(x_) < 40.0){
        throw std::runtime_error
    }
    //griglia 80 x 60, condizioni di range per x e y */
 };

vector2d operator+(vector2d const&, vector2d const&);
vector2d operator-(vector2d const&, vector2d const&);
vector2d operator*(vector2d &, double);
bool operator!=(vector2d const&, vector2d const&);

 struct boid{
    vector2d position;
    vector2d speed;
 };

struct Statistics{      //sostituire con una classe!!!
    double mean_distance;
    double mean_speed; 
    double dev_mean_distance; 
    double dev_mean_speed; 

};
 

void fill (std::vector<boid>&, double, double, double, double);

//usare il calcolo vettoriale con somma, norma...
//funzione restituisce un vettore dei primi vicini

std::vector<boid> near_boids (std::vector<boid> const&, double, boid const&);

vector2d speed_now(vector2d&, vector2d const&, vector2d const&, vector2d const&);

vector2d position_now(vector2d& pos, vector2d &v, double const delta_t);

vector2d separation(double const, double const, boid const&, std::vector<boid> const&);
    
vector2d alignment(double const, boid const&, std::vector<boid> const&);

vector2d cohesion(double const, boid const&, std::vector<boid> const&); 

void Weierstrass(boid&, double, double); 

Statistics statistics(std::vector<boid> const&);    



}

 /* void near_boids(std::vector<boid> &b, double d, std::vector<int> &near){
    for (auto it : b){
        auto x = it.position.get_x();
        auto y = it.position.get_y();
        int i{0};
        
        for(auto it1 : b){
            auto x1 = it1.position.get_x();
            auto y1 = it1.position.get_y();

            if(std::sqrt(x * x1 + y * y1) < d){
                ++near[i];
            }

        }
        
    }
}
}*/
#endif
