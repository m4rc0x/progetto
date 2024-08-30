#ifndef BOIDS_HPP
#define BOIDS_HPP

#include <vector>

namespace pj{

 class vector2d{
    
    double x_;    
    double y_;

    public:
    vector2d(double x = 0., double y = 0.); 
    double get_x() const;
    double get_y() const;
    void set_x(double);
    void set_y(double);
    double norm () const;

 };

vector2d operator+(vector2d const&, vector2d const&);
vector2d operator-(vector2d const&, vector2d const&);
vector2d operator*(vector2d &, double);
vector2d operator+=(vector2d const&, vector2d const& );
bool operator!=(vector2d const&, vector2d const&);



 struct boid{
    vector2d position_;
    vector2d speed_;
 };

struct Statistics{     
    double mean_distance;
    double mean_speed; 
    double dev_mean_distance; 
    double dev_mean_speed; 

};

//i boids assumono posizioni e velocità iniziali generate casualmente 
void fill (std::vector<boid>&, double, double, double, double);

//la funzione determina i boids vicini ad ogni boid 
std::vector<boid> near_boids (std::vector<boid> const&, double, boid const&);

//velocità e posizione ad ogni istante di tempo
vector2d position_now(vector2d& pos, vector2d &v, double const delta_t);

//regole di velocità 
vector2d separation(double const, double const, boid const&, std::vector<boid> const&);
vector2d alignment(double const, boid const&, std::vector<boid> const&);
vector2d cohesion(double const, boid const&, std::vector<boid> const&); 

//la funzione controlla il comportamento ai bordi 
void pacman(boid&, double, double); 

Statistics statistics(std::vector<boid> const&);    

}

#endif
