#include <vector>
#include <random>
#include <cmath>
#include <cassert>
#include "position.hpp"
#include "speed.hpp"
//#include <stdexpet> per introdurre throw std::runtime_error (vedi data abstraction)
//cerr e .what()

namespace pf{

    vector2d::vector2d(double x, double y): x_{x}, y_{y} {  //l'assert si deve trovare alla fine del costruttore o all'inizio di un metodo
       //quando l'assert non è verificato, utilizzare l'istruzione terminate
    }         //costruttore
    double vector2d::get_x () const {return x_;}   //è necessario??? dobrebbe essere implementata: parametro di tipo vector2d e v.x_ v.y_
    double vector2d::get_y () const {return y_;} 
    void vector2d::set_x(double x){ x_ = x;} //set_x, set_y
    void vector2d::set_y(double y){ y_ = y;}
    double vector2d::norm() const {    //const
    return std::sqrt(x_ * x_ + y_ * y_);
    }

    //costruttore di default
    //metodi per operazioni vettoriali: somma e prodotto scalare
    //assert(std::abs(x_) < 40.0 && std::abs(y_) < 30;

    /*if(std::abs(x_) < 40.0){
        throw std::runtime_error
    }
    //griglia 80 x 60, condizioni di range per x e y */

vector2d operator+(vector2d const &v1, vector2d const &v2){ //parametri const& negli operatori!!!
    
    vector2d v; 
    v.set_x(v1.get_x()+v2.get_x());
    v.set_y(v1.get_y()+v2.get_y());
    
    return v;
 }

 vector2d operator-(vector2d const &v1, vector2d const &v2){
    
    vector2d v; 
    v.set_x(v1.get_x()-v2.get_x());
    v.set_y(v1.get_y()-v2.get_y());
    
    return v;
 }


 vector2d operator*(vector2d &v, const double f){
    v.set_x(v.get_x()* f);
    v.set_y(v.get_y()* f);
    return v;
 }

bool operator!=(vector2d const &v1, vector2d const &v2){       //controlla bj != bi
    return (v1.get_x() != v2.get_x()) || (v1.get_y() != v2.get_y());
}


   
 void fill (std::vector<boid> &b, double const range_x, double const range_y, double const sx, double const sy){

   std::random_device rd; 
   std::mt19937 gen(rd());  

   std::uniform_real_distribution<> dis1(-(range_x/2),(range_x/2));  
   for(auto& it : b){
    it.position.set_x(dis1(gen));
   }

    std::uniform_real_distribution<> dis2(-(range_y/2),(range_y/2));
   for(auto& it : b){                   //it riparte dal primo elemento eni successivi cicli???
    it.position.set_y(dis2(gen));
   }

   std::uniform_real_distribution<> dis3(-sx,sx);   
     for(auto& it : b){
    it.speed.set_x(dis3(gen));
   }

    std::uniform_real_distribution<> dis4(-sy,sy);   
     for(auto& it : b){
    it.speed.set_y(dis4(gen));
   }

}

std::vector<boid> near_boids (std::vector<boid> const &b, double const d, boid const &b_i)
{
    std::vector<boid> near;
    vector2d b_ij; 
    for(auto& b_j : b){

        b_ij = b_i.position - b_j.position; 

        if(b_ij.norm() < d){
        near.push_back(b_j);
        }
    }
    return near;
}

vector2d speed_now(vector2d& v, vector2d const& v1, vector2d const &v2, vector2d const& v3){
    v = v + v1; 
    v = v + v2; 
    v = v + v3; 

    return v; 
}

vector2d position_now(vector2d& s, vector2d &v, double const delta_t){

   vector2d delta_p = v *(1/delta_t);   //nell'operatore* v non è un parametro const!
    s = s + delta_p;

    return s; 
}

vector2d separation(double const s, double const ds, boid const& b, std::vector<boid> const& near){
    vector2d v1{0.,0.};

    for (auto b1 : near){
    
        vector2d delta_s = b.position - b1.position; 

        if(delta_s.norm() < ds){
            v1 = v1 + delta_s;
        }
        
      
    }
    v1 = v1*(-s);
    return v1;
}

vector2d alignment(double const a, boid const &b, std::vector<boid> const& near)
{
    vector2d v2{0.,0.};
    for(auto b1 : near){

    if(b1.speed != b.speed){
        v2 = v2 + b1.speed;
    
    }
}
    v2 = v2 * (1/(static_cast<double>(near.size()) - 1));
    v2 = v2 - b.speed; 
    v2 = v2*a;

    return v2; 
}

vector2d cohesion(double const c, boid const &b, std::vector<boid> const &near){
    
    vector2d x_cm{0.,0.};
    for(auto b1 : near){
        if(b1.speed != b.speed){
            x_cm = x_cm + b1.position;
        }
    }
    x_cm = x_cm * (1/ static_cast<double>((near.size()) - 1)); 
    vector2d v3 = x_cm - b.position; 
    v3 = v3*c;
    return v3;

    

}

void Weierstrass (boid& b_e, double const range_x, double const range_y){
    

    //ASSERT
    if(std::abs(b_e.position.get_x()) > (range_x/2 - 1) )
    {
        b_e.position.set_x(b_e.position.get_x() * (-1));
    }  

    if(std::abs(b_e.position.get_y()) > (range_y/2 - 1))        
    {

    }


    
        b_e.position.set_y(b_e.position.get_y() * (-1));
    }


Statistics statistics(std::vector<boid> const& boids) {
   
    std::vector<double> distances; 
    std::vector<double> speed_;
    std::vector<double> dev_d;
    std::vector<double> dev_s; 

    double distance{0.}; 

    auto it1 = boids.begin(); 
    for(; it1 != boids.end(); ++it1){

        speed_.push_back((*it1).speed.norm());      //accumula i moduli delle velocità //MODIFICA NON SERVE UN VETTORE!!!!

       auto it2 = it1 + 1; 

        for(; it2 != boids.end(); ++it2){
            
            distance += ((*it1).position - (*it2).position).norm(); 

        }
        distances.push_back(distance);  

    }
    double n = static_cast<double>(distances.size() * (distances.size() -1) / 2);       //NO VETTORE
    double mean_distance = std::accumulate(distances.begin(), distances.end(), 0.) / n;         
    double mean_speed = std::accumulate(speed_.begin(), speed_.end(), 0.) / static_cast<double>(speed_.size()); 

    auto it = distances.begin(); 
    for(; it != distances.end(); ++it){
        dev_d.push_back(std::pow(mean_distance - (*it), 2));
    }
    
    it = speed_.begin();
    for(; it != speed_.end(); ++it){
        dev_s.push_back(std::pow(mean_speed - (*it), 2));
    }

    double dev_distance = std::sqrt((std::accumulate(dev_d.begin(), dev_d.end(),0.))/(n-1));        //SENZA ACCUMULATE 
    double dev_speed = std::sqrt((std::accumulate(dev_s.begin(), dev_s.end(),0.))/(static_cast<double>(speed_.size())-1)); 


    return Statistics{mean_distance, mean_speed, dev_distance, dev_speed};
}
}
