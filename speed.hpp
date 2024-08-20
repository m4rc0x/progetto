#ifndef SPEED_HPP
#define SPEED_HPP

namespace pf{

    vector2d speed_now(vector2d&, vector2d const&, vector2d const&, vector2d const&);

    vector2d position_now(vector2d& pos, vector2d &v, double const delta_t);

    vector2d separation(double const, double const, boid const&, std::vector<boid> const&);
    
    vector2d alignment(double const, boid const&, std::vector<boid> const&);

    vector2d cohesion(double const, boid const&, std::vector<boid> const&); 
        
        
}

#endif
