#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include <stdexcept>

#include "doctest.h"    


TEST_CASE("Testing the near boids function"){


    std::vector<pj::boid> flock(8); 
    long unsigned int i{0};
        
    for(; i < 8; ++i){

        flock[i].position_.set_x(static_cast<double>(i) * 5); 
        flock[i].position_.set_y(static_cast<double>(i) * 5); 

    }
    pj::boid boid_i = flock[3];
    std::vector<pj::boid> near = pj::near_boids(flock, 15, boid_i);
    
    SUBCASE ("Testing the correct number of near ones"){
        CHECK(near.size() == 4); 
    }

    SUBCASE("Testing the exception with a short distance"){
        CHECK_THROWS(pj::near_boids(flock, 1, boid_i));
    }

    SUBCASE("Testing the correct estimation of near boids"){
    

        CHECK(near[0].position_.get_x() == 5); 
        CHECK(near[0].position_.get_y() == 5); 
        CHECK(near[1].position_.get_x() == 10); 
        CHECK(near[1].position_.get_y() == 10); 
        CHECK(near[2].position_.get_x() == 20); 
        CHECK(near[2].position_.get_y() == 20); 
        CHECK(near[3].position_.get_x() == 25); 
        CHECK(near[3].position_.get_y() == 25);

    }
}

TEST_CASE("Testing position_now function"){
    pj::vector2d pos; 
    pj::vector2d sp;
    double time{0.1}; 

    pos.set_x(-2.5);
    pos.set_y(4.2);
    sp.set_x(0.5);
    sp.set_y(1.8);
    pos = pj::position_now(pos, sp, time); 
    CHECK(pos.get_x() == 2.5);   
    CHECK(pos.get_y() == 22.2); 
}

TEST_CASE("Testing speed rules functions"){
    double s{0.5}; 
    double ds{10.0};; 
    double a{0.2}; 
    double c{0.3}; 
    std::vector<pj::boid> flock(4); 

    flock[0].position_.set_x(3.0);
    flock[0].position_.set_y(-2.5);
    flock[1].position_.set_x(10.0);
    flock[1].position_.set_y(0.0);
    flock[2].position_.set_x(5.0);
    flock[2].position_.set_y(-5.0);
    flock[3].position_.set_x(4.0);
    flock[3].position_.set_y(2.0);
    
    flock[0].speed_.set_x(1.0);
    flock[0].speed_.set_y(-3.0);
    flock[1].speed_.set_x(4.0);
    flock[1].speed_.set_y(0.5);
    flock[2].speed_.set_x(3.0);
    flock[2].speed_.set_y(-1.0);
    flock[3].speed_.set_x(0.5);
    flock[3].speed_.set_y(3.0);

    SUBCASE("Testing the separation rule"){
        pj::vector2d v1_0 = pj::separation(s, ds, flock[0], flock); 
        CHECK(v1_0.get_x() == 5.0);
        CHECK(v1_0.get_y() == 2.25);

        pj::vector2d v1_1 = pj::separation(s, ds, flock[1], flock); 
        CHECK(v1_1.get_x() == -9.0);
        CHECK(v1_1.get_y() == -2.75);

        pj::vector2d v1_2 = pj::separation(s, ds, flock[2], flock); 
        CHECK(v1_2.get_x() == 1.0);
        CHECK(v1_2.get_y() == 7.25);

        pj::vector2d v1_3 = pj::separation(s, ds, flock[3], flock); 
        CHECK(v1_3.get_x() == 3.0);
        CHECK(v1_3.get_y() == -6.75);
    }


    SUBCASE("Testing the alignment rule"){

         

        pj::vector2d v2_0 = pj::alignment(a, flock[0], flock); 
        
        CHECK(v2_0.get_x() == doctest::Approx(0.3));
        CHECK(v2_0.get_y() == doctest::Approx(0.766667));


        pj::vector2d v2_1 = pj::alignment(a, flock[1], flock); 
        CHECK(v2_1.get_x() == doctest::Approx(-0.5));
        CHECK(v2_1.get_y() == doctest::Approx(-0.166667));

        pj::vector2d v2_2 = pj::alignment(a, flock[2], flock); 
        CHECK(v2_2.get_x() == doctest::Approx(-0.233333));
        CHECK(v2_2.get_y() == doctest::Approx(0.233333));

        pj::vector2d v2_3 = pj::alignment(a, flock[3], flock); 
        CHECK(v2_3.get_x() == doctest::Approx(0.433333));
        CHECK(v2_3.get_y() == doctest::Approx(-0.833333));


    }

    
  
    SUBCASE("Testing the cohesion rule"){
        pj::vector2d v3_0 = pj::cohesion(c, flock[0], flock); 
        CHECK(v3_0.get_x() == doctest::Approx(1.0));
        CHECK(v3_0.get_y() == doctest::Approx(0.45));

        pj::vector2d v3_1 = pj::cohesion(c, flock[1], flock); 
        CHECK(v3_1.get_x() == doctest::Approx(-1.8));
        CHECK(v3_1.get_y() == doctest::Approx(-0.55));

        pj::vector2d v3_2 = pj::cohesion(c, flock[2], flock); 
        CHECK(v3_2.get_x() == doctest::Approx(0.2));
        CHECK(v3_2.get_y() == doctest::Approx(1.45));

        pj::vector2d v3_3 = pj::cohesion(c, flock[3], flock); 
        CHECK(v3_3.get_x() == doctest::Approx(0.6));
        CHECK(v3_3.get_y() == doctest::Approx(-1.35));
    }

   
}

TEST_CASE("Testing the pacman function"){
    std::vector<pj::boid> flock(2);


    flock[0].position_.set_x(600.0);
    flock[0].position_.set_y(1.0);
    flock[1].position_.set_x(0.0);
    flock[1].position_.set_y(-900.0);

    pj::pacman(flock[0], 1000, 1000);
    pj::pacman(flock[1], 1000, 1000);
    CHECK (flock[0].position_.get_x() == -500.0);
    CHECK (flock[0].position_.get_y() == 1.0);
    CHECK (flock[1].position_.get_x() == 0.0);
    CHECK (flock[1].position_.get_y() == 500.0);

   
}

TEST_CASE("Testing the statistic function"){

    std::vector<pj::boid> flock(4);    

    flock[0].position_.set_x(3.0);
    flock[0].position_.set_y(-2.5);
    flock[1].position_.set_x(10.0);
    flock[1].position_.set_y(0.0);
    flock[2].position_.set_x(5.0);
    flock[2].position_.set_y(-5.0);
    flock[3].position_.set_x(4.0);
    flock[3].position_.set_y(2.0);
    
    flock[0].speed_.set_x(1.0);
    flock[0].speed_.set_y(-3.0);
    flock[1].speed_.set_x(4.0);
    flock[1].speed_.set_y(0.5);
    flock[2].speed_.set_x(3.0);
    flock[2].speed_.set_y(-1.0);
    flock[3].speed_.set_x(0.5);
    flock[3].speed_.set_y(3.0);

    pj::Statistics results = pj::statistics(flock); 
    CHECK (results.mean_distance == doctest::Approx(5.95184)); 
    CHECK (results.mean_speed == doctest::Approx(3.34927));
    CHECK (results.dev_mean_distance == doctest::Approx(0.68807).epsilon(0.00001));   
    CHECK (results.dev_mean_speed == doctest::Approx(0.229).epsilon(0.001)); 
 
}


