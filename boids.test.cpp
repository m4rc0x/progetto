#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "boids.hpp"
#include <stdexcept>

#include "doctest.h"    
//UTILIZZARE DOCTEST.APPROX!!! E EPSILON
//per popolare usare generatori di numeri casuali????

TEST_CASE("Testing the near boids function"){

 

    std::vector<pf::boid> boids(8); 
    long unsigned int i{0};
        
    for(; i < 8; ++i){

        boids[i].position.set_x(static_cast<double>(i) * 5); 
        boids[i].position.set_y(static_cast<double>(i) * 5); 

    }
    pf::boid b = boids[3];
    std::vector<pf::boid> n = pf::near_boids(boids, 15, b);
    
    SUBCASE ("Testing the correct number of near ones"){
        CHECK(n.size() == 5); 
    }

    SUBCASE("Testing the exception"){
        CHECK_THROWS(pf::near_boids(boids, 1, b));
    }

    SUBCASE("Testing the correct estimation of near boids"){
    
        CHECK(n.size() == 5); 
        CHECK(n[0].position.get_x() == 5); 
        CHECK(n[0].position.get_y() == 5); 
        CHECK(n[1].position.get_x() == 10); 
        CHECK(n[1].position.get_y() == 10); 
        CHECK(n[2].position.get_x() == 15); 
        CHECK(n[2].position.get_y() == 15); 
        CHECK(n[3].position.get_x() == 20); 
        CHECK(n[3].position.get_y() == 20);
        CHECK(n[4].position.get_x() == 25); 
        CHECK(n[4].position.get_y() == 25);
    }
}


TEST_CASE("Testing speed_now function"){

    pf::vector2d v;
    pf::vector2d v1; 
    pf::vector2d v2;
    pf::vector2d v3;

    v.set_x(0.5);
    v.set_y(1.0);
    v1.set_x(2.0);
    v1.set_y(1.5);
    v2.set_x(-0.9);
    v2.set_y(5.4);
    v3.set_x(3.1);
    v3.set_y(-2.7);
    v = pf::speed_now(v, v1, v2, v3);

    CHECK(v.get_x() == 4.7); 
    CHECK(v.get_y() == 5.2); 
}

TEST_CASE("Testing position_now function"){
    pf::vector2d position; 
    pf::vector2d speed;
    double time{0.1}; 

    position.set_x(-2.5);
    position.set_y(4.2);
    speed.set_x(0.5);
    speed.set_y(1.8);
    pf::vector2d p = pf::position_now(position, speed, time); 
    CHECK(p.get_x() == 2.5);   
    CHECK(p.get_y() == 22.2); 
}

TEST_CASE("Testing speed rules functions"){
    double s{0.5}; 
    double ds{10.0};; 
    double a{0.2}; 
    double c{0.3}; 
    std::vector<pf::boid> boids(4); 

    boids[0].position.set_x(3.0);
    boids[0].position.set_y(-2.5);
    boids[1].position.set_x(10.0);
    boids[1].position.set_y(0.0);
    boids[2].position.set_x(5.0);
    boids[2].position.set_y(-5.0);
    boids[3].position.set_x(4.0);
    boids[3].position.set_y(2.0);
    
    boids[0].speed.set_x(1.0);
    boids[0].speed.set_y(-3.0);
    boids[1].speed.set_x(4.0);
    boids[1].speed.set_y(0.5);
    boids[2].speed.set_x(3.0);
    boids[2].speed.set_y(-1.0);
    boids[3].speed.set_x(0.5);
    boids[3].speed.set_y(3.0);

    SUBCASE("Testing the separation rule"){
        pf::vector2d v1_0 = pf::separation(s, ds, boids[0], boids); 
        CHECK(v1_0.get_x() == 5.0);
        CHECK(v1_0.get_y() == 2.25);

        pf::vector2d v1_1 = pf::separation(s, ds, boids[1], boids); 
        CHECK(v1_1.get_x() == -9.0);
        CHECK(v1_1.get_y() == -2.75);

        pf::vector2d v1_2 = pf::separation(s, ds, boids[2], boids); 
        CHECK(v1_2.get_x() == 1.0);
        CHECK(v1_2.get_y() == 7.25);

        pf::vector2d v1_3 = pf::separation(s, ds, boids[3], boids); 
        CHECK(v1_3.get_x() == 3.0);
        CHECK(v1_3.get_y() == -6.75);
    }


    SUBCASE("Testing the alignment rule"){

         

        pf::vector2d v2_0 = pf::alignment(a, boids[0], boids); 
        
        CHECK(v2_0.get_x() == doctest::Approx(0.3));
        CHECK(v2_0.get_y() == doctest::Approx(0.766667));


        pf::vector2d v2_1 = pf::alignment(a, boids[1], boids); 
        CHECK(v2_1.get_x() == doctest::Approx(-0.5));
        CHECK(v2_1.get_y() == doctest::Approx(-0.166667));

        pf::vector2d v2_2 = pf::alignment(a, boids[2], boids); 
        CHECK(v2_2.get_x() == doctest::Approx(-0.233333));
        CHECK(v2_2.get_y() == doctest::Approx(0.233333));

        pf::vector2d v2_3 = pf::alignment(a, boids[3], boids); 
        CHECK(v2_3.get_x() == doctest::Approx(0.433333));
        CHECK(v2_3.get_y() == doctest::Approx(-0.833333));


    }

    
  
    SUBCASE("Testing the centre of mass"){
        pf::vector2d v3_0 = pf::cohesion(c, boids[0], boids); 
        CHECK(v3_0.get_x() == doctest::Approx(1.0));
        CHECK(v3_0.get_y() == doctest::Approx(0.45));

        pf::vector2d v3_1 = pf::cohesion(c, boids[1], boids); 
        CHECK(v3_1.get_x() == doctest::Approx(-1.8));
        CHECK(v3_1.get_y() == doctest::Approx(-0.55));

        pf::vector2d v3_2 = pf::cohesion(c, boids[2], boids); 
        CHECK(v3_2.get_x() == doctest::Approx(0.2));
        CHECK(v3_2.get_y() == doctest::Approx(1.45));

        pf::vector2d v3_3 = pf::cohesion(c, boids[3], boids); 
        CHECK(v3_3.get_x() == doctest::Approx(0.6));
        CHECK(v3_3.get_y() == doctest::Approx(-1.35));
    }
}


TEST_CASE("Testing the statistic function"){

    std::vector<pf::boid> boids(4);     //stesse condizioni iniziali del test sulle regole di velocità

    boids[0].position.set_x(3.0);
    boids[0].position.set_y(-2.5);
    boids[1].position.set_x(10.0);
    boids[1].position.set_y(0.0);
    boids[2].position.set_x(5.0);
    boids[2].position.set_y(-5.0);
    boids[3].position.set_x(4.0);
    boids[3].position.set_y(2.0);
    
    boids[0].speed.set_x(1.0);
    boids[0].speed.set_y(-3.0);
    boids[1].speed.set_x(4.0);
    boids[1].speed.set_y(0.5);
    boids[2].speed.set_x(3.0);
    boids[2].speed.set_y(-1.0);
    boids[3].speed.set_x(0.5);
    boids[3].speed.set_y(3.0);

    pf::Statistics results = pf::statistics(boids); 
    CHECK (results.mean_distance == doctest::Approx(5.95184)); 
    CHECK (results.mean_speed == doctest::Approx(3.34927));
    CHECK (results.dev_mean_distance == doctest::Approx(0.68807).epsilon(0.00001));   
    CHECK (results.dev_mean_speed == doctest::Approx(0.229).epsilon(0.001)); 
 
}

TEST_CASE("Testing the Weiestrass function"){
    std::vector<pf::boid> boids(2);


    boids[0].position.set_x(45.0);
    boids[0].position.set_y(1.0);
    boids[1].position.set_x(0.0);
    boids[1].position.set_y(-90.0);


    boids[0].speed.set_x(2.0);
    boids[0].speed.set_y(-1.0);
    boids[1].speed.set_x(-1.3);
    boids[1].speed.set_y(0.6);

    pf::Weierstrass(boids[0], 40, 80);
    pf::Weierstrass(boids[1], 40, 80);
    CHECK (boids[0].speed.get_x() == -2.0);
    CHECK (boids[0].speed.get_y() == -1.0);
    CHECK (boids[1].speed.get_x() == -1.3);
    CHECK (boids[1].speed.get_y() == -0.6);

   
}
