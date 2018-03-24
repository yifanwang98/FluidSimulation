//
//  particle.h
//  FluidSimulation
//
//  Created by Yifan Wang on 2018/3/24.
//  Copyright © 2018年 Yifan Wang. All rights reserved.
//

#ifndef particle_h
#define particle_h
#endif /* particle_h */

// Constants
const double GRAVITY = 9.80665;     // UNIT: m/(s^2)

const double REST_DENSITY = 10.0;   // Rest density: ρ0

const double STIFFNESS = 0.004;     // Stiffness parameter k
const double STIFF_NEAR = 0.01;     // Stiffness parameter k_near
const double STIFF_SPRING = 0.3;    // Stiffness parameter k_spring

const double PLASTICIRY = 0.04;     // Plasticity constant α

const double INTERACTION_R = 5.0;   // Radius of interaction h

const double PARTICLE_RADIUS = 1.0; // Radius of a particle

const int LIST_SIZE = 20000;        // Number of particles for simulation

// Structs

/* Position of a particle */
typedef struct Position {
    float x, y, z;
} Position;

/* Velocity of a particle */
typedef struct Velocity {
    float x, y, z;  // Direction
    float velocity; // Numeric
} Velocity;

typedef struct Particle {
    Position prevPosition;  // Previous position
    Position pdctPosition;  // Predicted position
    Velocity velocity;      // Velocity of the particle
    double density;         // Density
    double nearDensity;     // Near density
    int visited;            // Visited in a loop or not
} Particle;
