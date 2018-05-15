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

const double REST_DENSITY = 1.0;   // Rest density: ρ0

const double STIFFNESS = 0.01;    // Stiffness parameter k
const double STIFF_NEAR = 0.9;    // Stiffness parameter k_near
const double STIFF_SPRING = 0.7;//0.32;    // Stiffness parameter k_spring

const double PLASTICITY = 0.04;     // Plasticity constant α

const double VISCOSITY_SIGMA = 2.0; // [5.3] If a highly viscous behavior is desired, σ can be increased.
const double VISCOSITY_BETA = 1.0;  // [5.3] For less viscous fluids, only β should be set to a non-zero value.

const double YIELD_RATIO = 0.1;     // [5.2] Yield ratio, denoted γ, for which choose a value between 0 and 0.2

const double INTERACT_RADIUS = 3; // Radius of interaction h

const double PARTICLE_RADIUS = 0.5; // Radius of a particle

const double TIME_INTERVAL = 1/30.0;  // Time interval

const double REST_LENGTH = PARTICLE_RADIUS;     // Spring rest length

// Structs

/* Position of a particle */
typedef struct Position {
    double x, y, z;
} Position;

/* Velocity of a particle */
typedef struct Velocity {
    double x, y, z;  // Direction
    double velocity; // Numeric
} Velocity;

typedef struct Particle {
    Position prevPosition;  // Previous position
    Position pdctPosition;  // Predicted position
    Velocity velocity;      // Velocity of the particle
    double density;         // Density
    double nearDensity;     // Near density
    int index;
} Particle;


