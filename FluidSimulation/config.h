//
//  config.h
//  FluidSimulation
//
//  Created by Yifan Wang on 2018/3/24.
//  Copyright © 2018年 Yifan Wang. All rights reserved.
//

# import "particle.h"

# ifndef config_h
# define config_h
# endif /* config_h */

const int WINDOW_SIZE = 640;   // Window width

const int LIST_SIZE = 500;    // Number of particles for simulation

Particle particleList[LIST_SIZE];

double springList[LIST_SIZE][LIST_SIZE];

const double input_xMin = 3.0;
const double input_xMax = 7.0;

const double input_yMin = 5.0;
const double input_yMax = 100.0;

const double input_zMin = 0.0;
const double input_zMax = 4.0;

// Config of the tank
const double TANK_xMin = 0.0;
const double TANK_xMax = 20.0;

const double TANK_yMin = 0.0;
const double TANK_yMax = 600.0;

const double TANK_zMin = 0.0;
const double TANK_zMax = 5.0;

/* Initialize particleList such that list is filled with particles */
void initParticleList(void);

/* Simulate in one time interval */
void simulation(void);

/* Calculate velocity of a particle (numerically)*/
void calculateVelocity(Particle*);

/* Update every particle's velocity according to gravity */
void applyGravity(void);

/* Update given particle's velocity according to gravity */
void applyGravityOnOneParticle(Particle*);

/* Save previous position and advance to predicted position */
void positionSaveAndAdvance(void);

/* Use previous position to compute next velocity for every particle */
void computeNextVelocity(void);

/* Modify velocities with pairwise viscosity impulses */
void applyViscosity_Ver3(void);

/* Add and remove springs, change rest lengths */
void adjustSprings_Ver3(void);

/* Modify positions according to double density relaxation */
void doubleDensityRelaxation_Ver3(void);

/* Modify positions according to collisions */
void resolveCollisions_Ver4(void);
void resolveCollisions(void);
void resolveCollisionsHelper(Particle*);
void extra(void);
/* Render Particles */
void render(void);

/* Initialize Workspace*/
void init(void);
void display(void);


