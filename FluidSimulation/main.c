//
//  main.c
//  FluidSimulation
//
//  Created by Yifan Wang on 2018/3/24.
//  Copyright © 2018年 Yifan Wang. All rights reserved.
//

# include <stdio.h>
# include <stdlib.h>
# include <math.h>

# include "config.h"

# include <OpenGL/gl.h>
# include <OpenGl/glu.h>
# include <GLUT/glut.h>

void reshapeFunc(GLint, GLint);
void keyEvent(GLubyte, GLint, GLint);

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowPosition(300, 200);
    glutInitWindowSize(WINDOW_SIZE, WINDOW_SIZE);
    glutCreateWindow("CSE 328 - Project");
    
    init();
    
    glutDisplayFunc(display);
    glutReshapeFunc(reshapeFunc);
    glutKeyboardFunc(keyEvent);
    
    glutMainLoop();
    
    return 0;
}

void init(void) {
    glClearColor(1.0, 1.0, 1.0, 0.0);
    glMatrixMode(GL_MODELVIEW);
    gluLookAt((TANK_xMax - TANK_xMin) / 2, TANK_yMax, TANK_zMax + 5, (TANK_xMax - TANK_xMin) / 2, 0, 0, 0, 1, 0);
    
    // Initialize Position
    initParticleList();
}

void printParticle(Particle* p) {
    printf("%d; (%.2f, %.2f, %.2f)\n", p->index, p->pdctPosition.x, p->pdctPosition.y, p->pdctPosition.z);
    printf("\t; (%.2f, %.2f, %.2f)\n", p->velocity.x, p->velocity.y, p->velocity.z);
}

void initParticleList() {
    double currentX = input_xMin;
    double currentY = input_yMin;
    double currentZ = input_zMin;
    
    for (int i = 0; i < LIST_SIZE; i++) {
        // Index
        particleList[i].index = i;
        
        // Position
        particleList[i].pdctPosition.x = currentX;
        particleList[i].pdctPosition.y = currentY;
        particleList[i].pdctPosition.z = currentZ;
        
        // Calculate next particle's position
        currentX += 2 * PARTICLE_RADIUS;
        if (currentX > input_xMax) {
            currentZ += 2 * PARTICLE_RADIUS;
            currentX = input_xMin;
        }
        if (currentZ > input_zMax) {
            currentX = input_xMin;
            currentZ = input_zMin;
            currentY += 2 * PARTICLE_RADIUS;
        }
        
        // If given block cannot contain that many particles, terminates.
        if (currentY > input_yMax) {
            fprintf(stderr, "\nError: only %d particles can be inserted\n\n", i);
            exit(EXIT_FAILURE);
        }
    }
    
    for(int i = 0; i < LIST_SIZE; i++){
        for(int j = 0; j < LIST_SIZE; j++){
            springList[i][j] = -1;
        }
    }
}

void simulation() {
    // Apply gravity
    applyGravity();
    
    // Modify velocities with pairwise viscosity impulses
    applyViscosity_Ver3();
    
    // Save previous position and advance to predicted position
    positionSaveAndAdvance();
    
    // Add and remove springs, change rest lengths and modify positions according to springs,
    adjustSprings_Ver3();
    
    // Modify positions according to double density relaxation
    doubleDensityRelaxation_Ver3();
    
    // Modify positions according to collisions
    resolveCollisions_Ver4();
    
    // Use previous position to compute next velocity
    computeNextVelocity();
    
    // Extra credit
    extra();
    
    // Rendering
    render();
}

/******************
 *    Gravity
 ******************/

void applyGravity() {
    for (int i = 0; i < LIST_SIZE; i++) {
        applyGravityOnOneParticle(&particleList[i]);
    }
}

void applyGravityOnOneParticle(Particle* particle) {
    (particle->velocity).y -= TIME_INTERVAL * GRAVITY;
    // calculateVelocity(p);
}

/******************
 *  Save & Advance
 ******************/

void positionSaveAndAdvance() {
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleList[i];
        
        // Save previous position
        (p->prevPosition).x = (p->pdctPosition).x;
        (p->prevPosition).y = (p->pdctPosition).y;
        (p->prevPosition).z = (p->pdctPosition).z;
        
        // Advance to predicted position
        (p->pdctPosition).x += TIME_INTERVAL * (p->velocity).x;
        (p->pdctPosition).y += TIME_INTERVAL * (p->velocity).y;
        (p->pdctPosition).z += TIME_INTERVAL * (p->velocity).z;
    }
}

/******************
 * Apply Viscosity
 ******************/
void applyViscosity_Ver3 () {
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleList[i];
        for (int j = i + 1; j < LIST_SIZE; j++) {
            Particle* neighbour = &particleList[j];
            
            const double deltaX = (neighbour->pdctPosition).x - (p->pdctPosition).x;
            const double deltaY = (neighbour->pdctPosition).y - (p->pdctPosition).y;
            const double deltaZ = (neighbour->pdctPosition).z - (p->pdctPosition).z;
            const double distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            
            if (distance > INTERACT_RADIUS) {
                continue;
            }
            
            // Calculate q
            const double q = distance / INTERACT_RADIUS;
            if (q < 1) {
                // inward radical velocity
                const double u = (p->velocity.x - neighbour->velocity.x) * deltaX / distance +
                                    (p->velocity.y - neighbour->velocity.y) * deltaY / distance +
                                    (p->velocity.z - neighbour->velocity.z) * deltaZ / distance;
                if(u > 0) {
                    // Linear and quadratic impulses
                    const double factor = TIME_INTERVAL * (1 - q) * (VISCOSITY_SIGMA * u + VISCOSITY_BETA * u * u);
                    double I[3] = {0, 0, 0};
                    I[0] = factor * deltaX / distance;
                    I[1] = factor * deltaY / distance;
                    I[2] = factor * deltaZ / distance;
                    
                    p->velocity.x = p->velocity.x - I[0] * 0.5;
                    p->velocity.y = p->velocity.y - I[1] * 0.5;
                    p->velocity.z = p->velocity.z - I[2] * 0.5;
                    
                    neighbour->velocity.x = neighbour->velocity.x + I[0] * 0.5;
                    neighbour->velocity.y = neighbour->velocity.y + I[1] * 0.5;
                    neighbour->velocity.z = neighbour->velocity.z + I[2] * 0.5;
                }
            }
        }
    }
}

/*****************
 * Adjust Spring
 *****************/
void adjustSprings_Ver3() {
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleList[i];
        for (int j = i + 1; j < LIST_SIZE; j++) {
            Particle* neighbour = &particleList[j];
            
            const double deltaX = (neighbour->pdctPosition).x - (p->pdctPosition).x;
            const double deltaY = (neighbour->pdctPosition).y - (p->pdctPosition).y;
            const double deltaZ = (neighbour->pdctPosition).z - (p->pdctPosition).z;
            const double distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            
            if (distance > INTERACT_RADIUS) {
                continue;
            }
            
            // Calculate q
            const double q = distance / INTERACT_RADIUS;
            if (q < 1) {
                // If there is no spring ij, add spring ij with rest length h
                if (springList[i][j] != -1) {
                    springList[i][j] = INTERACT_RADIUS;
                }
                // Tolerable deformation = yield ratio * rest length
                double d = YIELD_RATIO * springList[i][j];
                
                if(distance > REST_LENGTH + d) { // Stretch
                    springList[i][j] = springList[i][j] + TIME_INTERVAL * PLASTICITY * (distance - REST_LENGTH - d);
                } else if (distance < REST_LENGTH - d) { // Compress
                    springList[i][j] = springList[i][j] - TIME_INTERVAL * PLASTICITY * (REST_LENGTH - d - distance);
                }
            }
        }
    }
    
    // Remove spring
    for(int i = 0; i < LIST_SIZE; i++) {
        for (int j = i + 1; j < LIST_SIZE; j++) {
            if(springList[i][j] > INTERACT_RADIUS) {
                springList[i][j] = -1.0;
            }
        }
    }
    
    // Spring Displacement
    for(int i = 0; i < LIST_SIZE; i++) {
        for (int j = i + 1; j < LIST_SIZE; j++) {
            if(springList[i][j] != -1) {
                const double deltaX = (&particleList[j])->pdctPosition.x - (&particleList[i])->pdctPosition.x;
                const double deltaY = (&particleList[j])->pdctPosition.y - (&particleList[i])->pdctPosition.y;
                const double deltaZ = (&particleList[j])->pdctPosition.z - (&particleList[i])->pdctPosition.z;
                const double distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
                
                if (distance > INTERACT_RADIUS) {
                    continue;
                }
                
                const double Lij = springList[i][j];
                
                const double factor = TIME_INTERVAL * TIME_INTERVAL * STIFF_SPRING * (1 - Lij / INTERACT_RADIUS) * (Lij - distance);
                
                double D[3] = {0, 0, 0};
                D[0] = factor * deltaX / distance;
                D[1] = factor * deltaY / distance;
                D[2] = factor * deltaZ / distance;
                
                (&particleList[i])->pdctPosition.x = (&particleList[i])->pdctPosition.x - D[0] * 0.5;
                (&particleList[i])->pdctPosition.y = (&particleList[i])->pdctPosition.y - D[1] * 0.5;
                (&particleList[i])->pdctPosition.z = (&particleList[i])->pdctPosition.z - D[2] * 0.5;
                
                (&particleList[j])->pdctPosition.x = (&particleList[j])->pdctPosition.x + D[0] * 0.5;
                (&particleList[j])->pdctPosition.y = (&particleList[j])->pdctPosition.y + D[1] * 0.5;
                (&particleList[j])->pdctPosition.z = (&particleList[j])->pdctPosition.z + D[2] * 0.5;
            }
        }
    }
}

/****************************
 * Double Density Relaxation
 ****************************/
void doubleDensityRelaxation_Ver3() {
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleList[i];
        
        // Reset densities
        p->density = 0;
        p->nearDensity = 0;
        
        // Compute Density And Near-Density
        for (int j = 0; j < LIST_SIZE; j++) {
            if(i == j)
                continue;
            Particle* neighbour = &particleList[j];
            const double deltaX = (neighbour->pdctPosition).x - (p->pdctPosition).x;
            const double deltaY = (neighbour->pdctPosition).y - (p->pdctPosition).y;
            const double deltaZ = (neighbour->pdctPosition).z - (p->pdctPosition).z;
            const double distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            
            if(distance > INTERACT_RADIUS)
                continue;
            
            const double q = distance / INTERACT_RADIUS;
            if(q < 1) {
                p->density = p->density + (1 - q) * (1 - q);
                p->nearDensity = p->nearDensity + (1 - q) * (1 - q) * (1 - q);
            }
        }
        
        // Compute pressure and near pressure
        double P = STIFFNESS * (p->density - REST_DENSITY);
        double P_near = STIFF_NEAR * p->nearDensity;
        double dx[3] = {0, 0, 0};
        for (int j = 0; j < LIST_SIZE; j++) {
            if(i == j)
                continue;
            Particle* neighbour = &particleList[j];
            const double deltaX = (neighbour->pdctPosition).x - (p->pdctPosition).x;
            const double deltaY = (neighbour->pdctPosition).y - (p->pdctPosition).y;
            const double deltaZ = (neighbour->pdctPosition).z - (p->pdctPosition).z;
            const double distance = sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
            
            if(distance > INTERACT_RADIUS)
                continue;
            
            const double q = distance / INTERACT_RADIUS;
            if(q < 1) {
                const double factor = TIME_INTERVAL * TIME_INTERVAL * (P * (1 - q) + P_near * (1 - q) * (1 - q));
                double D[3] = {factor, factor, factor};
                D[0] = D[0] * deltaX / distance;
                D[1] = D[1] * deltaY / distance;
                D[2] = D[2] * deltaZ / distance;
                neighbour->pdctPosition.x = neighbour->pdctPosition.x + D[0] / 2;
                neighbour->pdctPosition.y = neighbour->pdctPosition.y + D[1] / 2;
                neighbour->pdctPosition.z = neighbour->pdctPosition.z + D[2] / 2;
                dx[0] = dx[0] - D[0] / 2;
                dx[1] = dx[1] - D[1] / 2;
                dx[2] = dx[2] - D[2] / 2;
            }
        }
        p->pdctPosition.x = p->pdctPosition.x + dx[0];
        p->pdctPosition.y = p->pdctPosition.y + dx[1];
        p->pdctPosition.z = p->pdctPosition.z + dx[2];
    }
}


int count = 0;
int onair = 1;
/*******************
 *     Collision
 *******************/
void resolveCollisions_Ver4() {
    count = 0;
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleList[i];
        /* Out of X bound */
        if ((p->pdctPosition).x < TANK_xMin) {
            (p->velocity).x *= -0.9;
            (p->pdctPosition).x = TANK_xMin + (p->velocity).x * TIME_INTERVAL;
        } else if ((p->pdctPosition).x > TANK_xMax) {
            (p->velocity).x *= -0.9;
            (p->pdctPosition).x = TANK_xMax - (p->velocity).x * TIME_INTERVAL;
        }
        /* Out of Y bound */
        if ((p->pdctPosition).y <= TANK_yMin) {
            (p->velocity).y *= -0.9;
            (p->pdctPosition).y = TANK_yMin;// + (p->velocity).y * TIME_INTERVAL;
            count++;
            onair = 0;
        } else if ((p->pdctPosition).y > TANK_yMax) {
            (p->velocity).y *= -0.9;
            (p->pdctPosition).y = TANK_yMax - (p->velocity).y * TIME_INTERVAL;
        }
        /* Out of Z bound */
        if ((p->pdctPosition).z < TANK_zMin) {
            (p->velocity).z *= -0.9;
            (p->pdctPosition).z = TANK_zMin + (p->velocity).z * TIME_INTERVAL;
        }/*else if ((p->pdctPosition).z > TANK_zMax) {
            //(p->velocity).z *= -1;
        }*/
    }
}

int justIncr = 0;
int added = 0;
int energyLoss = 0;
const double energyLossPercent = 0.9;

void extra() {
    if(count == 0){
        onair = 1;
        added = 0;
        if(justIncr == 0)
            energyLoss++;
        justIncr = 1;
    }
    if (count > LIST_SIZE * 0.1 && count < LIST_SIZE * 0.8 && onair == 0 && added == 0) {
        justIncr = 0;
        const double dv = sqrt(GRAVITY * input_yMin * 5 * 2 * pow(energyLossPercent, energyLoss));
        for (int i = 0; i < LIST_SIZE; i++) {
            Particle* p = &particleList[i];
            if((p->velocity).y < 0 && (p->pdctPosition).y <= TANK_yMin)
                (p->velocity).y = dv;
            else if ((p->velocity).y < 0) {
                if(p->pdctPosition.y > 0)
                    (p->velocity).y = dv * (1 - (p->pdctPosition.y / input_yMin)) * energyLossPercent;
            }
        }
        added = 1;
    }
}

/******************
 *     Velocity
 ******************/
void computeNextVelocity() {
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleList[i];
        
        // Use previous position to compute next velocity
        (p->velocity).x = ((p->pdctPosition).x - (p->prevPosition).x) / TIME_INTERVAL;
        (p->velocity).y = ((p->pdctPosition).y - (p->prevPosition).y) / TIME_INTERVAL;
        (p->velocity).z = ((p->pdctPosition).z - (p->prevPosition).z) / TIME_INTERVAL;
        
        // Update numeric value
        calculateVelocity(p);
    }
}

void calculateVelocity(Particle* p) {
    /*const double xSquare = (p->velocity).x * (p->velocity).x;
    const double ySquare = (p->velocity).y * (p->velocity).y;
    const double zSquare = (p->velocity).z * (p->velocity).z;
    (p->velocity).velocity = sqrt(xSquare + ySquare + zSquare);*/
}


/*******************
 *      Render
 *******************/
const GLfloat lightPos[] = {10.0, 20.0, -40.0, 0.0};
const int SPHERE_SLICES = 100;
const int SPHERE_STACKS = 100;

void render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
    glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
    glShadeModel(GL_SMOOTH);   // Enable smooth shading
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    gluLookAt(10.0, 10.0, 50.0, 7.0, 0.0, 0.0, 0.0, 1.0, 0.0);
    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);
    glColor3f (1.0, 0.0, 0.0);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    // Sort on z
    Particle particleListTemp[LIST_SIZE] = {0};
    for (int i = 0; i < LIST_SIZE; i++) {
        particleListTemp[i] = particleList[i];
    }
    for (int i = 0; i < LIST_SIZE; i++) {
        for (int j = i + 1; j < LIST_SIZE; j++) {
            if(particleListTemp[i].pdctPosition.z > particleListTemp[j].pdctPosition.z) {
                Particle p = particleListTemp[i];
                particleListTemp[i] = particleListTemp[j];
                particleListTemp[j] = p;
            }
        }
    }
    // Render
    for (int i = 0; i < LIST_SIZE; i++) {
        Particle* p = &particleListTemp[i];
        glTranslatef((p->pdctPosition).x, (p->pdctPosition).y, (p->pdctPosition).z);
        glutSolidSphere(PARTICLE_RADIUS, SPHERE_SLICES, SPHERE_STACKS);
        glTranslatef(-(p->pdctPosition).x, -(p->pdctPosition).y, -(p->pdctPosition).z);
    }
    glFlush();
}

void display(void){
    glClear(GL_COLOR_BUFFER_BIT);
    glColor3f(0.0, 0.0, 1.0);
}

void keyEvent(GLubyte key, GLint xMouse, GLint yMouse){
    switch (key) {
        /* Start */
        case 's':{
            while(1){
                simulation();
            }
        }
        /* Quit */
        case 'q':
            exit(EXIT_SUCCESS);
    }
}

void reshapeFunc(GLint newWidth, GLint newHeight){
    glViewport(0, 0, newWidth, newHeight);
    glMatrixMode(GL_PROJECTION);
    glFrustum(-1.0, 1.5, -1.0, 1.0, 3.7, 200.0);
    glClear(GL_COLOR_BUFFER_BIT);
}

