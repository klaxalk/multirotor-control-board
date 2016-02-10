//
// Created by ciba on 14.1.2016.
//

#ifndef QUAD_POM_FCE_H
#define QUAD_POM_FCE_H

int prusecik(float A[2],float B[2], float dist, float *prus1,float *prus2 );
float fce1(float x,float y);
float fce2(float x,float y,float dx,float dy);
float alfa_smooth(float delta);
int min_smooth(float a,float b,float c);
int target_intercept_pid(float target_pos[2], float velocity_a[2], float KP, float KI, float KD);
void get_target_coordinates(int clientID, int camera_handle, int ID_quadcopter, float target_pos[2], int *p_vision_data_validity);
int nearest_intersection(float rel_neigh_pos[2][2], float circle_dist,float p_prusecik_vysledny[2]);


#endif
