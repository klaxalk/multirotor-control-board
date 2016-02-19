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
void target_intercept_fabio(int clientID, int camera, int ID, float velocity_a[2]);
void target_intercept_spline(int clientID, int camera, int ID, float *velocity_a, float *position_current);
int target_intercept_pid(float target_pos[2], float velocity_a[2], float KP, float KI, float KD);
void get_target_coordinates(int clientID, int camera_handle, int ID_quadcopter, float target_pos[2], int *p_vision_data_validity);
int detect_neighbours(int clientID,int proximity[2],int ID_QUADCOPTER, float rel_neigh_pos[2][2], float eps[2],float circle_dist);
void get_neightbour_velocities(int clientID,int ID_QUADCOPTER,float rel_neigh_pos[2][2], int swarm_quantity, int *neigh_valid_count, char *receive_names[3], char **receive_str);
int nearest_intersection(float rel_neigh_pos[2][2], float circle_dist,float p_prusecik_vysledny[2]);
float distance(float x[2]);
int initialize_target(comm2mainMessage_t *message,float *neco);
int find_target(comm2mainMessage_t *message,float target_pos[2],float *kocka);

#endif
