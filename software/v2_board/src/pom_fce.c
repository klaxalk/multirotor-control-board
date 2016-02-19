//
// Created by ciba on 14.1.2016.
//

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "commTask.h"
#include <math.h>
#include "pom_fce.h"

#define NON_MATLAB_PARSING
#define MAX_EXT_API_CONNECTIONS 255

#ifndef VREP
#define VREP
//#include "v_repConst.h"
//#include "extApiPlatform.h"
#endif

//#include <extApi.h>
//#include "extApi.h"


//#include "extApi.c"
//#include "extApiPlatform.h"
//#include "extApiPlatform.c"

#define PI (3.141592653589793)
#define COV_MAT 1 // meni tvar krivky pro vypocet Mi, cim vetsi, tim rychlejsi zmeny, Mi=1-exp((x^2+y^2) * COVMAT)
#define TAU 3 //time interval for derivation, higher TAU, more robust
#define ALFA 0.036 //higher ALFA, higher sensitivity to target dynamic's variations
#define ERR_VISION 0.0
#define real_rand() ((double)rand() / (RAND_MAX +1.0) -0.5)

int prusecik(float A[2], float B[2], float dist, float *prus1, float *prus2) {
	float x1 = A[0];
	float y1 = A[1];
	float x3 = B[0];
	float y3 = B[1];

	//        printf("A: %f, %f \nB: %f, %f\ndist: %f\nprus1: %f %f\nprus2: %f %f\n",A[0],A[1],B[0],B[1],dist,prus1[0],prus1[1],prus2[0],prus2[1]);

	float d;
	d = (float) sqrt((A[0] - B[0]) * (A[0] - B[0]) + (A[1] - B[1]) * (A[1] - B[1]));

	float m = 0 + d / 2;

	//    printf("d = %.2f, m= %.2f\n",d,m);
	if (d <= (dist + dist) && d != 0) {

		float v = (float) sqrt((dist * dist - m * m));

		float Sx = x3 + (m / d) * (x1 - x3);
		float Sy = y3 + (m / d) * (y1 - y3);

		//     dve reseni +-

		prus1[0] = Sx + (v / d) * (y3 - y1);
		prus1[1] = Sy - (v / d) * (x3 - x1);

		prus2[0] = Sx - (v / d) * (y3 - y1);
		prus2[1] = Sy + (v / d) * (x3 - x1);

//		printf("pr1: %.2f %.2f   --pr2: %.2f %.2f \n", prus1[0], prus1[1], prus2[0], prus2[1]);
		return 0;
	} else
		return 1;
}

float fce1(float x, float y) {
	float Mi = 1 - exp(-(x * x + y * y) * COV_MAT);
	return Mi;
}

float fce2(float x, float y, float dx, float dy) {
	float dist = sqrt(x * x + y * y);
	float d_dist = sqrt(dx * dx + dy * dy);
	float M = 2 * dist + 0 * d_dist;
	printf("d: %f, d_d:%f", dist, d_dist);
	return M * 0.01;
}

float alfa_smooth(float delta) {

	if (delta < 0)
		delta *= -1;

	if (delta < 0.08) {
		return 0.2;
	} else if (delta > 0.32) {
		return 0.8;
	} else {
		return 2.5 * delta;
	}
}

int min_smooth(float a, float b, float c) {
	if (a < 0)
		a = -a;
	if (b < 0)
		b = -b;
	if (c < 0)
		c = -c;

	if (a <= b && a <= c)
		return 1;
	if (b <= a && b <= c)
		return 2;
	if (c <= a && c <= b)
		return 3;

	return 0;
}

/**
 * Sets velocity according to target's position
 * Parameters:
 * float target_pos[2] - current target position(x,y)
 * int vision_data_validity - 1/0, tells us  whether the data from vision sensor are valid
 * float velocity_a[2] - array for storing calculated thrust(x,y)
 * KP,KI,KD - constants for pid regulator
 */
int target_intercept_pid(float target_pos[2], float velocity_a[2], float KP, float KI, float KD) {
	static float target_pos_int[2];
	static float target_pos_old[2];
	float target_pos_der[2];

		for (int i = 0; i < 2; i++) {
			target_pos_int[i] += target_pos[i];
			target_pos_der[i] = target_pos[i] - target_pos_old[i];
			target_pos_old[i] = target_pos[i];
			velocity_a[i] = KP * target_pos[i] + KI * target_pos_int[i] + KD * target_pos_der[i];
		}
		return 0;
}

int nearest_intersection(float rel_neigh_pos[2][2], float circle_dist,float p_prusecik_vysledny[2]){
	float *p_prusecik1 = (float *) malloc(2 * sizeof(float));
	float *p_prusecik2 = (float *) malloc(2 * sizeof(float));
	int prusecik_error;

	if (prusecik(rel_neigh_pos[0], rel_neigh_pos[1], circle_dist, p_prusecik1, p_prusecik2) != 0) {
		printf("error prusecik\n");
		prusecik_error = 1;
	} else {
		prusecik_error = 0;
	}

	if (sqrt(pow(p_prusecik1[0], 2) + pow(p_prusecik1[1], 2)) > sqrt(pow(p_prusecik2[0], 2) + pow(p_prusecik2[1], 2))) {
		p_prusecik_vysledny[0] = p_prusecik2[0]; p_prusecik_vysledny[1] = p_prusecik2[1];
	} else {
		p_prusecik_vysledny[0] = p_prusecik1[0]; p_prusecik_vysledny[1] = p_prusecik1[1];
	}

	return prusecik_error;
}

float distance(float x[2]){
	return sqrt(x[0]*x[0] + x[1]*x[1]);
}

int initialize_target(comm2mainMessage_t *message, float *neco){
	float a = distance(message->data.bloby[0]);
	float b = distance(message->data.bloby[1]);
	float c = distance(message->data.bloby[2]);
	
	*neco = c;
	
	float x = fmin(a,fmin(b,c));
	
	if(x == a){ return 0;}
	else if(x == b) {return 1;}
	else { return 2;}
		
}

int find_target(comm2mainMessage_t *message,float target_pos[2],float *kocka){
	int i;
	float dist = pow(message->data.bloby[0][0] - target_pos[0],2.0) + pow(message->data.bloby[0][1] - target_pos[1],2.0);
	float min_dist = dist;
	int min_index = 0;
	
	for(i=1;i<message->data.pocet_blobu;i++){
		dist = pow(message->data.bloby[i][0] - target_pos[0],2.0) + pow(message->data.bloby[i][1] - target_pos[1],2.0);
		if(dist < min_dist){
			min_index = i;
			min_dist = dist;
		}
	}
	if(sqrt(min_dist) >= 0.5){
		*kocka = min_dist;
		return 100;
	}else{
		*kocka = min_dist;
		return min_index;
	}
	
}
