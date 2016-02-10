//
// Created by ciba on 14.1.2016.
//

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

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

void target_intercept_fabio(int clientID, int camera, int ID, float velocity_a[2]) {
	simxUChar detection_state;
	simxFloat** pp_auxVal = (simxFloat **) malloc(sizeof(simxFloat *) * 6);
	for (int i = 0; i < 6; ++i)
		pp_auxVal[i] = (simxFloat *) malloc(sizeof(simxFloat) * 15);

	simxInt** pp_auxValc = (simxInt **) malloc(sizeof(simxInt *) * 6);
	for (int i = 0; i < 6; ++i)
		pp_auxValc[i] = (simxInt *) malloc(sizeof(simxInt) * 15);

	float target_pos[2];
	int my_validity_vision = 0;
	float distance;
	float err = 0, errpom[2] = { 0.0, 0.0 };
	static int first_smooth = 1;

	static float Sx[2] = { 0.0, 0.0 };
	static float Sy[2] = { 0.0, 0.0 };
	float delta;
	float alfax, alfay;
	float Mi = 0;

	float Ax_array[TAU];
	float Ay_array[TAU];
	float Ax = 0, Ay = 0, dAx = 0, dAy = 0;

	if (simxReadVisionSensor(clientID, camera, &detection_state, pp_auxVal, pp_auxValc, simx_opmode_buffer) == 0) {

		for (int i = 0; i < 2; i++) {
			target_pos[i] = pp_auxVal[0][i + 1] - 0.5;
			if (target_pos[i] < -0.494) {
				target_pos[i] = 0;
				my_validity_vision = 0;
			} else
				my_validity_vision = 1;

			errpom[0] = ERR_VISION * real_rand();
			if (ID == 1 && i == 0) {

				simxWriteStringStream(clientID, "VISION_X_CLEAN", (char *) target_pos, sizeof(distance), simx_opmode_oneshot);
			}
			if (ID == 1 && i == 1) {
				simxWriteStringStream(clientID, "VISION_Y_CLEAN", (char *) (target_pos + 1), sizeof(distance), simx_opmode_oneshot);
			}

			target_pos[i] += errpom[0];
			err += sqrt(errpom[0] * errpom[0]) * 0.017;

			if (ID == 1 && i == 0) {
				simxWriteStringStream(clientID, "VISION_X_NOISE", (char *) target_pos, sizeof(distance), simx_opmode_oneshot);
			}
			if (ID == 1 && i == 1) {
				simxWriteStringStream(clientID, "VISION_Y_NOISE", (char *) (target_pos + 1), sizeof(distance), simx_opmode_oneshot);
			}

		}

		if (first_smooth) {
			Sx[1] = target_pos[0];
			Sy[1] = target_pos[1];
		}
		delta = target_pos[0] - Sx[1];
		alfax = alfa_smooth(delta);
		Sx[0] = alfax * delta + Sx[1];

		delta = target_pos[1] - Sy[1];
		alfay = alfa_smooth(delta);
		Sy[0] = alfay * delta + Sy[1];
		Sx[1] = Sx[0];
		Sy[1] = Sy[0];
		if (ID == 1) {
			simxWriteStringStream(clientID, "VISION_X_SMOOTH", (char *) (Sx), sizeof(distance), simx_opmode_oneshot);
			simxWriteStringStream(clientID, "VISION_Y_SMOOTH", (char *) (Sy), sizeof(distance), simx_opmode_oneshot);
		}

		Mi = fce1(target_pos[0], target_pos[1]);

		float angle = atan2(target_pos[1], target_pos[0]);
		Ax = Mi * cos(angle);
		Ay = Mi * sin(angle);

		//posunuti FIFO + abs(novy prvek)
		for (int i = TAU - 1; i > 0; i--) {
			Ax_array[i] = Ax_array[i - 1];
			Ay_array[i] = Ay_array[i - 1];
		}
		//abs
		Ax_array[0] = Ax >= 0 ? Ax : -Ax;
		Ay_array[0] = Ay >= 0 ? Ay : -Ay;

		//dAx,dAy
		dAx = Ax_array[0] - Ax_array[TAU - 1];
		dAy = Ay_array[0] - Ay_array[TAU - 1];

	} else {
		printf("ERROR: simxReadVisionSensor\n");
	}
	velocity_a[0] = dAx > 0 ? ALFA * Ax + velocity_a[0] : velocity_a[0];
	velocity_a[1] = dAy > 0 ? ALFA * Ay + velocity_a[1] : velocity_a[1];
}

void target_intercept_spline(int clientID, int camera, int ID, float *velocity_a, float *position_current) {
	simxUChar detekce;
	simxFloat** pp_auxVal = (simxFloat **) malloc(sizeof(simxFloat *) * 6);
	for (int i = 0; i < 6; ++i)
		pp_auxVal[i] = (simxFloat *) malloc(sizeof(simxFloat) * 15);

	simxInt** pp_auxValc = (simxInt **) malloc(sizeof(simxInt *) * 6);
	for (int i = 0; i < 6; ++i)
		pp_auxValc[i] = (simxInt *) malloc(sizeof(simxInt) * 15);

	float target_pos[2];
	float distance;
	float err = 0, errpom[2] = { 0.0, 0.0 };
	static int first_smooth = 1;

	static float Sx[2] = { 0.0, 0.0 };
	static float Sy[2] = { 0.0, 0.0 };
	float delta;
	float alfax, alfay;
	float Mi = 0;

	float Ax_array[TAU];
	float Ay_array[TAU];
	float Ax = 0, Ay = 0, dAx = 0, dAy = 0;

	if (simxReadVisionSensor(clientID, camera, &detekce, pp_auxVal, pp_auxValc, simx_opmode_buffer) == 0) {
		for (int i = 0; i < 2; i++) {
			target_pos[i] = pp_auxVal[0][i + 1] - 0.5;
			if (target_pos[i] < -0.494) {
				target_pos[i] = 0;
			}

			errpom[0] = ERR_VISION * real_rand();
			if (ID == 1 && i == 0) {

				simxWriteStringStream(clientID, "VISION_X_CLEAN", (char *) target_pos, sizeof(distance), simx_opmode_oneshot);
			}
			if (ID == 1 && i == 1) {
				simxWriteStringStream(clientID, "VISION_Y_CLEAN", (char *) (target_pos + 1), sizeof(distance), simx_opmode_oneshot);
			}

			target_pos[i] += errpom[0];
			err += sqrt(errpom[0] * errpom[0]) * 0.017;

			if (ID == 1 && i == 0) {
				simxWriteStringStream(clientID, "VISION_X_NOISE", (char *) target_pos, sizeof(distance), simx_opmode_oneshot);
			}
			if (ID == 1 && i == 1) {
				simxWriteStringStream(clientID, "VISION_Y_NOISE", (char *) (target_pos + 1), sizeof(distance), simx_opmode_oneshot);
			}

		}
		for (int j = 0; j < 2; ++j) {
			static float target_last[2];
			static float position_last[2];
			static float velocity_last[2];

			float q0 = position_last[j] * 0.05;
			float qf = target_pos[j] * 1.8;
			float t0 = 0;
			float tf = 0.1;
			float v0 = velocity_last[j];
			float vf = 0;

			float a0 = (qf * t0 * t0 * t0 - q0 * tf * tf * tf + 3 * q0 * t0 * tf * tf - 3 * qf * t0 * t0 * tf + t0 * tf * tf * tf * v0 - t0 * t0 * t0 * tf * vf
					- t0 * t0 * tf * tf * v0 + t0 * t0 * tf * tf * vf) / (t0 - tf) / (t0 - tf) / (t0 - tf);
			float a1 = (t0 * t0 * t0 * vf - tf * tf * tf * v0 - t0 * tf * tf * v0 + 2 * t0 * t0 * tf * v0 - 2 * t0 * tf * tf * vf + t0 * t0 * tf * vf
					- 6 * q0 * t0 * tf + 6 * qf * t0 * tf) / (t0 - tf) / (t0 - tf) / (t0 - tf);
			float a2 = (3 * q0 * t0 - 3 * qf * t0 + 3 * q0 * tf - 3 * qf * tf - t0 * t0 * v0 - 2 * t0 * t0 * vf + 2 * tf * tf * v0 + tf * tf * vf - t0 * tf * v0
					+ t0 * tf * vf) / (t0 - tf) / (t0 - tf) / (t0 - tf);
			float a3 = -(2 * q0 - 2 * qf - t0 * v0 - t0 * vf + tf * v0 + tf * vf) / (t0 - tf) / (t0 - tf) / (t0 - tf);

			float t = 0.045;

			target_last[j] = qf;
			velocity_a[j] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
			position_last[j] = velocity_a[j];
			velocity_last[j] = a1 + a2 * t + a3 * t * t;
//            velocity_a[j] =  target_pos[j];
//velocity_a[j] *=3;

		}

	}
}

void get_target_coordinates(int clientID, int camera_handle, int ID_quadcopter, float target_pos[2], int *p_vision_data_validity) {
	simxUChar detection_state;
	simxFloat** pp_auxVal = (simxFloat **) malloc(sizeof(simxFloat *) * 6);
	for (int i = 0; i < 6; ++i)
		pp_auxVal[i] = (simxFloat *) malloc(sizeof(simxFloat) * 15);

	simxInt** pp_auxValc = (simxInt **) malloc(sizeof(simxInt *) * 6);
	for (int i = 0; i < 6; ++i)
		pp_auxValc[i] = (simxInt *) malloc(sizeof(simxInt) * 15);

	float distance;
	float err = 0, errpom[2] = { 0.0, 0.0 };

	if (simxReadVisionSensor(clientID, camera_handle, &detection_state, pp_auxVal, pp_auxValc, simx_opmode_buffer) == 0) {
		*p_vision_data_validity = 1;
		for (int i = 0; i < 2; i++) {
			target_pos[i] = pp_auxVal[0][i + 1] - 0.5;
			if (target_pos[i] < -0.494) {
				target_pos[i] = 0;
				*p_vision_data_validity = 0;
			} else
				*p_vision_data_validity = 1;

			errpom[0] = ERR_VISION * real_rand();
			if (ID_quadcopter == 1 && i == 0) {

				simxWriteStringStream(clientID, "VISION_X_CLEAN", (char *) target_pos, sizeof(distance), simx_opmode_oneshot);
			}
			if (ID_quadcopter == 1 && i == 1) {
				simxWriteStringStream(clientID, "VISION_Y_CLEAN", (char *) (target_pos + 1), sizeof(distance), simx_opmode_oneshot);
			}

			target_pos[i] += errpom[0];
			err += sqrt(errpom[0] * errpom[0]) * 0.017;

			if (ID_quadcopter == 1 && i == 0) {
				simxWriteStringStream(clientID, "VISION_X_NOISE", (char *) target_pos, sizeof(distance), simx_opmode_oneshot);
			}
			if (ID_quadcopter == 1 && i == 1) {
				simxWriteStringStream(clientID, "VISION_Y_NOISE", (char *) (target_pos + 1), sizeof(distance), simx_opmode_oneshot);
			}

		}
	}
	else{
		target_pos[0] = 0; target_pos[1] = 0;
		*p_vision_data_validity = 0;
	}
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

int detect_neighbours(int clientID,int proximity[2],int ID_QUADCOPTER, float rel_neigh_pos[2][2], float eps[2],float circle_dist){
	float position_neigh[3] = { 0.0f, 0.0f, 0.0f }; // pozice souseda
	float distance; //vzdalenost souseda


	for (int i = 0; i < 2; i++) {
		if (simxReadProximitySensor(clientID, proximity[i], NULL, position_neigh, NULL, NULL, simx_opmode_buffer) == 0) {
			float z = position_neigh[2];
			float y = position_neigh[1];


			float alf[2]; // uhel natoceni proxi senzoru
			switch (ID_QUADCOPTER) {
				case 1:
					alf[0] = 60 * PI / 180;
					alf[1] = 0;
					break;
				case 2:
					alf[0] = -60 * PI / 180;
					alf[1] = -120 * PI / 180;
					break;
				case 3:
					alf[0] = 180 * PI / 180;
					alf[1] = 120 * PI / 180;
					break;
				default:
					break;
			}

//			errpom[0] = real_rand() * ERR_DIST;
			distance = sqrt(z * z + y * y);
//			if (ID_QUADCOPTER == 1) {
//				simxWriteStringStream(clientID, "DIST_CLEAN", (char *) &distance, sizeof(distance), simx_opmode_oneshot);
//			}
//			distance += errpom[0];
//			if (ID_QUADCOPTER == 1) {
//				simxWriteStringStream(clientID, "DIST_NOISE", (char *) &distance, sizeof(distance), simx_opmode_oneshot);
//			}
//			err += sqrt(errpom[0] * errpom[0]);
//
//			if (first_smooth)
//				Sdist[1] = distance;
//
//			delta = distance - Sdist[1];
//			alfax = alfa_smooth(delta);
//			Sdist[0] = alfax * delta + Sdist[1];
//			Sdist[1] = Sdist[0];
//
//			first_smooth = 0;

//			if (ID_QUADCOPTER == 1) {
//				simxWriteStringStream(clientID, "DIST_SMOOTH", (char *) Sdist, sizeof(distance), simx_opmode_oneshot);
//			}
//			distance = Sdist[1];

			rel_neigh_pos[i][0] = distance * cos(alf[i] + atan2(y, z));
			rel_neigh_pos[i][1] = distance * sin(alf[i] + atan2(y, z));

			//                    sprintf(text_pom,"%d:: %f*cos( %f + %f) cov:%f \n",i,distance,alf[i]*180/PI,atan2(y,z)*180/PI,COV_MAT);
			//                    strcat(text,text_pom);

			//eps pro intersection rule
			eps[i] = distance - circle_dist;
			if (eps[i] < 0)
				eps[i] = -eps[i];
			//                    sprintf(text_pom,"eps %d: %f, EPS: %f\n",i,eps[i],EPSILON);
			//                    strcat(text,text_pom);

		} else {
			printf("simxReadProximitySensor\n");
		}
	}


}

void get_neightbour_velocities(int clientID,int ID_QUADCOPTER,float rel_neigh_pos[2][2], int swarm_quantity, int *neigh_valid_count, char *receive_names[3], char **receive_str){

	int *receive_lenght = (int *) malloc(sizeof(int));
	float koule_swarm[swarm_quantity][2];
	float sum_neigh[2] = { 0.0f, 0.0f };

	neigh_valid_count = 0;
	for (int i = 0; i < swarm_quantity; i++) {
		if (i + 1 != ID_QUADCOPTER) {
			if (simxGetStringSignal(clientID, receive_names[i], receive_str, receive_lenght, simx_opmode_buffer) != 0) {
				printf("simxGetStringSignal\n");
			}
			receive_str[0][32] = '\0';
			int val;

			sscanf(*((char **) receive_str), "-v:%d-px-%f,py-%f-- ", &val, koule_swarm[i], koule_swarm[i] + 1);
			if (val == 1) {
				sum_neigh[0] += koule_swarm[i][0];
				sum_neigh[1] += koule_swarm[i][1];
				neigh_valid_count++;

			} else {
				sum_neigh[0] += 0;
				sum_neigh[1] += 0;
			}

		}
	}
	switch (ID_QUADCOPTER) {
		case '1':
			rel_neigh_pos[0][0] += koule_swarm[2 - 1][0];
			rel_neigh_pos[0][1] += koule_swarm[2 - 1][1];

			rel_neigh_pos[1][0] += koule_swarm[3 - 1][0];
			rel_neigh_pos[1][1] += koule_swarm[3 - 1][1];
			break;
		case '2':
			rel_neigh_pos[0][0] += koule_swarm[3 - 1][0];
			rel_neigh_pos[0][1] += koule_swarm[3 - 1][1];

			rel_neigh_pos[1][0] += koule_swarm[1 - 1][0];
			rel_neigh_pos[1][1] += koule_swarm[1 - 1][1];
			break;
		case '3':
			rel_neigh_pos[0][0] += koule_swarm[1 - 1][0];
			rel_neigh_pos[0][1] += koule_swarm[1 - 1][1];

			rel_neigh_pos[1][0] += koule_swarm[2 - 1][0];
			rel_neigh_pos[1][1] += koule_swarm[2 - 1][1];
			break;
	}

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
