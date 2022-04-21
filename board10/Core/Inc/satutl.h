/*
 * satutl.h
 *
 *  Created on: 7 abr. 2022
 *      Author: jose_
 */

#ifndef INC_SATUTL_H_
#define INC_SATUTL_H_

#define ST_SIZE 256


int read_twoline(char line1[ST_SIZE], char line2[ST_SIZE], long search_satno, orbit_t *orb);
void *vector(size_t num, size_t size);

#endif /* INC_SATUTL_H_ */
