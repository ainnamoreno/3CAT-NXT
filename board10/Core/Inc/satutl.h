/*
 * satutl.h
 *
 *  Created on: 7 abr. 2022
 *      Author: jose_
 */

#ifndef INC_SATUTL_H_
#define INC_SATUTL_H_

#define ST_SIZE 256



int read_twoline(tle_data tle, orbit_t *orb);
void *vector(size_t num, size_t size);


#endif /* INC_SATUTL_H_ */
