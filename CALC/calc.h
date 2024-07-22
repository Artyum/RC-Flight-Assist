#ifndef CALC_CALC_H_
#define CALC_CALC_H_

#include "../config.h"
#include "../RCFA/rcfa.h"

double r2d(double r);
double d2r(double d);
double calc_bearing(s_wsp a, s_wsp b);

#if RADIO_MODE==1

#define EQ_RADIUS			6378.1370
#define POLAR_RADIUS		6356.7523142
#define EARTH_ECC			0.081082

double r2d(double r);
double d2r(double d);

double calc_earth_radius(double lat);
double calc_dist(s_wsp p1, s_wsp p2);
double bearing_chg(double brng, double add);
void calc_target(s_wsp *ret, s_wsp p, double brng, double dist);

#endif
#endif /* CALC_CALC_H_ */
