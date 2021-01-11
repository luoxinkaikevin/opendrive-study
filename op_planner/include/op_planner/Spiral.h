
/// \file MappingHelpers.h
/// \brief Helper functions for mapping operation such as (load and initialize vector maps , convert map from one format to another, .. )
/// \author Hatem Darweesh
/// \date Jul 2, 2016

#ifndef SPIRAL_H_
#define SPIRAL_H_

namespace opendrive
{
	/**
* compute the actual "standard" spiral, starting with curvature 0
* @param s      run-length along spiral
* @param cDot   first derivative of curvature [1/m2]
* @param x      resulting x-coordinate in spirals local co-ordinate system [m]
* @param y      resulting y-coordinate in spirals local co-ordinate system [m]
* @param t      tangent direction at s [rad]
*/

	void odrSpiral(double s, double cDot, double *x, double *y, double *t);
	double sign(double gamma);
	void eulerSpiral(double x0, double y0, double kappa0, double theta0,
					 double gamma, double s, double &real, double &imag, double &theta);

} /* namespace PlannerHNS */

#endif /* MAPPINGHELPERS_H_ */
