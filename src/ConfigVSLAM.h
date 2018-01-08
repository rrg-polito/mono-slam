/*
 * ConfigVSLAM.h
 *
 *  Created on: Jul 10, 2013
 *      Author: ludovico
 */

#ifndef CONFIGVSLAM_H_
#define CONFIGVSLAM_H_

#include <cstddef>
#include "camModel.hpp"

/* I'll edit here
	yasin 
	what' every parameters means and does?
*/

class ConfigVSLAM {
public:
	ConfigVSLAM(char *file = NULL);

	float sigma_vx, sigma_vy, sigma_vz;
	float sigma_wx, sigma_wy, sigma_wz;

	float rho_0;
	float sigma_rho_0;
/* intializing of feauters inverse depth and covarinace
   sutiable values are in the thises..
*/
	int window_size;
	int sigma_pixel;
	
	int kernel_size;
	int sigma_size;
	
	camConfig camParams;
	int scale;

	float T_camera;
	// it affect the blurring by adding to speed after prediction
	// make it change via gui

	int nInitFeatures;
	int min_features;
	int max_features;

	int forsePlane;

};

#endif /* CONFIGVSLAM_H_ */
