/*
    Copyright (C) 2014 Politecnico di Torino


    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with this program; if not, write to the Free Software Foundation, Inc.,
    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

    Contributors:
        Ludovico O. Russo (ludovico.russo@polito.it)
*/

#ifndef CONFIGVSLAM_H_
#define CONFIGVSLAM_H_

#include <cstddef>
#include "camModel.hpp"



class ConfigVSLAM {
public:
	ConfigVSLAM(char *file = NULL);

	float sigma_vx, sigma_vy, sigma_vz;
	float sigma_wx, sigma_wy, sigma_wz;

	float rho_0;
	float sigma_rho_0;

	int window_size;
	int sigma_pixel;
	
	int kernel_size;
	int sigma_size;
	
	camConfig camParams;
	int scale;

	float T_camera;

	int nInitFeatures;
	int min_features;
	int max_features;

	int forsePlane;

};

#endif /* CONFIGVSLAM_H_ */
