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

#include "ConfigVSLAM.h"

#include <libconfig.h++>
#include <iostream>
#include <fstream>
#include <iomanip>


using namespace libconfig;
using namespace std;

template<typename T>
void lookupAndPrintValue(Config & cfg, string name_value, T & value) {
	bool flag = cfg.lookupValue(name_value, value);
	cout << setw(20) << name_value <<  setw(30) << value << setw(30) << (flag ? "Read" : "Default") << endl;
}





ConfigVSLAM::ConfigVSLAM(char *file) {
	sigma_vx  = sigma_vy = sigma_vz = 0.01f;
	sigma_wx  = sigma_wy = sigma_wz = 0.01f;

	window_size = 21;
	sigma_pixel = 2;

	rho_0 = 0.1f;
	sigma_rho_0 = 0.25f;
	
	scale = 2;

	T_camera = 0.5f;

	sigma_size = 2;

	nInitFeatures = 5;

	min_features = 30;
	max_features = 100;

	forsePlane = 0;


	cout << "-------------------------------------------------------------------------------------------------" << endl;

	if (file) {

		Config cfg;
		bool flag;
		cout << "Opening configuration File " << file << endl;

		cfg.readFile(file);
		cout << "Reading configuration File " <<  file << endl;

		cout << "-------------------------------------------------------------------------------------------------" << endl;
		lookupAndPrintValue(cfg, "sigma_vx", sigma_vx);
		lookupAndPrintValue(cfg, "sigma_wx", sigma_wx);
		lookupAndPrintValue(cfg, "sigma_vy", sigma_vy);
		lookupAndPrintValue(cfg, "sigma_wy", sigma_wy);
		lookupAndPrintValue(cfg, "sigma_vz", sigma_vz);
		lookupAndPrintValue(cfg, "sigma_wz", sigma_wz);


		lookupAndPrintValue(cfg, "rho_0", rho_0);
		lookupAndPrintValue(cfg, "sigma_rho_0", sigma_rho_0);

		lookupAndPrintValue(cfg, "window_size", window_size);
		lookupAndPrintValue(cfg, "sigma_pixel", sigma_pixel);
		
		lookupAndPrintValue(cfg, "kernel_size", kernel_size);
		
		lookupAndPrintValue(cfg, "scale", scale);
		lookupAndPrintValue(cfg, "T_camera", T_camera);
		lookupAndPrintValue(cfg, "sigma_size", sigma_size);
		
		lookupAndPrintValue(cfg, "nInitFeatures", nInitFeatures);
		lookupAndPrintValue(cfg, "min_features", min_features);
		lookupAndPrintValue(cfg, "max_features", max_features);

		lookupAndPrintValue(cfg, "forsePlane", forsePlane);


		const Setting &root = cfg.getRoot();
		const Setting &camera = root["camera"];
		
		flag = camera.lookupValue("fx", camParams.fx);
		camParams.fx = camParams.fx/scale;
		cout << setw(20) << "camera.fx" <<  setw(30) << camParams.fx << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("fy", camParams.fy);
		camParams.fy = camParams.fy/scale;
		cout << setw(20) << "camera.fy" <<  setw(30) << camParams.fy << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("u0", camParams.u0);
		camParams.u0 = camParams.u0/scale;
		cout << setw(20) << "camera.u0" <<  setw(30) << camParams.u0 << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("v0", camParams.v0);
		camParams.v0 = camParams.v0/scale;



		cout << setw(20) << "camera.v0" <<  setw(30) << camParams.v0 << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("k1", camParams.k1);
		cout << setw(20) << "camera.k1" <<  setw(30) << camParams.k1 << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("k2", camParams.k2);
		cout << setw(20) << "camera.k2" <<  setw(30) << camParams.k2 << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("k3", camParams.k3);
		cout << setw(20) << "camera.k3" <<  setw(30) << camParams.k3 << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("p1", camParams.p1);
		cout << setw(20) << "camera.p1" <<  setw(30) << camParams.p1 << setw(30) << (flag ? "Read" : "Default") << endl;
		flag = camera.lookupValue("p2", camParams.p2);
		cout << setw(20) << "camera.p2" <<  setw(30) << camParams.p2 << setw(30) << (flag ? "Read" : "Default") << endl;
		




	} else {
		cout << "No configuration File" << endl;
		cout << "-------------------------------------------------------------------------------------------------" << endl;

		cout << setw(20) << "sigma_vx" << setw(30) << sigma_vx << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_vy" << setw(30) << sigma_vy << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_vz" << setw(30) << sigma_vz << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_wx" << setw(30) << sigma_wx << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_wy" << setw(30) << sigma_wy << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_wz" << setw(30) << sigma_wz << setw(30) << "Default" << endl;

		cout << setw(20) << "window_size"<< setw(30) << window_size << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_pixel"<< setw(30) << sigma_pixel << setw(30) << "Default" << endl;
		cout << setw(20) << "rho_0"<< setw(30) << rho_0 << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_rho_0"<< setw(30) << sigma_rho_0 << setw(30) << "Default" << endl;
		cout << setw(20) << "sigma_size"<< setw(30) << sigma_size << setw(30) << "Default" << endl;
		cout << setw(20) << "nInitFeatures"<< setw(30) << nInitFeatures << setw(30) << "Default" << endl;
		cout << setw(20) << "forsePlane"<< setw(30) << forsePlane << setw(30) << "Default" << endl;

        cout << setw(20) << "camera.v0" <<  setw(30) << camParams.v0 << setw(30) << "Default" << endl;
		cout << setw(20) << "camera.k1" <<  setw(30) << camParams.k1 << setw(30) << "Default" << endl;
		cout << setw(20) << "camera.k2" <<  setw(30) << camParams.k2 << setw(30) << "Default" << endl;
		cout << setw(20) << "camera.k3" <<  setw(30) << camParams.k3 << setw(30) << "Default" << endl;
		cout << setw(20) << "camera.p1" <<  setw(30) << camParams.p1 << setw(30) << "Default" << endl;
		cout << setw(20) << "camera.p2" <<  setw(30) << camParams.p2 << setw(30) << "Default" << endl;

	}

	cout << "-------------------------------------------------------------------------------------------------" << endl;


}

