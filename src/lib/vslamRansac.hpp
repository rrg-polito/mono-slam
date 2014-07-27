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

#define DEBUG

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>

#include "camModel.hpp"
#include "Patch.hpp"
#include "ConfigVSLAM.h"



/// \brief VSlam filter Class
/// This class incapsulates all the information required to run the filter.
///
class VSlamFilter {

    public:
    /// \brief Filter Contructor
    /// 
    /// This function initializes the filter
    /// 
    /// 
    /// 
    /// \param file Configuration file of the fiter
    VSlamFilter(char *file = NULL);


    /// \brief Process a new frame
    /// 
    /// The function processes a new available frame and 
    /// performs filter prediction and update
    /// 
    /// \param frame The new frame
    /// \param time_stamp The timestamp of the captured frame
    void processNewFrame(cv::Mat frame, double time_stamp);


    /// \brief return the actual state of the filter
    /// 
    /// This function return the frame of the filter, i.e., camera pose 
    /// (position and orientation) and velocities (linear and angular)
    /// 
    /// \return the camera state
    VectorXf getState();




    void captureNewFrame(cv::Mat newFrame, double time_stamp);

    void drawPrediction();

    float feature_index;

    int addFeature(cv::Point2f pf);

    void removeFeature(int index);

    void predict(float v_x = 0, float w_z = 0);
    void update(float v_x = 0, float w_z = 0);



    void captureNewFrame(cv::Mat newFrame);
    cv::Mat returnImageDrawed();


    void findNewFeatures(int num = -1);

    void convert2XYZ(int index);
    void findFeaturesConvertible2XYZ();
    Vector3f depth2XYZ(VectorXf f, MatrixXf &J);

    int numOfFeatures();


    private:
    CamModel cam;

    float dT;

    MatrixXf Vmax; //Covariance of speed (linear and rotation)

    MatrixXf Kt;
    MatrixXf Ft;
    MatrixXf Fnt;
    MatrixXf Ht; // measures prediction
    VectorXf h_out;
    VectorXf tot_h_out;

    MatrixXf St; // innovation covariance matrix



    cv::Mat frame, old_frame;
    cv::Mat originalImg;
    cv::Mat drawedImg;


    void drawUpdate(cv::Point f);

    protected:
    float map_scale; // scale of the map

    std::vector<Patch> patches;
    std::vector<Patch> deleted_patches;

    VectorXf mu;
    MatrixXf Sigma;
    int nFeatures;
    float T_camera;
    ConfigVSLAM config;


    int windowsSize;
    int sigma_pixel;
    int sigma_pixel_2;

    int kernel_min_size;
    int scale;
    int sigma_size;

    int nInitFeatures;

    int camera_state_dim;

    float vz_odom;
    MatrixXf Hvz_odom;


    double old_ts;


};

Vector3f inverse2XYZ(VectorXf f, Vector3f r, MatrixXf &J_hp_f , Matrix3f R = Matrix3f::Identity());
Vector3f inverse2XYZ(VectorXf f, Vector3f r);
