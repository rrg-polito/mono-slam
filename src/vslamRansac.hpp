#define DEBUG
// unknown op : for search: FS;

#include <opencv2/opencv.hpp>
#include "camModel.hpp"

#include "Patch.hpp"

#include <eigen3/Eigen/Dense>
#include "ConfigVSLAM.h"

/*  State vector mu consist of:
    
    1. camera vector of r(vector: x, y, z) and q(quantrian)
        and speed translational (v_z) and speed rotational(w_z)

    2. features vectors of :[ fx,fy,fz ]
        "Note that this is y transforemd form:
        psi = [xc yc zc theta phi rho];
        where [xc yc zc ] is the vector for the camera first time f is observed
        phi, theta is : elevation and azmith angel for f
        rho: is the recipoal of d the direct distanse from camera to f
        .i .e: rho: 1/d; which helps represent points at infinity and make the vector (normalize)"

*/
class VSlamFilter {
    CamModel cam;
    // some data structure in CamModel;  
	float dT;
    // time step dT;
    MatrixXf Vmax; //Covariance of speed (linear and rotation)

    MatrixXf Kt;
    MatrixXf Ft;
    MatrixXf Fnt;
    MatrixXf Ht; // measures prediction
    // Ok, Kt: could be kalman matrix.
    //Ft: system model,h_out: H-1 x;tot_h:whole matrix of h.?
    VectorXf h_out;
    VectorXf tot_h_out;

    MatrixXf St; // innovation covariance matrix
    // system covariance
    
    cv::Mat frame, old_frame;
    cv::Mat originalImg;
    cv::Mat drawedImg;
    // drawedImg for what?
    
    
    void drawPrediction();
    // alright predictiction give the next x vector,
    // take the old and Ft?(saved)
    void drawUpdate(cv::Point f);
    
protected:
    // only inner functions or friends can reach
    float map_scale; // scale of the map
    // what map?
    std::vector<Patch> patches;
    // form patch.h ?
    std::vector<Patch> deleted_patches;

    VectorXf mu;
    MatrixXf Sigma;
    // state
	int nFeatures; // number of features
    float T_camera; // ?
    ConfigVSLAM config;// form configslam;
	

    int windowsSize;// size of patch 
    int sigma_pixel;// standard divsion for ?
    int sigma_pixel_2;

	int kernel_min_size;// Patch size?
	int scale;//?
	int sigma_size;// for pridection elipsode?

	int nInitFeatures; // for what?

	int camera_state_dim;//5 vars? 3 trans , 2 rot ?

	float vz_odom; // unknown
	MatrixXf Hvz_odom;// ?
	
	
	double old_ts;// past dT?

	
public:



	float feature_index;
    // why float? what represent?
    VSlamFilter(char *file = NULL);
    // constructur, file of config??
    int addFeature(cv::Point2f pf);
    // what is does goes here?:
    void removeFeature(int index);
    // clear? remove point at index index
    void predict(Vector3f Translation_Speed_Control = Vector3f::Zero(), Vector3f Rotational_Speed_Control = Vector3f::Zero());
    void update(float v_x = 0, float w_z = 0);
    // for EKF alograthem , this is takining the new speed vectors (old: predicton )
    // or (clacluated : update) , and changing the state vector ( or correcting acorrding to k) 
    
    
    void captureNewFrame(cv::Mat newFrame);
    void captureNewFrame(cv::Mat newFrame, double time_stamp);
    // either you porvide the time where the new frame is acqurid or it's 1/frame_rate
    cv::Mat returnImageDrawed();
    // return Image to be drawed ? 
    VectorXf getState();
    // ok get state at any time , and how ?
    void findNewFeatures(int num = -1);
    //how , and is "num"  number of features?
    void convert2XYZ_ifLinear(int index);
    // convert what to XYZ;
    void convert2XYZ_ifLinearAll();
    // ok what are they?
    Vector3f inverseDepth2XyzWorld(VectorXf f, MatrixXf &J, int compute_Jacobian);
    // ok f is input what is it J?
    int numOfFeatures();
    //that should be sizeof(f)?


};
