#ifndef __PATCH_CLASS__
#define __PATCH_CLASS__

#include <opencv2/opencv.hpp>
#include <eigen3/Eigen/Dense>




#define XYZ true
#define INV false

#define USE_RANSAC
//USE_RANSAC is for dubug without ransac if omited (or with if like this , included 
// should be deleted because it's default.
// also the coresponding code for ifndef in vslamransac

using namespace Eigen;

class Patch {
protected:
	int nMatch;
    int nMisMatch;
    bool founded;
	bool isInInnovation;
	bool isInLi;
	bool isInLiDef;
	bool isInHi;
	bool removeFlag;
	bool ransacFlag;// Patch requires ransac to be done if true , or ignore ransac on it
	
	float quality_index;
	float ransac_index;
	

public:
	int position_in_z;// useful for patch covariiance extraction from S as it indicate covariance for x,y
	int n_tot, n_find;


	void update_quality_index(float matching_ratio = 0.2);
	float get_quality_index();

	void setIsInInnovation(bool flag);
	void setIsInLi(bool flag);
	void setIsInHi(bool flag);
	void ConfirmIsInLi();

	bool patchIsInInnovation();
	bool patchIsInLi();
	bool patchIsInHi();

	void setRemove();
	bool mustBeRemove();

	bool ransacFound();
	void setRansac();

	void blur(const Vector2f &p1, const  Vector2f &p2, int kernel_min_size);
	void deblur(const Vector2f &p1, const  Vector2f &p2);

	void blurTest(const Vector2f &p1, const  Vector2f &p2, int kernel_min_size, int iName, int deltaT);


	
	int position_in_state;
	bool coding;

    MatrixXd mPatch;// wtf;

    cv::Mat patch;
    cv::Mat matching_patch;

    cv::Mat matched_patch;
    cv::Mat matched_patch_blur;



    cv::Point2f center;
	

	float numMatch;
	float numMismatch;

	bool findMatch(cv::Mat frame, MatrixXf covarianceMatrix, float sigma_size = 3.0f, bool showimg_flag = false);
	Patch(const cv::Mat &p, cv::Point2f c, int position = 0);
	
	Vector2f z;
	Vector2f h;
	
	MatrixXf H; 
	
	Vector3f XYZ_pos;

	
	void change_position(int dp);



	void drawUpdate(cv::Mat &img, int index);
	
	bool isXYZ();
	void convertInXYZ();

	int imgCounter;

};

#endif
