#include "Patch.hpp"
#include <stdlib.h>
#include <stdio.h>
#include "libblur.h"

#include <fstream>

float computeCorrelation(cv::Mat f1, cv::Mat f2);

// TODO: put the parameter in other place..
// what if many comparing evaluate to bigger than 0.8
// that means it's  not optimal so..
float patch_matching_threshold = 0.8;

void Patch::blurTest(const Vector2f &p1, const  Vector2f &p2, int kernel_min_size, int iName, int deltaT) {
	/*
		this function sound for debugging porposes , for it shows and save th blured_patch
		we could enhanse this by creating a whole debug mode or interface 
		and showing both the original and blured patch beside each other.
	*/
	/*
		TODO: no need to image write , 
		it test values for T_camera , to better match the real blurring effect

	*/

	cv::Mat blurredPatch;
	if ((p1-p2).norm() > kernel_min_size) {
		// patch is global
		blurredPatch = blurPatch(patch, cv::Point2f(p1(0), p1(1)), cv::Point2f(p2(0), p2(1))).clone();
	} else {
		blurredPatch = patch.clone();
	}

	char name[100];
	/* are there any string ways instead of arrays of chars
		imgCounter is only for blurTest..
	*/
	sprintf(name, "%04d-%02d-%03d.jpg", imgCounter++, iName, deltaT);
	std::cout << name << std::endl;

	cv::imwrite(name, blurredPatch);
	cv::imshow("cioa", blurredPatch);
	cv::waitKey(1);

}


void Patch::blur(const Vector2f &p1, const  Vector2f &p2, int kernel_min_size) {

	if ((p1-p2).norm() > kernel_min_size) {
		matching_patch = blurPatch(patch, cv::Point2f(p1(0), p1(1)), cv::Point2f(p2(0), p2(1))).clone();
	} else {
		matching_patch = patch.clone();
	}
}


void Patch::deblur(const Vector2f &p1, const  Vector2f &p2) {
	patch = deblurPatch(patch, cv::Point2f(p1(0), p1(1)), cv::Point2f(p2(0), p2(1)));
}


void Patch::change_position(int dp) {
	position_in_state += dp;
}


bool Patch::isXYZ() {
	return coding;
}

void Patch::convertInXYZ() {
	coding = XYZ;
}

Patch::Patch(const cv::Mat &p, cv::Point2f c, int position) {
	numMatch = 1;
	// where it's used and misMatch, and for what??
	numMismatch = 0;
	n_tot = 1;
	n_find = 1;
	isInInnovation = false;
	isInLi = false;
	isInHi = false;
	removeFlag = false;

	patch = p.clone();
	center = c;
	ransacFlag = true;


	quality_index = 0.0f;
	coding = INV;
	position_in_state = position;

	imgCounter = 0;

}

void Patch::setRansac() {
	ransacFlag = false;
}


bool Patch::ransacFound() {
	return ransacFlag;
}


void Patch::setRemove() {
	removeFlag = true;
}
bool Patch::mustBeRemove() {
	return removeFlag;
}


void Patch::setIsInInnovation(bool flag) {
	isInInnovation = flag;
	isInLi = isInHi = false;
	ransacFlag = flag;
	isInLiDef = false;
#ifndef USE_RANSAC
	//	n_find++;
#endif

}

void Patch::setIsInLi(bool flag){
	isInLi = flag;
}

// useless
void Patch::ConfirmIsInLi(){
	isInLiDef = isInLi;
}

void Patch::update_quality_index(float matching_ratio = 0.2) {

	if (this->patchIsInHi() || this->patchIsInLi()) n_find++;

	quality_index = (float)(n_tot - n_find)/((float)n_find);

	if (quality_index > matching_ratio) this->setRemove();
}


float Patch::get_quality_index() {
	return quality_index;

}




void Patch::setIsInHi(bool flag) {
	isInHi = flag;

	/*
	numMatch = 0.9*numMatch;
	numMismatch = 0.9*numMismatch;
	*/
	
	// ransac index seems useless!

	if(!isInHi) ransac_index = 0.5;
	else {
		ransac_index = 0;
#ifdef USE_RANSAC
		//n_find++;
#endif
	}

}

bool Patch::patchIsInInnovation() {
	return isInInnovation;
}

bool Patch::patchIsInLi(){
	return isInLi;
}

bool Patch::patchIsInHi(){
	return isInHi;
}



void Patch::drawUpdate(cv::Mat &img, int index) {

	//std::cout << "feature " << index << "is in innovation = " << this->patchIsInInnovation() << std::endl;

	if (!this->patchIsInInnovation()) return;
	// draw only how match correctly!
	int windowSize = patch.cols;
    cv::rectangle(img, cv::Point(z(0) - windowSize/2, z(1) - windowSize/2), cv::Point(z(0) + windowSize/2, z(1) + windowSize/2), CV_RGB(0,0,255), 1);
    //cv::Rect roi = cv::Rect(z(0) - windowSize/2, z(1) - windowSize/2, windowSize, windowSize);
    //this->patch.copyTo(img(roi));
    if (isInHi) cv::circle(img, cv::Point(z(0),z(1)), 3, CV_RGB(0,255,0) ,2);
    else if (isInLi) cv::circle(img, cv::Point(z(0),z(1)), 3, CV_RGB(0,0,255) ,2);
    else if (isInInnovation) cv::circle(img, cv::Point(z(0),z(1)), 3, CV_RGB(255,0,0) ,2);

   // char text[10];

   // sprintf(text,"%d",index);
   // cv::putText(img, text, cv::Point(z(0) - windowSize/2, z(1) - windowSize/2), cv::FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(255,0,0));
}

bool Patch::findMatch(cv::Mat frame, MatrixXf covarianceMatrix, float sigma_size, bool showimg_flag) {

	static int imgn = 0;
	n_tot++;
    int windowSize = matching_patch.cols;
    
    int uc = h(0);
    int vc = h(1);
    MatrixXf invSMatrix = covarianceMatrix.inverse();
	cv::Mat newPatch;
	cv::Mat newPatch_no_blur;

    
    float x_2_coeff = invSMatrix(0,0);
    float y_2_coeff = invSMatrix(1,1);
    float yx_coeff = 2*invSMatrix(1,0);

    float max = -1;
    
    float sigma_2 = sigma_size*sigma_size;
    
    float delta_u = sigma_size*sqrt((double)covarianceMatrix(0,0));
    float delta_v = sigma_size*sqrt((double)covarianceMatrix(1,1));

	// not more than 20*20 pixel even if covariance bigger
    if (delta_u > 20) delta_u = 20;
    if (delta_v > 20) delta_v = 20;

    float newmax;
    for (int i = uc - delta_u; i <= uc + delta_u; i++) {
        for (int j = vc - delta_v; j <= vc + delta_v; j++) {
            if (i > windowSize/2 && j > windowSize/2 && i < frame.size().width - windowSize/2 && j < frame.size().height - windowSize/2) {
                if (x_2_coeff*(i - uc)*(i - uc) + y_2_coeff*(j - vc)*(j - vc) + yx_coeff*(i - uc)*(j - vc) <= sigma_2) {
                    cv::Mat subImg = cv::Mat(frame, cv::Rect( i - windowSize/2, j-windowSize/2, windowSize,windowSize));
                    newmax = computeCorrelation(this->matching_patch, subImg);
                  //  new_max_no_blurr = computeCorrelation(this->patch, subImg);

                    if ( newmax > max) {
                        this->center = cv::Point2f(i,j);
                        max = newmax;
                        newPatch = subImg.clone();
                    }
                }
            }
        }
    }
    
    this->matched_patch_blur = newPatch;
	// TODO : is the patch should update to latest frame to better matching
    if (showimg_flag) {
		cv::Mat img;
		hconcat(this->patch, this->matching_patch, img);
		hconcat(img, this->matched_patch_blur, img);
		cv::imshow("Blurring", img);
		char name[100];
		sprintf(name, "blur%03d.jpg",imgn++);
		cv::imwrite(name, img);
		cv::waitKey(1);
    }




    if (max < patch_matching_threshold) {
        this->center = cv::Point2f(-1,-1);
        this->founded = false;
        this->setIsInInnovation(false);
        return false;
    } else {
		this->founded = true;
	//    this->patch =  0.95*this->patch.clone() + 0.05*newPatch.clone();
		this->z(0) = this->center.x;
		this->z(1) = this->center.y;
     //   this->setIsInInnovation(true);
		return true;
    }
}

float computeCorrelation(cv::Mat f1, cv::Mat f2) {


	// TODO: search for faster or more adequte functions
	// built-in opencv, ...
	int step = 1;

    double m1 = 0, m2 = 0, n1 = 0, n2 = 0;
    double corr = 0;

    int n = f1.rows*f1.cols;

    for (int i = 0; i < f1.rows; i+=step) {
        for (int j = 0; j < f1.cols; j+=step) {
            m1 += f1.at<uchar>(i,j);
            m2 += f2.at<uchar>(i,j);
        }
    }

    m1 = m1/n;
    m2 = m2/n;

    float s1 = 0, s2 = 0;
    for (int i = 0; i < f1.rows; i+=step) {
        for (int j = 0; j < f1.cols; j+=step) {
            s1 = f1.at<uchar>(i,j);
            n1 += (s1-m1)*(s1-m1);

            s2 = f2.at<uchar>(i,j);
            n2 += (s2-m2)*(s2-m2);

            corr += (s1-m1)*(s2-m2);
        }
    }
    return corr/sqrt(n2*n1);

}

