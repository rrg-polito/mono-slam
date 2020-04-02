#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
/*
C library to perform Input/Output operations
Input and Output operations can also be performed in C++ using 
the C Standard Input and Output Library (cstdio, known as stdio.h in the C language).
This library uses what are called streams to operate with physical devices such as keyboards,
printers, terminals or with any other type of files supported by the system.
Streams are an abstraction to interact with these in an uniform way;
All streams have similar properties independently 
of the individual characteristics of the physical media
they are associated with.

    scanf
*/
#include <stdlib.h>
/*
C Standard General Utilities Library
This header defines several general purpose functions,
 including dynamic memory management, random number generation,
 communication with the environment, integer arithmetics,
  searching, sorting and converting.
*/
#include <eigen3/Eigen/Dense>


using namespace Eigen;

void plotFeatures(cv::Mat img, std::vector<cv::Point2f> features);

MatrixXf vConcat(MatrixXf m1, MatrixXf m2);
MatrixXf hConcat(MatrixXf m1, MatrixXf m2);
VectorXf Concat(VectorXf m1, VectorXf m2);




