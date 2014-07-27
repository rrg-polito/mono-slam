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



#include "utils.hpp"



void plotFeatures(cv::Mat img, std::vector<cv::Point2f> features) {
    int fontFace = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontScale = 0.5;
    int thickness = 3;
    char buffer [33];

    for (int i = 0; i < features.size(); i++ ) {
        sprintf(buffer,"%d",i);
        std::string s(buffer);
        cv::circle(img, features[i], 10, CV_RGB(255, 32, 32));
        cv::putText(img, s, features[i], fontFace, fontScale, CV_RGB(32,255, 32), thickness, 8);
    }
}

MatrixXf vConcat(MatrixXf m1, MatrixXf m2) {
    if (m1.rows() == 0) return m2;
    else {
        MatrixXf concatMatrix(m1.rows()+m2.rows(), m1.cols());
        concatMatrix << m1,m2;
        return concatMatrix;
    }

}

VectorXf Concat(VectorXf m1, VectorXf  m2) {
    if (m1.rows() == 0) return m2;
    else {
        VectorXf concatMatrix(m1.rows()+m2.rows());
        concatMatrix << m1,m2;
        return concatMatrix;
    }

}

MatrixXf hConcat(MatrixXf  m1,  MatrixXf m2) {
    if (m1.cols() == 0) return m2;
    else {
        MatrixXf concatMatrix(m1.rows(), m1.cols() + m2.cols());
        concatMatrix << m1,m2;
        return concatMatrix;
    }

}
