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
        Giuseppe Airo' Farulla (giuseppe.airofarulla@polito.it)
*/




#ifndef LIBBLUR_H_
#define LIBBLUR_H_


#include <opencv2/opencv.hpp>

// This function is to evaluate the kernel
// (the smallest one possible inferable from EKF)
// knowing initial and final point after camera shake

cv::Mat evaluateKernel(cv::Point2f one, cv::Point2f two);




// Input: patch non blurrata + parametri del kernel (parametri predetti da kalman)
// Output: patch blurrata

// This function is to blur the patch with the predicted kernel
// (the smallest one possible inferable from EKF)

cv::Mat blurPatch(const cv::Mat &patch, cv::Point2f one, cv::Point2f two);





// Input: nuova patch blurrata + parametri del kernel (parametri stimati)
// Output: patch deblurrata


// This function is to deblur the patch with the estimated kernel
// LR deconvolution is used in the spatial domain
// (for a quick reference see http://en.wikipedia.org/wiki/RichardsonñLucy_deconvolution )


// Input kernel is supposed to be normalised (sum of its element equal to 1)


// If the deblurring process is not giving high quality outcomes, try to
// augment the number of iterations or to pre-process the kernel taping its edges
// (using http://read.pudn.com/downloads112/sourcecode/graph/texture_mapping/464953/images/edgetaper.m__.htm )


cv::Mat deblurPatch(const cv::Mat &patch, cv::Point2f one, cv::Point2f two);

#endif /* LIBBLUR_H_ */
