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




#include <opencv2/opencv.hpp>
#include <ctime>
#include <vslamRansac.hpp>
#include <fstream>
#include <string>



int main(int argc, char** argv)
{

  if (argc != 3) {
    std::cout << "Usage: " << argv[0] << " config_file.conf output_file.txt" << std::endl;
  }
  std::cout << "Starting monoslam_exec" << std::endl;

  std::ofstream ofile;
  ofile.open(argv[2]);

  VSlamFilter slam(argv[1]);


  cv::VideoCapture cap(0);
  for (int i = 0 ;; i++) {
    cv::Mat frame;
    cap >> frame;


    slam.processNewFrame(frame, time());
    slam.drawPrediction();
    ofile << slam.getState().transpose().segment(3, 4) << std::endl;
    std::cout << slam.getState().transpose().segment(3, 4) << std::endl;
  }
  return 0;
}
