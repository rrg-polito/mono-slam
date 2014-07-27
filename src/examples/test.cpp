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
    std::cout << "Starting monoslam_exec" << std::endl;

    std::ofstream ofile;
    ofile.open("/Volumes/HD2/out.txt");

    VSlamFilter slam("/Volumes/HD2/orientoma/img/conf.cfg");

    std::ifstream fimgs;
    fimgs.open("/Volumes/HD2/orientoma/img/data/d1/t.tex");


    cv::Mat frame;
    for (int i = 0 ;; i++) {
        std::string imgpath;

        fimgs >> imgpath;
        if (imgpath == "") {
            break;
        }

        std::stringstream ss(imgpath);
        std::string word;
        while( getline(ss, word, '/') );

        std::stringstream sw2(word);
        double time;
        sw2 >> time;

        cv::Mat frame = cv::imread(imgpath);

        slam.processNewFrame(frame, time);
        slam.drawPrediction();
        ofile << slam.getState().transpose().segment(3, 4) << std::endl;
        std::cout << slam.getState().transpose().segment(3, 4) << std::endl;
    }
    return 0;
}
