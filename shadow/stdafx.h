#pragma once
// stdafx.h : 자주 사용하지만 자주 변경되지는 않는
// 표준 시스템 포함 파일 및 프로젝트 관련 포함 파일이
// 들어 있는 포함 파일입니다.
//

#pragma once


//
#include "targetver.h"
#include <iostream>
#include <random>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <vector>

#include <atlstr.h>
#include <stdio.h>
#include <tchar.h>
#include <fstream>
#include <algorithm>

//GDAL
#include "GDALImage.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

//DBSCAN
//#include "dbscan.h"


// OpenCV Header File
#include "core.hpp"
#include "calib3d.hpp"
#include "features2d.hpp"
#include "features2d/features2d.hpp"
#include "highgui.hpp"
#include "imgproc.hpp"
#include "opencv.hpp"
#include "opencv_modules.hpp"
#include "world.hpp"
#include "xfeatures2d.hpp"
#include "ximgproc.hpp"
#include "xfeatures2d/nonfree.hpp"

#include "core/core_c.h"
#include "calib3d/calib3d_c.h"
#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"

#include "core/cuda.hpp"
#include "xfeatures2d/cuda.hpp"
#include "cudaarithm.hpp"
#include "cudafeatures2d.hpp"
#include "cudafilters.hpp"
#include "cudaimgproc.hpp"
#include "cudaobjdetect.hpp"
#include "cudaoptflow.hpp"
#include "cudastereo.hpp"
#include "cudawarping.hpp"

#include <omp.h>

using namespace std;
using namespace cv;




#define Setzero(x) {x=0;}


#define DECTOR_FAST "FAST"
#define DECTOR_SIFT "SIFT"
#define DECTOR_SURF "SURF"
#define DECTOR_ORB "ORB"
#define DECTOR_BRISK "BRISK"
#define DECTOR_MSER "MSER"
#define DECTOR_GFTT "GFTT"
#define DECTOR_HARRIS "HARRIS"
#define DECTOR_DENSE "Dense"
#define DECTOR_SIMPLEBLOB "SimpleBlob"
#define DECTOR_GRIDFAST "GridFAST"

#define DESCRIPTOR_SIFT "SIFT"
#define DESCRIPTOR_SURF "SURF"
#define DESCRIPTOR_BRIEF "BRIEF"
#define DESCRIPTOR_BRISK "BRISK"
#define DESCRIPTOR_ORB "ORB"
#define DESCRIPTOR_FREAK "FREAK"

#define MATCHER_BRUTEFORCE "BruteForce"
#define MATCHER_BRUTEFORCE_L1 "BruteForce-L1"
#define MATCHER_BRUTEFORCE_SL2 "BruteForce-SL2"
#define MATCHER_BRUTEFORCE_HAMMING "BruteForce-Hamming"
#define MATCHER_BRUTEFORCE_HAMMING_2 "BruteForce-Hamming(2)"
#define MATCHER_FlannBased "FlannBased"