#include "stdafx.h"
#include <fstream>
#include <string>
#include "GDALImage.h"

//null값으로 둘 더미값 지정
enum constant { NA = 999 };



void image_normalization(Mat &src, double new_max, double new_min) {

	Mat temp(src.rows, src.cols, CV_8UC1);//&src.type);\

	double min = 16384, max = 0;

	for (int r = 0; r < src.rows; r++) {
		for (int c = 0; c < src.cols; c++) {
			double value = (double)src.at<float>(r, c);
			if (value < 0) src.at<float>(r, c) = 0;
			else if (value == NA) {}
			else {
				if (min > value) min = value;
				if (max < value && max <= 16384) max = value;
				if (value > 16384) src.at<float>(r, c) = 16384;
			}
		}
	}


	for (int r = 0; r < src.rows; r++) {
		for (int c = 0; c < src.cols; c++) {
			double value = (double)src.at<float>(r, c);
			if (value == NA) {
				temp.at<uchar>(r, c) = 255;
			}
			else {
				temp.at<uchar>(r, c) = (value - min)*(new_max - new_min) / (max - min) + new_min;
			}
		}
	}

	src = temp;
}
Mat image_threshold(Mat input_img, Mat &output_img) {
	double th = 0, th1 = 0;

	int h, w;
	h = input_img.rows;
	w = input_img.cols;
	double out = h * w*0.05;

	//Mat output_img = Mat::zeros(h, w, CV_8UC3);

	//normalize(input_img, norm,   0.000, 99.999, NORM_MINMAX);
	image_normalization(input_img, 999.99, 0.00);
	Mat norm = input_img;
	int hist_value[100000];
	int imax = 99999;
	int imin = 0;

	//cout << max << endl << min;
	for (int i = 0; i <= imax; i++) {
		hist_value[i] = 0;
	}

	////histogram 만들기
	int pixel = 0;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			pixel = round(norm.at<float>(r, c) * 100); //e+6
			hist_value[pixel] += 1;
		}
	}

	//outlier 제거
	//int	i = imax;
	//while (hist_value[i] < out) {
	//	hist_value[i - 1] = hist_value[i - 1] + hist_value[i];
	//	hist_value[i] = 0;
	//	i--;
	//}


	int t = 0;
	double tmax = -1000000000000000;

	while (t <= imax) {
		double wb = 0, mb = 0, vb = 0, wf = 0, mf = 0, vf = 0, sumb = 0, sumf = 0, bv = 0, hg = 0;
		for (int b = imin; b < t; b++) {
			sumb += hist_value[b];
			mb += b * hist_value[b];
		}
		for (int f = t; f < imax; f++) {
			sumf += hist_value[f];
			mf += f * hist_value[f];
		}
		mb = mb / sumb;
		wb = sumb / (h*w);
		mf = mf / sumf;
		wf = sumf / (h*w);

		for (int v = 0; v < imax; v++) {
			vb = hist_value[v] * (v - mb) *(v - mb);
		}
		for (int v = 0; v < imax; v++) {
			vb = hist_value[v] * (v - mb) *(v - mb);
		}
		vb = vb / sumb;
		vf = vf / sumf;

		//otsu method
		//bv = (wb*mb*mb + wf * mf*mf);

		//valley emphasis method임
		hg = hist_value[t] / (sumb + sumf);
		bv = (1 - hg)*(wb*mb*mb + wf * mf*mf);

		if (tmax <= bv) {
			tmax = bv;
			th = t;
		}
		t++;
	}
	delete hist_value;

	th = th / 100.000;
	float pixel_real = 0;
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			pixel_real = norm.at<double>(r, c);
			if (th > pixel_real) {
				output_img.at<uchar>(r, c) = 0;
			}
			else {
				output_img.at<uchar>(r, c) = 255;
			}
		}
	}

	return output_img;
}
Mat binarization(Mat input_img, double th) {

	int Ysize = input_img.rows;
	int Xsize = input_img.cols;

	Mat output_img = Mat::zeros(Ysize, Xsize, CV_8U);

	int pixel_real;
	for (int r = 0; r < Ysize; r++) {
		for (int c = 0; c < Xsize; c++) {
			pixel_real = input_img.at<double>(r, c);
			if (th > pixel_real) {
				output_img.at<uchar>(r, c) = 0;
			}
			else {
				output_img.at<uchar>(r, c) = 255;
			}
		}
	}
	return output_img;
}
Mat Reversing(Mat input_img) {

	int Ysize = input_img.rows;
	int Xsize = input_img.cols;

	Mat output_img = Mat::zeros(Ysize, Xsize, CV_8U);

	int pixel_real;
	for (int r = 0; r < Ysize; r++) {
		for (int c = 0; c < Xsize; c++) {
			pixel_real = input_img.at<uchar>(r, c);
			if (pixel_real == 255) {
				output_img.at<uchar>(r, c) = 0;
			}
			else {
				output_img.at<uchar>(r, c) = 1;
			}
		}
	}
	return output_img;
}
int main() {

	//Mat imageR = imread("C:\\Users\\PureumKim\\source\\repos\\shadow_building\\data\\RE3_1B_B_clip1.tif");
	//Mat imageG= imread();
	//Mat imageB= imread();
	//Mat imageN= imread();



	Mat image[4];
	//Mat aaaa = imread("D:\\image_src\\PlanetScope\\개성\\files\\20211023_012710_36_241b_3B_udm2_clip.tif");
	char* filePath[4];
	//*filePath = "C:\\Users\\PureumKim\\Desktop\\ssss.tif";
	*filePath = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_B.tif";
	*(filePath + 1) = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_G.tif";
	*(filePath + 2) = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_R.tif";
	*(filePath + 3) = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_N.tif";
	//*filePath = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_B.tif";
	//*(filePath + 1) = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_G.tif";
	//*(filePath + 2) = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_R.tif";
	//*(filePath + 3) = "D:\\image_src\\CAS500\\C1_20211212020142_04027_00651519_L2G\\C1_20211212020142_04027_00651519_L2G_N.tif";


	cout << filePath << endl << filePath + 1 << endl << filePath[0];
	//int sizeX=8105, sizeY=7595;
	int sizeX, sizeY;
	float* Buffer[4];


	GDALAllRegister();

	GDALDataset* poDataset;
	GDALRasterBand* poRasterBand;
	GDALDriver *poDriverGTIFF;
	const char *pszFormat = "GTIFF";
	poDriverGTIFF = GetGDALDriverManager()->GetDriverByName(pszFormat);
	for (int i = 0; i < 4; i++) {
		poDataset = (GDALDataset*)GDALOpen(filePath[i], GA_ReadOnly); //filled Clip
		if (poDataset == NULL)
		{
			cout << "poDateset is NULL!" << endl;
		}
		sizeX = poDataset->GetRasterXSize();
		sizeY = poDataset->GetRasterYSize();
		Buffer[i] = new float[sizeX * sizeY];
		poRasterBand = poDataset->GetRasterBand(1);
		poRasterBand->RasterIO(GF_Read, 0, 0, sizeX, sizeY, Buffer[i], sizeX, sizeY, GDT_Float32, 0, 0);
		image[i] = Mat(sizeY, sizeX, CV_32FC1, Buffer[i]);
	}

	float three = 0.333333333333333333333;
	float coef[] = { three,three, three,-(sqrt(6) / 6), -(sqrt(6) / 6), (sqrt(6) / 3),(sqrt(6) / 6), -(sqrt(6) / 3), 0 };

	Mat intensity = coef[0] * image[2] + coef[1] * image[1] + coef[2] * image[0];
	Mat v1 = coef[3] * image[2] + coef[4] * image[1] + coef[5] * image[0];
	Mat v2 = coef[6] * image[2] + coef[7] * image[1] + coef[8] * image[0];

	float hue, isi, v11, temp, imsi;
	Mat dst(sizeY, sizeX, CV_32FC1);
	Mat dst1(sizeY, sizeX, CV_32FC1);
	for (int j = 0; j < sizeY; j++) {
		for (int k = 0; k < sizeX; k++) {
			v11 = v1.at<float>(j, k);
			if (v11 != 0) {

				hue = atan2(v2.at<float>(j, k), v11);
				imsi = (intensity.at<float>(j, k) - hue) / (intensity.at<float>(j, k) + hue);
				isi = image[3].at<float>(j, k)*((intensity.at<float>(j, k) - hue) / (intensity.at<float>(j, k) + hue));
				temp = log10(isi);
				imsi = log10(imsi);
			}
			else {
				//NULL 더미값 부여
				temp = NA;
				imsi = NA;
			}
			//if (temp < 0) { 4 
			//	temp = 0;
			//}
			dst.at<float>(j, k) = temp;
			dst1.at<float>(j, k) = imsi;
		}
	}

	Mat equal(sizeY, sizeX, CV_8UC1);
	Mat equal1(sizeY, sizeX, CV_8UC1);

	//8bit 형식 영상으로 으로 정규화
	image_normalization(dst, 255, 0);


	//THRESH_BINARY: 지정한 threshold값으로 이진화
	cout << threshold(dst, equal, 188, 255, THRESH_BINARY) << endl;
	threshold(dst, equal1, 190, 255, THRESH_BINARY);

	//그림자 1 비그림자 0으로 설정
	equal = Reversing(equal);
	equal1 = Reversing(equal1);


	float* colrow = new float[sizeX * sizeY];
	float* colrow1 = new float[sizeX * sizeY];
	for (int r = 0; r < sizeY; r++) {
		for (int c = 0; c < sizeX; c++) {
			*(colrow + (sizeX*r + c)) = equal.at<uchar>(r, c);
			*(colrow1 + (sizeX*r + c)) = equal1.at<uchar>(r, c);
		}
	}


	char pathcolrow[100] = "hb_lsi_188.tif";
	char pathcolrow1[100] = "hb_lsi_190.tif";

	double srtmGeoT[6];
	poDataset->GetGeoTransform(srtmGeoT);

	poDataset = poDriverGTIFF->Create(pathcolrow1, sizeX, sizeY, 1, GDT_Float32, NULL);
	poDataset->SetGeoTransform(srtmGeoT);

	poDataset->SetProjection(poDataset->GetProjectionRef());
	poRasterBand = poDataset->GetRasterBand(1);

	poRasterBand->RasterIO(GF_Write, 0, 0, sizeX, sizeY, colrow1, sizeX, sizeY, GDT_Float32, 0, 0);

	poDataset->GetGeoTransform(srtmGeoT);

	poDataset = poDriverGTIFF->Create(pathcolrow, sizeX, sizeY, 1, GDT_Float32, NULL);
	poDataset->SetGeoTransform(srtmGeoT);

	poDataset->SetProjection(poDataset->GetProjectionRef());
	poRasterBand = poDataset->GetRasterBand(1);

	poRasterBand->RasterIO(GF_Write, 0, 0, sizeX, sizeY, colrow, sizeX, sizeY, GDT_Float32, 0, 0);

	return 0;
}