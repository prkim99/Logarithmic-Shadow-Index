// GDALImage Class Ver 0.9
// Made by Ahmkun  2008. 12. 12
// 
// GDAL 을 이용한 영상 입출력
//
// GDAL Settings
// Preprocessor -> ./GDAL15/include
// lib Pass     -> ./GDAL15/lib
// Add lib      -> gdal_i.lib 
//
// 1. 대부분의 영상포맷을 읽을 수 있음
// 2. 영상의 저장은 GTIFF 혹은 BMP 만 가능 (추후 수정 필요)
// 3. Write 함수에 ImageType을 설정 시 "BMP", "GTiff" 로 적어야 함
//
#pragma once

#include <iostream>
#include <cmath>
#include "gdal_priv.h"


using namespace std;

#define BMP 0
#define GTIFF 1

template<class T>
class GDALImage
{
public:
	int nXSize, nYSize;	// 읽은 영상의 Col, Row 크기
	int RasterCount;    // 읽은 영상의 밴드 정보
	
	GDALDataType m_GDALType;
	
	T * _ImageBuffer;
	double * Image_Float64;
	float  * Image_Float32;
	short  * Image_Int16;
	unsigned short * Image_UInt16;
	unsigned char  * Image_BYTE;
		
public:
	GDALImage();

	int ReadImage(char* InputName, int ch_flag);
	int ReadImage1(char* InputName, T* MemBuffer, int ReduceFactor);
	int ReadSubImage(char* InputName, T* MemBuffer,int XSize, int YSize, int ULX, int ULY); // 부분읽기
	int WriteImage(int ImageType, GDALDataType i_DataType, char* OutputName, T * _ImageBuffer);
	int WriteImage1(int ImageType, GDALDataType i_DataType, char* OutputName, T * R, T* G, T* B);
	void GetBuffer(T * _ImageBuffer);  // 읽어온 영상을 메모리에 저장 
	void SetBuffer(T * _ImageBuffer);  // 메모리의 영상을 저장
	void setGDALAllRegister(); // GDALAllRegister();

	int Get_nXSize(); // Col 불러옴
	int Get_nYSize(); // Row 불러옴
	int Set_nXSize(int XSize); // Col 
	int Set_nYSize(int YSize); // Row 
	//** 공간 정보
	//** adfGeoTransform[0]: Top-left pixel's X coordinate
	//** adfGeoTransform[1]: Pixel resolution in column direction
	//** adfGeoTransform[2]: Equal to zero
	//** adfGeoTransform[3]: Top-left pixel's Y coordinate
	//** adfGeoTransform[4]: Equal to zero
	//** adfGeoTransform[5]: Pixel resolution in row direction(negative value)
	double adfGeoTransform[6];  
	double setGeoTransForm(double OriginX=0, double OriginY=0, double Width=0, double Height=0);

	int AdaptiveFilter(int mask, T * buf, int col, int row, double Sigma=42);
	int GaussianFilter(int masksize, T * buf, int col, int row);
	int MedianFilter(int mask, T * buf, int col, int row);
	int OrderStatFilter(int mask, T * buf, int col, int row, int order);

	int Sort(float *Neighbor, int n);
	int Equalization_Selective(unsigned char *Buf, int s_thresh,int e_thresh, int col, int row);
	int Equalization_Selective(unsigned short *Buf, int s_thresh, int e_thresh, int col, int row);
	int RunFiltering(int ROW, int COL, T val, T* chData);
	
	// For MMatching
	int GetPixelCnt(int ROW, int COL,int i, int j, T val, T* chData);
	int GetD_PixelCnt(int ROW, int COL,int i, int j, T val, T* chData);
	int GetR_PixelCnt(int ROW, int COL,int i, int j, T val, T* chData);
	int SMedianFilter(int mask, T * buf, int col, int row);
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<class T>
GDALImage<T>::GDALImage() 
{
	_ImageBuffer = NULL;
	Image_Float64 = NULL;
	Image_Float32 = NULL;
	Image_Int16 = NULL;
	Image_UInt16 = NULL;
	Image_BYTE = NULL;
}

template<class T>
int GDALImage<T>::Get_nXSize(){	return(nXSize); }

template<class T>
int GDALImage<T>::Get_nYSize(){	return(nYSize); }

template<class T>
int GDALImage<T>::Set_nXSize(int XSize){ nXSize = XSize; return 0;}

template<class T>
int GDALImage<T>::Set_nYSize(int YSize){ nYSize = YSize; return 0;}


template<class T>
int GDALImage<T>::ReadImage1(char* InputName, T* MemBuffer, int ReduceFactor)
{
	//Openning the File
	const char *pszFilename = InputName;
	GDALDataset *poDataset;
	GDALAllRegister(); 
	poDataset = (GDALDataset *) GDALOpen( pszFilename, GA_ReadOnly );
	if( poDataset == NULL ) printf( "Couldn't open dataset %s\n", pszFilename );

	//Fetching a Raster Band Color
	GDALRasterBand  *poBand;
	poBand = poDataset->GetRasterBand( 1 );
	poDataset->GetGeoTransform( adfGeoTransform );

	//Reading Raster Data
	nXSize = poDataset->GetRasterXSize();
	nYSize = poDataset->GetRasterYSize();
	m_GDALType = poBand->GetRasterDataType();
	//cout<<"Input Image raster type : "<<GDALGetDataTypeName(poBand->GetRasterDataType()) <<" Type"<<endl; 

	switch(m_GDALType)
	{
		case GDT_Float64:
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, MemBuffer, nXSize/ReduceFactor, nYSize/ReduceFactor, m_GDALType , 0, 0 );
			break;
		case GDT_Float32:
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, MemBuffer, nXSize/ReduceFactor, nYSize/ReduceFactor, m_GDALType , 0, 0 );
			break;
		case GDT_Int16:
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, MemBuffer, nXSize/ReduceFactor, nYSize/ReduceFactor, m_GDALType , 0, 0 );
			break;
		case GDT_UInt16:
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, MemBuffer, nXSize/ReduceFactor, nYSize/ReduceFactor, m_GDALType , 0, 0 );
			break;
		case GDT_Byte:
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, MemBuffer, nXSize/ReduceFactor, nYSize/ReduceFactor, m_GDALType , 0, 0 );
			break;
		default:
		return false;
	}

	GDALClose( (GDALDatasetH) poDataset );
	return 0;
}

//** 2018-10-04, Revised by Wansang Yoon, for multi-channel image processing
template<class T>
int GDALImage<T>::ReadImage(char* InputName, int ch_flag)
{
	//Openning the File
	const char *pszFilename = InputName;
	GDALDataset *poDataset;
	GDALAllRegister(); 

	poDataset = (GDALDataset *) GDALOpen( pszFilename, GA_ReadOnly );
	if( poDataset == NULL ) printf( "Couldn't open dataset %s\n", pszFilename );
	//0928 HyeongJun update
	GDALRasterBand  *poBand;
	//Fetching a Raster Band Color
	if (ch_flag > poDataset->GetRasterCount()) {
		poBand = poDataset->GetRasterBand(1);
	} else {
		poBand = poDataset->GetRasterBand(ch_flag);
	}//end: if ~ else (ch_flag > poDataset->GetRasterCount())

	poDataset->GetGeoTransform(adfGeoTransform);

	//Reading Raster Data
	nXSize = poDataset->GetRasterXSize();
	nYSize = poDataset->GetRasterYSize();
	m_GDALType = poBand->GetRasterDataType();
	//cout<<"Input Image raster type : "<<GDALGetDataTypeName(poBand->GetRasterDataType()) <<" Type"<<endl; 

	_ImageBuffer = new T [nXSize*nYSize];

	switch(m_GDALType)
	{
		case GDT_Float64:
			Image_Float64 = new double [nXSize*nYSize];
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, Image_Float64, nXSize, nYSize, m_GDALType , 0, 0 );
			for(int i=0; i<nXSize*nYSize; i++) _ImageBuffer[i] = (T)Image_Float64[i];
			delete[] Image_Float64;
			Image_Float64 == NULL;
			break;
		case GDT_Float32:
			Image_Float32 = new float [nXSize*nYSize];
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, Image_Float32, nXSize, nYSize, m_GDALType , 0, 0 );
			for(int i=0; i<nXSize*nYSize; i++) _ImageBuffer[i] = (T)Image_Float32[i];
			delete[] Image_Float32;
			Image_Float32 == NULL;
			break;
		case GDT_Int16:
			Image_Int16 = new short [nXSize*nYSize];
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, Image_Int16, nXSize, nYSize, m_GDALType , 0, 0 );
			for(int i=0; i<nXSize*nYSize; i++) _ImageBuffer[i] = (T)Image_Int16[i];
			delete[] Image_Int16;
			Image_Int16 == NULL;
			break;
		case GDT_UInt16:
			Image_UInt16 = new unsigned short [nXSize*nYSize];
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, Image_UInt16, nXSize, nYSize, m_GDALType , 0, 0 );
			for(int i=0; i<nXSize*nYSize; i++) _ImageBuffer[i] = (T)Image_UInt16[i];
			delete[] Image_UInt16;
			Image_UInt16 == NULL;
			break;
		case GDT_Byte:
			Image_BYTE = new unsigned char [nXSize*nYSize];
			poBand->RasterIO( GF_Read, 0, 0, nXSize, nYSize, Image_BYTE, nXSize, nYSize, m_GDALType , 0, 0 );
			for(int i=0; i<nXSize*nYSize; i++) _ImageBuffer[i] = (T)Image_BYTE[i];
			delete[] Image_BYTE;
			Image_BYTE == NULL;
			break;
		default:
		return false;
	}

	GDALClose( (GDALDatasetH) poDataset );
	return 0;
}

template<class T>
int GDALImage<T>::ReadSubImage(char* InputName, T* MemBuffer,int XSize, int YSize, int ULX, int ULY)
{
	//Openning the File
	const char *pszFilename = InputName;
	GDALDataset *poDataset;
	GDALAllRegister(); 
	poDataset = (GDALDataset *) GDALOpen( pszFilename, GA_ReadOnly );
	  
	//Fetching a Raster Band Color
	GDALRasterBand  *poBand;
	poBand = poDataset->GetRasterBand( 1 );

	//Reading Raster Data
	nXSize = XSize;
	nYSize = YSize;
	m_GDALType = poBand->GetRasterDataType();
	

	switch(m_GDALType)
	{
		case GDT_Float64:
			Image_Float64 = new double [nXSize*nYSize];
			poBand->RasterIO( GF_Read, ULX, ULY, XSize, YSize, MemBuffer, XSize, YSize, m_GDALType , 0, 0 );
			break;
		case GDT_Float32:
			Image_Float32 = new float [nXSize*nYSize];
			poBand->RasterIO( GF_Read, ULX, ULY, XSize, YSize, MemBuffer, XSize, YSize, m_GDALType , 0, 0 );
			break;
		case GDT_Int16:
			Image_Int16 = new short [nXSize*nYSize];
			poBand->RasterIO( GF_Read, ULX, ULY, XSize, YSize, MemBuffer, XSize, YSize, m_GDALType , 0, 0 );
			break;
		case GDT_UInt16:
			Image_UInt16 = new unsigned short [nXSize*nYSize];
			poBand->RasterIO( GF_Read, ULX, ULY, XSize, YSize, MemBuffer, XSize, YSize, m_GDALType , 0, 0 );
			break;
		case GDT_Byte:
			Image_BYTE = new unsigned char [nXSize*nYSize];
			poBand->RasterIO( GF_Read, ULX, ULY, XSize, YSize, MemBuffer, XSize, YSize, m_GDALType , 0, 0 );
			break;
		default:
		return false;
	}
	//부분이미지추출결과확인
/*	GDALImage<unsigned char> PP;
	PP.Set_nXSize(XSize);
	PP.Set_nYSize(YSize);
	unsigned char * TT = new unsigned char[XSize*YSize];
	for(int i=0; i<XSize*YSize; i++) TT[i] = (unsigned char)MemBuffer[i];
	PP.WriteImage(BMP,GDT_Byte,"SubImageExtract.bmp",TT);
*/
	GDALClose( (GDALDatasetH) poDataset );
	return 0;
}

template<class T>
void GDALImage<T>::GetBuffer(T * Buffer)
{
	for(int i=0; i<nXSize*nYSize ; i++)
	Buffer[i] = _ImageBuffer[i];		
}

template<class T>
void GDALImage<T>::SetBuffer(T * _ImageBuffer){
	this->_ImageBuffer = new T[nXSize*nYSize];
	for (int i = 0; i < nXSize*nYSize; i++)
		this->_ImageBuffer[i] = _ImageBuffer[i];
}
template<class T>
void GDALImage<T>::setGDALAllRegister(){ GDALAllRegister(); }

template<class T>
double GDALImage<T>::setGeoTransForm(double OriginX, double OriginY, double Width, double Height)
{
	adfGeoTransform[0] = OriginX;
	adfGeoTransform[1] = Width;
	adfGeoTransform[2] = 0;
	adfGeoTransform[3] = OriginY;
	adfGeoTransform[4] = 0;
	adfGeoTransform[5] = Height;
	return 0;
}

template<class T>
int GDALImage<T>::WriteImage(int ImageType, GDALDataType i_DataType, char* OutputName, T * _ImageBuffer)
{
	GDALDriver *poDriver;
	GDALDataset *poDstDS;       
	char **papszMetadata;
	char **papszOptions = NULL;
	const char *pszFormat;
	if(ImageType == BMP )
	{
		pszFormat = "BMP";
		poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

		if( poDriver == NULL )	exit( 1 );

		papszMetadata = poDriver->GetMetadata();
		poDstDS = poDriver->Create( OutputName, nXSize, nYSize, 1, GDT_Byte, papszOptions );

		GDALRasterBand *poBand;
		poBand = poDstDS->GetRasterBand(1);
		poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize, _ImageBuffer, nXSize, nYSize, GDT_Byte , 0, 0 );
	}
	else
	{
		pszFormat = "GTiff";
		poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		if( poDriver == NULL )	exit( 1 );

		papszMetadata = poDriver->GetMetadata();
		GDALRasterBand *poBand;
		
		double adfGTransform[6];
		for(int i=0; i<6 ; i++)	adfGTransform[i] = adfGeoTransform[i];

		switch(i_DataType)
		{
			case GDT_Float64:
				poDstDS = poDriver->Create( OutputName, nXSize, nYSize, 1, GDT_Float64, papszOptions );
				poDstDS->SetGeoTransform( adfGTransform );
				poBand = poDstDS->GetRasterBand(1);
				poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize, _ImageBuffer, nXSize, nYSize, GDT_Float64 , 0, 0 );
				break;
			case GDT_Float32:
				poDstDS = poDriver->Create( OutputName, nXSize, nYSize, 1, GDT_Float32, papszOptions );
				poDstDS->SetGeoTransform( adfGTransform );
				poBand = poDstDS->GetRasterBand(1);
				poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize, _ImageBuffer, nXSize, nYSize, GDT_Float32 , 0, 0 );
				break;
			case GDT_Int16:
				poDstDS = poDriver->Create( OutputName, nXSize, nYSize, 1, GDT_Int16, papszOptions );
				poDstDS->SetGeoTransform( adfGTransform );
				poBand = poDstDS->GetRasterBand(1);
				poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize, _ImageBuffer, nXSize, nYSize, GDT_Int16 , 0, 0 );
				break;
			case GDT_UInt16:
				poDstDS = poDriver->Create( OutputName, nXSize, nYSize, 1, GDT_UInt16, papszOptions );
				poDstDS->SetGeoTransform( adfGTransform );
				poBand = poDstDS->GetRasterBand(1);
				poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize, _ImageBuffer, nXSize, nYSize, GDT_UInt16 , 0, 0 );
				break;
			case GDT_Byte:
				poDstDS = poDriver->Create( OutputName, nXSize, nYSize, 1, GDT_Byte, papszOptions );
				poDstDS->SetGeoTransform( adfGTransform );
				poBand = poDstDS->GetRasterBand(1);
				poBand->RasterIO( GF_Write, 0, 0, nXSize, nYSize, _ImageBuffer, nXSize, nYSize, GDT_Byte , 0, 0 );
				break;
			default:
			return false;
		}
	}

	GDALClose( (GDALDatasetH) poDstDS );
	
	return 0 ;
}
template<class T>
int GDALImage<T>::WriteImage1(int ImageType, GDALDataType i_DataType, char* OutputName, T * R, T* G, T* B)
{
	GDALDriver *poDriver;
	GDALDataset *poDstDS;
	char **papszMetadata;
	char **papszOptions = NULL;
	const char *pszFormat;
	if (ImageType == BMP)
	{
		pszFormat = "BMP";
		poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		if (poDriver == NULL)	exit(1);

		papszMetadata = poDriver->GetMetadata();
		poDstDS = poDriver->Create(OutputName, nXSize, nYSize, 1, GDT_Byte, papszOptions);

		GDALRasterBand *poBand;
		poBand = poDstDS->GetRasterBand(1);
		poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, R, nXSize, nYSize, GDT_Byte, 0, 0);
	}
	else
	{
		pszFormat = "GTiff";
		poDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);
		if (poDriver == NULL)	exit(1);

		papszMetadata = poDriver->GetMetadata();

		GDALRasterBand *poBand;

		double adfGTransform[6];
		for (int i = 0; i<6; i++)	adfGTransform[i] = adfGeoTransform[i];

		switch (i_DataType)
		{
		case GDT_Float64:
			poDstDS = poDriver->Create(OutputName, nXSize, nYSize, 1, GDT_Float64, papszOptions);
			poDstDS->SetGeoTransform(adfGTransform);
			poBand = poDstDS->GetRasterBand(1);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, R, nXSize, nYSize, GDT_Float64, 0, 0);
			break;
		case GDT_Float32:
			poDstDS = poDriver->Create(OutputName, nXSize, nYSize, 1, GDT_Float32, papszOptions);
			poDstDS->SetGeoTransform(adfGTransform);
			poBand = poDstDS->GetRasterBand(1);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, R, nXSize, nYSize, GDT_Float32, 0, 0);
			break;
		case GDT_Int16:
			poDstDS = poDriver->Create(OutputName, nXSize, nYSize, 1, GDT_Int16, papszOptions);
			poDstDS->SetGeoTransform(adfGTransform);
			poBand = poDstDS->GetRasterBand(1);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, R, nXSize, nYSize, GDT_Int16, 0, 0);
			break;
		case GDT_UInt16:
			poDstDS = poDriver->Create(OutputName, nXSize, nYSize, 1, GDT_UInt16, papszOptions);
			poDstDS->SetGeoTransform(adfGTransform);
			poBand = poDstDS->GetRasterBand(1);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, R, nXSize, nYSize, GDT_UInt16, 0, 0);
			poBand = poDstDS->GetRasterBand(2);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, G, nXSize, nYSize, GDT_UInt16, 0, 0);
			poBand = poDstDS->GetRasterBand(3);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, B, nXSize, nYSize, GDT_UInt16, 0, 0);
			break;
		case GDT_Byte:
			poDstDS = poDriver->Create(OutputName, nXSize, nYSize, 3, GDT_Byte, papszOptions);
			poDstDS->SetGeoTransform(adfGTransform);
			poBand = poDstDS->GetRasterBand(1);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, R, nXSize, nYSize, GDT_Byte, 0, 0);
			poBand = poDstDS->GetRasterBand(2);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, G, nXSize, nYSize, GDT_Byte, 0, 0);
			poBand = poDstDS->GetRasterBand(3);
			poBand->RasterIO(GF_Write, 0, 0, nXSize, nYSize, B, nXSize, nYSize, GDT_Byte, 0, 0);
			break;
		default:
			return false;
		}
	}

	GDALClose((GDALDatasetH)poDstDS);

	return 0;
}
template<class T>
int GDALImage<T>::GaussianFilter(int masksize, T * buf, int col, int row)
{
	int i,j,k,l;

	double sigma = masksize/(2.5) ; //시그마 처리

	int length = (int)masksize;
	int center = (int)(length/2+0.5);
	double val;

	int m_height, m_width;
	m_height = m_width = masksize;
	double * mask = new double[m_height*m_width];
	double PI = 3.1415926535;

	double  summask = 0;
	
	// Calculating Mask Value
	for(i=0 ; i<m_height ; i++)
	{
		for(j=0 ; j<m_width ; j++)
		{
			mask[i*m_width+j] = (1.0/(2.0*PI*sigma*sigma)) 
				* exp(-((i-center)*(i-center) + (j-center)*(j-center))*(1.0/
				(2.0*sigma*sigma)));
			
			summask += mask[i*m_width+j];
		}
	}
	
	// Normalizing Mask Value
	for(i=0 ; i<m_height*m_width ; i++)	
		mask[i] = mask[i]/summask;

	T * n_buf = new T [row*col];
	
	// filtering
	for(k=center ; k<row-center ; k++)
	{
		for(l=center ; l<col-center ; l++)
		{
			val = 0;
			for(i=0 ; i<m_height ; i++)
			{
				for(j=0 ; j<m_width ; j++)
				{
					val = val + (mask[i*m_width+j] * buf[(k-center+i)*col+(l-center+j)]);
				}
			}
			if(val>255) n_buf[k*col+l] = 255;
			if(val<0) n_buf[k*col+l] = 0;
			else n_buf[k*col+l] = (T)val;
		//	n_buf[k*col+l] = (T)val;
		}
	}

	for(k=0 ; k<row ; k++)
	{
		for(l=0 ; l<col ; l++)
		{
			if(k<center || l<center || k>=row-center || l>=col-center) 
				n_buf[k*col+l] = buf[k*col+l];			
		}
	}

	//Copy 
	for(i=0 ; i<row*col ; i++)	
		buf[i] = n_buf[i];

	delete [] n_buf;
	delete [] mask;
	
	return 0;
}

template<class T>
int GDALImage<T>::MedianFilter(int mask, T * buf, int col, int row)
{
	int i, j, k,l ,a,b;
	int masksize = mask*mask;
	int maskflag = mask/2;
	int median = masksize/2;
	float *Neighbor = new float[masksize];
	T *temp_orientation = new T[col*row];
	for(i=0; i<col*row ; i++) 
	{
		temp_orientation[i] = buf[i];
	}
	
	for(j=maskflag; j<row-maskflag; j++)
	{
		for(i=maskflag; i<col-maskflag; i++)
		{
			for(a=0, k=j-maskflag ; a<mask ; k++, a++)
			{
				for(b=0, l=i-maskflag ; b<mask; l++, b++)
				{
					Neighbor[a*mask+b] = buf[k*col+l];
				}
			}
			Sort(Neighbor, masksize);
			temp_orientation[j*col+i]=(T)Neighbor[median];
		}
	}

	for(i=0; i<col*row ; i++) 
	{
		buf[i] = temp_orientation[i];
	}
	delete [] temp_orientation;
	delete [] Neighbor;
	return 0 ;
}

template<class T>
int GDALImage<T>::OrderStatFilter(int mask, T * buf, int col, int row, int order)
{
	int i, j, k,l ,a,b;
	int masksize = mask*mask;
	int maskflag = mask/2;
	int median = masksize/2;
	float *Neighbor = new float[masksize];
	T *temp_orientation = new T[col*row];
	for(i=0; i<col*row ; i++) 
	{
		temp_orientation[i] = buf[i];
	}
	
	for(j=maskflag; j<row-maskflag; j++)
	{
		for(i=maskflag; i<col-maskflag; i++)
		{
			for(a=0, k=j-maskflag ; a<mask ; k++, a++)
			{
				for(b=0, l=i-maskflag ; b<mask; l++, b++)
				{
					Neighbor[a*mask+b] = buf[k*col+l];
				}
			}
			Sort(Neighbor, masksize);
			temp_orientation[j*col+i]=(T)Neighbor[order];
		}
	}

	for(i=0; i<col*row ; i++) 
	{
		buf[i] = temp_orientation[i];
	}
	delete [] temp_orientation;
	delete [] Neighbor;
	return 0 ;
}

// 값이 없을때는 처리하지 않는 필터
template<class T>
int GDALImage<T>::SMedianFilter(int mask, T * buf, int col, int row)
{
	int i, j, k,l ,a,b;
	int masksize = mask*mask;
	int maskflag = mask/2;
	int median = masksize/2;
	float *Neighbor = new float[masksize];
	T *temp_orientation = new T[col*row];
	for(i=0; i<col*row ; i++) 
	{
		temp_orientation[i] = buf[i];
	}

	double MinValue = 10000;
	
	for(j=maskflag; j<row-maskflag; j++)
	{
		for(i=maskflag; i<col-maskflag; i++)
		{
			if( buf[j*col+i] !=0 && buf[j*col+i] != -1)
			{
				MinValue = 100000;
				for(a=0, k=j-maskflag ; a<mask ; k++, a++)
				{
					for(b=0, l=i-maskflag ; b<mask; l++, b++)
					{
						if( buf[k*col+l] !=0 && buf[k*col+l] != -1)
						{
							if(MinValue>buf[k*col+l])
							{
								MinValue = buf[k*col+l];
							}
						}
					}
				}
				if(MinValue != 100000)
				{							
					for(a=0, k=j-maskflag ; a<mask ; k++, a++)
					{
						for(b=0, l=i-maskflag ; b<mask; l++, b++)
						{
							if( buf[k*col+l] !=0 && buf[k*col+l] != -1)
							{
								Neighbor[a*mask+b] = buf[k*col+l];
							}
							else Neighbor[a*mask+b] = MinValue;
						}
					}
					Sort(Neighbor, masksize);
					temp_orientation[j*col+i]=(T)Neighbor[median];
				}
				else temp_orientation[j*col+i] = buf[j*col+i];
			}
			else temp_orientation[j*col+i]=-1;
		}
	}

	for(i=0; i<col*row ; i++) 
	{
		buf[i] = temp_orientation[i];
	}
	delete [] temp_orientation;
	delete [] Neighbor;
	return 0 ;
}


template<class T>
int GDALImage<T>::Sort(float *Neighbor, int n)
{
	int i, j;
	float next;
	for(i=1; i<n; i++)
	{
		next=Neighbor[i];
		for(j=i-1; j>=0 && next<Neighbor[j]; j--)
			Neighbor[j+1]=Neighbor[j];
		Neighbor[j+1]=next;
	}
	return 0 ;
}


template<class T>
int GDALImage<T>::Equalization_Selective(unsigned char *Buf, int s_thresh, int e_thresh, int col, int row)
{
	int i,j;
	unsigned int *I_Hist = new unsigned int[256];
	unsigned int *Sum_Hist = new unsigned int[256];

	double _I;

	int _row = row;
	int _col = col;

	unsigned char *I_change = new unsigned char[_row*_col];
	unsigned char *I_array = new unsigned char[_row*_col];

	for(i = 0 ; i<256 ; i++)	
	{
		I_Hist[i] =0;
	}// 초기화

	for(int j = 0; j < _row; j++)
	{
		for(int k = 0; k < _col; k++)
		{
			_I = Buf[(j*_col+k)];
			I_array[j*_col+k] = (unsigned char)_I;
			I_Hist[(int)_I]++;
		}
	}

	//히스토그램 정규화 합 계산
	int sum = 0;
	float min = 255*(float)s_thresh/100;
	float max = 255*(100-(float)e_thresh)/100;
	float scale_factor = (max-min)/(float)(_col*_row);
	for(i=0 ; i<256; i++)
	{
		if(i>=min && i<=max)
		{
			sum += I_Hist[i];
			Sum_Hist[i] = (int)((sum*scale_factor) + 0.5);
		}
		else if(i<min)
		{
			Sum_Hist[i] = (unsigned int)min;//(int)((sum*scale_factor) + 0.5);
		}
		else if(i>max)
		{
			Sum_Hist[i] = (unsigned int)max;//(int)((sum*scale_factor) + 0.5);
		}
	//	cout<<Sum_Hist[i]<<endl;
	}

	// 
	for(i = 0; i < _row; i++)
	{
		for(int j = 0; j < _col; j++)
		{
			I_change[i*_col+j] = Sum_Hist[I_array[i*_col+j]];
		}
	}

	for( j = 0; j < _row; j++)
	{
		for(int k = 0; k < _col; k++)
		{
			_I = I_change[j*_col+k];
			if(_I<0) Buf[(j*_col+k)] = 0;
			else if(_I>255 ) Buf[(j*_col+k)] = 255;
			else Buf[(j*_col+k)] = (unsigned char)_I;
		}
	}
	delete [] I_Hist;
	delete [] Sum_Hist;
	delete [] I_change;
	delete [] I_array;
	return 0 ;
}

template<class T>
int GDALImage<T>::Equalization_Selective(unsigned short *Buf, int s_thresh, int e_thresh, int col, int row)
{
	int i,j;
	unsigned int *I_Hist = new unsigned int[2048];
	unsigned int *Sum_Hist = new unsigned int[2048];

	double _I;

	int _row = row;
	int _col = col;

	unsigned short *I_change = new unsigned short[_row*_col];
	unsigned short *I_array = new unsigned short[_row*_col];

	for(i = 0 ; i<2048 ; i++)	
	{
		I_Hist[i] =0;
	}// 초기화

	for(int j = 0; j < _row; j++)
	{
		for(int k = 0; k < _col; k++)
		{
			_I = Buf[(j*_col+k)];
			I_array[j*_col+k] = (unsigned short)_I;
			I_Hist[(int)_I]++;
		}
	}

	//히스토그램 정규화 합 계산
	int sum = 0;
	float min = 2047*(float)s_thresh/100;
	float max = 2047*(100-(float)e_thresh)/100;
	float scale_factor = (max-min)/(float)(_col*_row);
	for(i=0 ; i<2048; i++)
	{
		if(i>=min && i<=max)
		{
			sum += I_Hist[i];
			Sum_Hist[i] = (int)((sum*scale_factor) + 0.5);
		}
		else if(i<min)
		{
			Sum_Hist[i] = (unsigned int)min;//(int)((sum*scale_factor) + 0.5);
		}
		else if(i>max)
		{
			Sum_Hist[i] = (unsigned int)max;//(int)((sum*scale_factor) + 0.5);
		}
	//	cout<<Sum_Hist[i]<<endl;
	}

	// 
	for(i = 0; i < _row; i++)
	{
		for(int j = 0; j < _col; j++)
		{
			I_change[i*_col+j] = Sum_Hist[I_array[i*_col+j]];
		}
	}

	for( j = 0; j < _row; j++)
	{
		for(int k = 0; k < _col; k++)
		{
			_I = I_change[j*_col+k];
			if(_I<0) Buf[(j*_col+k)] = 0;
			else if(_I>2047 ) Buf[(j*_col+k)] = 2047;
			else Buf[(j*_col+k)] = (unsigned short)_I;
		}
	}
	delete [] I_Hist;
	delete [] Sum_Hist;
	delete [] I_change;
	delete [] I_array;
	return 0 ;
}


template<class T>
int GDALImage<T>::AdaptiveFilter(int mask, T * buf, int col, int row, double Sigma)
{
	int i, j, k,l ,a,b;
	int masksize = mask*mask;
	int maskflag = mask/2;
	int median = masksize/2;
	
	T *temp = new T[col*row];
	for(i=0; i<col*row ; i++) 
	{
		temp[i] = 0;
	}

	double Gx, Gy, Weight;
	double UP = 0;
	double DN = 0;
	
	for(j=maskflag+1; j<row-maskflag-1; j++)
	{
		for(i=maskflag+1; i<col-maskflag-1; i++)
		{
			UP = DN = 0;
			for(a=0, k=j-maskflag ; a<mask ; k++, a++)
			{
				for(b=0, l=i-maskflag ; b<mask; l++, b++)
				{
					Gx =  (buf[k*col+(l+1)]-buf[k*col+(l-1)])/2;
					Gy =  (buf[(k+1)*col+l]-buf[(k-1)*col+l])/2;
					Weight = pow(2.718281828, (double)( (-1)*( Gx*Gx + Gy*Gy) / (2*Sigma*Sigma) ) );	
					UP += (buf[k*col+l]*Weight);
					DN += Weight;
				}
			}
			temp[j*col+i] = (T)(UP/DN);	
		}
	}
	for(i=0; i<col*row ; i++) 
	{
		if(temp[i]<0) buf[(j*col+k)] = 0;
		else if(temp[i]>=2047 ) buf[(j*col+k)] = 2047;
		else buf[(j*col+k)] = (unsigned short)temp[i];
	}
	delete [] temp;
	return 0 ;
}

template<class T>
int GDALImage<T>:: RunFiltering(int ROW, int COL, T val, T* chData)
{
	T * chCopyData = new T [ROW*COL];
	for(int i=0 ; i<ROW*COL ; i++) chCopyData[i] = chData[i];

	for(int i=1; i<ROW-1; i++)
	{
		for(int j=1; j<COL-1; j++)
		{
			if(chCopyData[(i-1)*COL+j] == val && GetD_PixelCnt(ROW,COL,i,j,val,chCopyData)>0)
				chData[i*COL+j] = val;
			if(chCopyData[(i)*COL+j-1] == val && GetR_PixelCnt(ROW,COL,i,j,val,chCopyData)>0)
				chData[i*COL+j] = val;
			
			if(chCopyData[(i-1)*COL+(j-1)] == val && chCopyData[(i+1)*COL+(j+1)] == val)
				chData[i*COL+j] = val;
			if(chCopyData[(i+1)*COL+(j-1)] == val && chCopyData[(i-1)*COL+(j+1)] == val)
				chData[i*COL+j] = val;
		}
	}
	
	for(int i=0 ; i<ROW*COL ; i++) chCopyData[i] = chData[i];
	for(int i=1; i<ROW-1; i++){
		for(int j=1; j<COL-1; j++){
			if(chCopyData[i*COL+j]==val)
			{
				if(GetPixelCnt(ROW,COL,i,j,val,chCopyData) < 2)
					chData[i*COL+j] = 0;
			}
		}
	}

	delete [] chCopyData;

	return 1;
}

template<class T>
int GDALImage<T>:: GetPixelCnt(int ROW, int COL,int i, int j, T val, T* chData)
{
	int nCnt = 0;
	if(chData[(i-1)*COL+(j-1)] == val) nCnt++;
	if(chData[(i-1)*COL+(j)] == val) nCnt++;
	if(chData[(i-1)*COL+(j+1)] == val) nCnt++;
	if(chData[(i)*COL+(j-1)] == val) nCnt++;
	if(chData[(i)*COL+(j)] == val) nCnt++;
	if(chData[(i)*COL+(j+1)] == val) nCnt++;
	if(chData[(i+1)*COL+(j-1)] == val) nCnt++;
	if(chData[(i+1)*COL+(j)] == val) nCnt++;
	if(chData[(i+1)*COL+(j+1)] == val) nCnt++;
	return nCnt;
}

template<class T>
int GDALImage<T>:: GetD_PixelCnt(int ROW, int COL,int i, int j, T val, T* chData)
{
	int nCnt = 0;
	if(chData[(i+1)*COL+(j)] == val) nCnt++;
	if(chData[(i+1)*COL+(j-1)] == val) nCnt++;
	if(chData[(i+1)*COL+(j+1)] == val) nCnt++;
	return nCnt;
}

template<class T>
int GDALImage<T>:: GetR_PixelCnt(int ROW, int COL,int i, int j, T val, T* chData)
{
	int nCnt = 0;
	if(chData[(i-1)*COL+(j+1)] == val) nCnt++;
	if(chData[(i)*COL+(j+1)] == val) nCnt++;
	if(chData[(i+1)*COL+(j+1)] == val) nCnt++;
	return nCnt;
}