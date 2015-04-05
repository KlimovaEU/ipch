
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#define PI 3.14159265
#include  <opencv\cv.h>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;
using namespace std;
struct sphere 
{ 
	int x,y,d;
};

struct quad 
{
	Point a, b, c, d; 
	int s;					         	 // кол-во точек
	unsigned long long int sum_x, sum_y;                 // суммарное значение по координате
} ;

Point inters(struct sphere sph, int i, int j,int n)
{   
	double x,y,z;
	i = i % n;
	j = j % n;
	x=sph.d *sin (2*PI/n*i)*cos(2*PI/n*j) / 2 +sph.x;
	y=sph.d *sin (2*PI/n*i)*sin(2*PI/n*j) / 2 + sph.y;
	z=sph.d *cos (2*PI/n*i) /2 + sph.d/2;
	double l =x-sph.x;
	double m =y-sph.y;
	double p =z-sph.d;
	
	Point a;
	a.x = int(sph.x - (sph.d *l)/p );
	a.y = int(sph.y - (sph.d *m)/p );
	return a;
}

Point planeInters ( Point c1, Point c2,Point l3,Point l4)
{
	//точка пересечения двух линий,где каждая задана двумя точками
	double a1,b1,a2,b2,c,d;
	if (abs(c1.x-c2.x)>0)
		a1=(c1.y -c2.y)/(c1.x-c2.x);
	else a1=(c1.y -c2.y);

	if (abs(l3.x-l4.x)>0)
		a2=(l3.y -l4.y)/(l3.x-l4.x);
	else a2=(l3.y -l4.y);

	b1= c1.y - a1*c1.x;               //a1 * x -1 *y + b1 =0
	b2= l3.y - a2*l3.x;				  //a2 * x -1 *y + b2 =0
	c= -a1 + a2;
	Point a;
	a.x = (-b2 +b1)/c;
	a.y = (- a1*b2 +a2*b1)/c;

	return a;
}

double squareTriangle(Point a, Point b, Point c)
{	
	return 0.5*abs((a.x -c.x)*(b.y-c.y) - (b.x-c.x)*(a.y-c.y));
}

bool belong_to_quad ( struct quad q , Point a)
{// проверка принадлежности точки четырехугольнику
	bool b;
	double s1,s2;
	s1 = squareTriangle(q.a, q.b, q.c)+squareTriangle(q.c, q.b,q.d);
	s2 = 0;
	s2+= squareTriangle(q.a, q.d, a);
	s2+= squareTriangle(q.a, q.c, a);
	s2+= squareTriangle(q.c, q.b, a);
	s2+= squareTriangle(q.d, q.b, a);
	if (s1+0.01<s2) b=0;
	else b=1;

	return b;

}


void help()
{
	cout << "\n Default is test2.jpeg\n" << endl;
}

int main(int argc, char** argv)
{
	const char* filename = argc >= 2 ? argv[1] : "test1.jpeg";

	Mat src = imread(filename, 0);
	if(src.empty())
	{
		help();
		cout << "can not open " << filename << endl;
		return -1;
	}

	Mat dst, abs_dst,gray,cdst;
	GaussianBlur( src,abs_dst, Size(3,3), 1, 0, BORDER_DEFAULT );
	Laplacian(abs_dst, dst, CV_16S, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs(dst, dst);
	threshold(dst,dst,50,180,THRESH_BINARY);
	cvtColor( dst, cdst, CV_GRAY2BGR); 
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 50, 20, 10 );

	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 1, CV_AA);
	}
	
	Point pt3,pt2;
	const int n=30;    // количество меридиан
	const int nn =n*n; 
	struct sphere sph; // координаты центра (x,y) и диаметр
	sph.x =int( cdst.cols /2);
	sph.y =int( cdst.rows /2);
	cout<< cdst.cols <<" "<< cdst.rows<<endl;
	sph.d = 1000;
	double a=1.5*PI;
	int step =50;
	int ind;
	vector<quad> quadVector;
	vector<int> index_list,i_l,ix_r1, ix_r,i_m,ix_l,ix_l1;
	vector<vector<int>> coord,coord_x0,coord_xrows;
	quad q;
	//
	// заполнение массива четырехугольников значениями координат углов
	//
	for (int i=1; i<n; i++) 
	{
		for (int j=1; j<n; j++)
		{
			// пересечение сферы и плоскости
			q.s = 0;
			q.sum_x=0;
			q.sum_y=0;
			q.a = inters(sph,i,j,n);  
			q.b = inters(sph,i,j+1,n);  
			q.c = inters(sph,i+1,j,n);
			q.d = inters(sph,i+1,j+1,n);  
			quadVector.push_back(q);
		}
	}

	// упорядочим по координатам список quadVector-ов
	for( size_t i = 0; i <quadVector.size(); i++)
	{
		//отдельно запоминаем те quadVector, которые лежат ЛЕВЕЕ изображения
		if (quadVector[i].a.y <0)
			{	
				if( quadVector[i].a.x<0)

					ix_l.push_back(i);
				else 
					if ( quadVector[i].a.x>cdst.cols)
						ix_r.push_back(i);
					else 
						i_l.push_back(i);
				
			}				
		else 
		//отдельно запоминаем те quadVector, которые лежат ПРАВЕЕ изображения
			if (quadVector[i].a.y>cdst.rows)
			{	
				if( quadVector[i].a.x<0)
					ix_l1.push_back(i);
				else 
					if ( quadVector[i].a.x>cdst.cols)
						ix_r1.push_back(i);
					else 
						index_list.push_back(i);
				
			}			
	}
	// 
	//  заполнение coord_x0 
	//  coord_x0- вектор, который хранит номера quadVector( у которых a.y<0), упорядоченные по x
	coord_x0.push_back(ix_l);
	ix_l.clear();
	for (int k = 1; k<=cdst.cols/(1*step); k++)
	{
		for( size_t i = 0; i <i_l.size(); i++)
		{
			int t= i_l[i];
			if (quadVector[t].a.x <k*step && quadVector[t].a.x>=(k-1)*step)
				i_m.push_back(i);	
			else 
				if (quadVector[t].a.x>=(cdst.cols/step)*step)
					ix_l.push_back(i);
		}
		coord_x0.push_back(i_m);
		i_m.clear();
		
	}
	coord_x0.push_back(ix_l);
	coord_x0.push_back(ix_r);
	ix_r.clear();
	i_l.clear();
	//
	// 
	//  заполнение coord_xrows 
	//  coord_xrows- вектор, который хранит номера quadVector( у которых a.y>rows), упорядоченные по x
	coord_xrows.push_back(ix_l1);
	for (int k = 1; k<=cdst.cols/(1*step); k++)
	{
		for( size_t i = 0; i <index_list.size(); i++)
		{
			int t= index_list[i];
			if (quadVector[t].a.x <k*step && quadVector[t].a.x>=(k-1)*step)
					i_m.push_back(i);	
			else 
				if (quadVector[t].a.x>=(cdst.cols/step)*step)
					i_l.push_back(i);
		}
		coord_xrows.push_back(i_m);
		i_m.clear();	
	}
	coord_xrows.push_back(i_l);
	i_l.clear();
    coord_xrows.push_back(ix_r1);
	ix_r1.clear();
	index_list.clear();
	// filling coord (the main part of classification)
 

	for (int k = 1; k<=(cdst.rows/step); k++)
	{
		for( size_t i = 0; i <quadVector.size(); i++)
		{
			if (quadVector[i].a.y <k*step && quadVector[i].a.y>=(k-1)*step)
					i_l.push_back(i);
			else
				if (quadVector[i].a.y>=(cdst.cols/step)*step)
					i_m.push_back(i);
		}
		coord.push_back(i_l);
		i_l.clear();
	}
	coord.push_back(i_m);
	i_m.clear();
	//dst.release();
	//
	//search of line crosses
	//
	
	for( auto i = lines.begin(); i!=lines.end(); i++)
	{
			Vec4i l = *i;     
			for(auto j =i+1; j != lines.end(); j++) 
			{
				Vec4i p = *j;
				pt2=planeInters(Point(l[0],l[1]),Point(l[2],l[3]),Point(p[0],p[1]),Point(p[2],p[3])); // точка пересечения двух прямых в плоскости 
				//
				//vector choosing with necessary index
				//
				if (pt2.y<0)
					if (pt2.x<0)
						i_l=coord_x0[0];
					else 
						if(pt2.x>cdst.cols)
							i_l =coord_x0[coord_x0.size()-1];
						else
							i_l = coord_x0[ceil((double( pt2.x/step)))];

				else 
					if (pt2.y>cdst.rows)
						if (pt2.x<0)
							i_l=coord_xrows[0];
						else 
							if(pt2.x>cdst.cols)
								i_l =coord_xrows[coord_xrows.size()-1];
							else
								i_l = coord_xrows[ceil((double( pt2.x/step)))];
					else 
						//point lies inside the image
						{
							ind =ceil((double( pt2.y/step)));
						//	cout<<pt2.x<<" "<<pt2.y<<" "<<ind<<" "<<coord.size()<<endl;
							i_l=coord[ind];
						}
				for (size_t k= 0; k <i_l.size(); k++) 
				{
					if (belong_to_quad(quadVector[i_l[k]],pt2)) // checking  point belonging to the quadVector
					{
						quadVector[i_l[k]].s++;
						quadVector[i_l[k]].sum_x+=pt2.x;
						quadVector[i_l[k]].sum_y+=pt2.y;
						break;
					}
				}
			 }               
	}
	//maximum(s) search 
	//
	int ind1=0, ind2=0,ind3 =0;
	int max1=quadVector[0].s;
	int max2=quadVector[0].s;
	int max3=quadVector[0].s;
	for (size_t i = 0; i <quadVector.size(); i++) 
	{
		if (quadVector[i].s>max1)
		{
			max3 =max2;
			ind3 =ind2;
			max2 =max1;
			ind2 =ind1;
			max1 =quadVector[i].s;
			ind1 =i;
		}
		else 
			if (quadVector[i].s>max2)
			{
				max3 =max2;
				ind3 =ind2;
				max2 = quadVector[i].s;
				ind2 = i;
			}
			else 
				if(quadVector[i].s>max3)
				{
					max3 = quadVector[i].s;
					ind3 = i;
				}
		cout<<i<<' '<<quadVector[i].s<<endl; 
	}

	// convert matrix
	CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
	IplImage *src1=0, *dst1=0;
	src1 = cvLoadImage(filename,1);
	//		setting one element 
	//		void  cvmSet( CvMat* mat, int row, int col, double value );
	//      
	double x,y, x1,y1,a1,b;
	x =(quadVector[ind1].sum_x/quadVector[ind1].s);
	y =(quadVector[ind1].sum_y/quadVector[ind1].s);
	x1=(quadVector[ind2].sum_x/quadVector[ind2].s);
	y1=(quadVector[ind2].sum_y/quadVector[ind2].s);
	//vanishing points
	cout<<x<<" "<<y<<endl;
    	cout<<x1<<" "<<y1<<endl;

	//  calculate the equation of vanishing line using cross product
	b = x*y1 - x1*y;
	a1= (y-y1)/b;
	b= (x1-x)/b;
    	//  a1*x - y+ b = 0
	//  lead to this form:  a1/b *x  - y/b + 1 = 0
	cvmSet( warp_matrix, 0, 0, 1 ); 
	cvmSet( warp_matrix, 1, 0, 0 );
	cvmSet( warp_matrix, 2, 0, a1);
	cvmSet( warp_matrix, 0, 1, 0 );
	cvmSet( warp_matrix, 1, 1, 1 );			
	cvmSet( warp_matrix, 2, 1, b );
	cvmSet( warp_matrix, 0, 2, 0 );
	cvmSet( warp_matrix, 1, 2, 0 );
	cvmSet( warp_matrix, 2, 2, 1 );

	dst1 = cvCloneImage(src1);
	// convert prospects
	cvWarpPerspective(src1,dst1,warp_matrix);

	double u,v,u1,v1,cot;
	// calculate new vanishing points, u=H*x; v=H*x; where H=warp_matrix;
	u =x/(a1*x + b*y +1); 
	v =y/(a1*x + b*y +1);

	u1 =x1/(a1*x1 + b*y1 +1);
	v1 =y1/(a1*x1 + b*y1 +1);
	// rotation matrix
	cvmSet( warp_matrix, 0, 0, cos(u/sqrt(u*u+v*v))); 
	cvmSet( warp_matrix, 1, 0, sin(u/sqrt(u*u+v*v)));
	cvmSet( warp_matrix, 2, 0, 0);
	cvmSet( warp_matrix, 0, 1, -sin(u/sqrt(u*u+v*v)) );
	cvmSet( warp_matrix, 1, 1, cos(u/sqrt(u*u+v*v)) );			
	cvmSet( warp_matrix, 2, 1, 0 );
	cvmSet( warp_matrix, 0, 2, 0 );
	cvmSet( warp_matrix, 1, 2, 0 );
	cvmSet( warp_matrix, 2, 2, 1 );
	cvWarpPerspective(dst1,dst1,warp_matrix);

	//calculate ctg between the vectors (u,v) and (u1,v1)
	cot = (u*u1 + v*v1)/abs(u*v1 -u1*v);
	cout<<cot<<" cot"<<endl;

	//                      ( 1, cot, 0,
	//                        0,  1,  0,
	// our new matix  for     0,  0 , 1)
	// restoring the orthogonality of angles
	cvmSet( warp_matrix, 0, 0, 1 ); 
	cvmSet( warp_matrix, 1, 0, 0 );
	cvmSet( warp_matrix, 2, 0, 0);
	cvmSet( warp_matrix, 0, 1, cot );
	cvmSet( warp_matrix, 1, 1, 1 );			
	cvmSet( warp_matrix, 2, 1, 0 );
	cvmSet( warp_matrix, 0, 2, 0 );
	cvmSet( warp_matrix, 1, 2, 0 );
	cvmSet( warp_matrix, 2, 2, 1 );
	
	cvWarpPerspective(dst1,dst1,warp_matrix);

	cvNamedWindow( "cvWarpPerspective",0);
	cvShowImage( "cvWarpPerspective", dst1 );

	cvWaitKey(0);
	cvReleaseMat(&warp_matrix);
	cvReleaseImage(&src1);
	cvReleaseImage(&dst1);

	namedWindow("detected lines",WINDOW_NORMAL);
	imshow("detected lines", cdst);
	imwrite("detected lines.jpg", cdst);
	waitKey();
	return 0;
}
