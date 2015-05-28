
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
double length (Point a, Point b)
{   
	return sqrt(double((a.x -b.x)*(a.x -b.x) +(a.y-b.y)*(a.y-b.y)));
}
struct quad 
{
	Point a, b, c, d; 
	int s;												 // кол-во точек
	unsigned long long int sum_x, sum_y;                 // суммарное значение по координате
} ;


Point inters(struct sphere sph, int i, int j,int n)
{   
	double sphx =sph.x, sphy = sph.y, sphd =sph.d;
	double x,y,z;
	i = i % (n+1);
	j = j % (n+1);
	x=sphd *sin (2*PI*i/n)*cos(2*PI*j/n)/ 2 + sphx;
	y=sphd *sin (2*PI*i/n)*sin(2*PI*j/n)/ 2 + sphy;
	z=sphd *cos (2*PI*i/n)/2 + sphd/2;
	double l =x-sphx;
	double m =y-sphy;
	double p =z-sphd;
	Point a;
	a.x = int(sphx - (sphd *l)/p );
	a.y = int(sphy - (sphd *m)/p );

	return a;
}

double tangens (Point a , Point b)
{
	if (a.x==b.x) return 100;
	else
		if (a.x<b.x)	
		{	
			return (b.y -a.y)/(b.x -a.x);

		}
		else 
			return (a.y -b.y)/(a.x - b.x);
}
bool belong_to_rightSide (Point a , Point b)
{
	if (a.x<b.x)	
	{
		if  (a.y<b.y)	
			return 1;
		else return  0;
	}
	else 
		if  (a.y>b.y)	
			return 1;
		else return  0;
}
Point planeInters ( Point c1, Point c2,Point l3,Point l4)
{
	double c1x = c1.x, c2x =c2.x, l3x = l3.x, l4x = l4.x;
	double c1y = c1.y, c2y =c2.y, l3y = l3.y, l4y = l4.y;

	double a1,b1,a2,b2,c,d;
	if (abs(c1x-c2x)>0)
		a1=(c1y -c2y)/(c1x-c2x);
	else a1=(c1y -c2y);
	if (abs(l3x-l4x)>0)
		a2=(l3y -l4y)/(l3x-l4x);
	else a2=(l3y -l4y);

	b1= c1y - a1*c1x;               //a1 * x -1 *y + b1 =0
	b2= l3y - a2*l3x;				  //a2 * x -1 *y + b2 =0
	c= -a1 + a2;
	Point a;
	a.x = (b1 - b2)/c;
	a.y = ( a2*b1 -a1*b2)/c;

	return a;
}

double squareTriangle(Point a, Point b, Point c)
{	
	double ax= a.x, bx = b.x, cx =c.x;
	double ay = a.y, by =b.y, cy= c.y;
	return  abs((ax -cx)*(by-cy) - (bx-cx)*(ay-cy))/2;
}

bool belong_to_quad ( struct quad q , Point a)
{// проверка принадлежности точки четырехугольнику
	bool b;
	double s1,s2;
	s1 = squareTriangle(q.a, q.b, q.c)+squareTriangle(q.c, q.d,q.b);
	s2 = 0;
	s2+= squareTriangle(q.a, q.b, a);
	s2+= squareTriangle(q.a, q.c, a);
	s2+= squareTriangle(q.c, q.d, a);
	s2+= squareTriangle(q.d, q.b, a);
	if (abs(s1-s2)>1) b=0;
	else b=1;
	//	cout<<abs(s1-s2)<<endl;
	return b;

}


void help()
{
	cout << "\n Default is test2.jpeg\n" << endl;
}
vector<Point> search_susp(Point begin, Point end, Mat grad_x)
{
	vector<Point> r;
	double val, val1,mid;
	while (r.size()==0 && abs(begin.x -end.x)>2)
	{
		val =grad_x.at<float>(begin.x,begin.y);
		val1=grad_x.at<float>(end.x,end.y);
		if (val==0)
		{
			r.push_back(begin);
		}
		if (val1==0)
		{
			r.push_back(end);
		}
		else 
			if (val1*val<0 )
			{ 
				mid =grad_x.at<float>((end.x+ begin.x)/2,(end.y + begin.y)/2);
				if (mid == 0) r.push_back(Point((end.x+ begin.x)/2,(end.y + begin.y)/2));
				else
					if (val*mid<0)
						end= Point((end.x+ begin.x)/2,(end.y + begin.y)/2);
					else 
						begin =Point((end.x+ begin.x)/2,(end.y + begin.y)/2);
			}
			else break;
	}
	return r;
}

vector<Point> change_point(const char* filename, vector<Point> a_begin,  vector<Point> a_end)
{
	Mat src, src_gray;
	Mat grad;
	char* window_name = "Sobel Demo - Simple Edge Detector";
	int scale = 1;
	int delta = 0;
	int ddepth = CV_32F;
	Point p;
	/// Load an image
	src = imread( filename,0);
	GaussianBlur( src, src, Size(3,3), 0, 0, BORDER_DEFAULT );
	cvtColor( src, src_gray,CV_GRAY2BGR );

	Mat grad_x;
	Sobel( src_gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	vector <Point> susp_p, hp;               //suspicious points, points that could be border 
	for (size_t i=0; i<a_begin.size(); i++)
	{
		hp.clear();
		hp=search_susp(a_begin[i],a_end[i], grad_x);	
		for (size_t j=0; j<hp.size(); j++)
			susp_p.push_back(hp[j]);
	}
	return susp_p;
}

Mat image_return (const char* filename)
{
	Mat src = imread(filename, 0);
	Mat dst, abs_dst,gray,cdst;
	GaussianBlur( src,abs_dst, Size(3,3), 1, 0, BORDER_DEFAULT );
	Laplacian(abs_dst, dst, CV_16S, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs(dst, dst);
	threshold(dst,dst,80,180,THRESH_BINARY);
	cvtColor( dst, cdst, CV_GRAY2BGR); 
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 50, 20, 10 );
	vector<Point> a_begin,a_end;
	double h;

	for( size_t i = 0; i < lines.size(); i++ )
	{   
		Vec4i l = lines[i];
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 4, CV_AA);
		if ( abs( tangens(Point(l[0], l[1]),Point(l[2], l[3])) ) <0.15)
		{
			h=length(Point(l[0], l[1]),Point(l[2], l[3]));
			if (h>50)
			{
				a_begin.push_back( Point(l[0], l[1]));
				a_end.push_back(Point(l[2], l[3]));
			}
		}
	}
	a_begin=change_point(filename, a_begin, a_end); //getting suspecicious points that could be border 

	double len =0,len1=0, h1;
	Point b1, b2,b3,b4; //borders
	for( size_t i = 0; i < lines.size(); i++ )
	{
		Vec4i l = lines[i];
		if( belong_to_rightSide(Point(l[0], l[1]),Point(l[2], l[3]))!=1)
			line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 4, CV_AA);
		if (tangens(Point(l[0], l[1]),Point(l[2], l[3]))<-8)
		{ 
			h=length(Point(l[0], l[1]),Point(l[2], l[3]));
			if (len< h)
			{
				len = h;
				b1 = Point(l[0], l[1]);
				b2 = Point(l[2], l[3]);
			}
		}
		if (tangens(Point(l[0], l[1]),Point(l[2], l[3]))>8)
		{
			h1=length(Point(l[0], l[1]),Point(l[2], l[3]));
			if (len1< h1)
			{
				len1 = h1;
				b3 = Point(l[0], l[1]);
				b4 = Point(l[2], l[3]);
			}
		}
	}
	vector<Point> pnt;
	Point up = planeInters(b1, b2, b3, b4);
	Point bot, lim;
	Point pt,down;
	int s=0;
	bot.y = cdst.rows;
	lim.y = cdst.rows;
	bot.x = b2.x + double((bot.y -b2.y)*(b1.x - b2.x))/double(b1.y -b2.y);
	lim.x = b4.x + double((bot.y -b4.y)*(b3.x - b4.x))/double(b3.y -b4.y);

	for( size_t i = 0; i < a_begin.size(); i++ )
	{
		pt =a_begin[i];
		if ((b3.x!=b4.x)&&(b1.x!=b2.x)&&pt.y>(b1.y-b2.y)*(pt.x -b2.x)/(b1.x-b2.x) +b2.y  && pt.y>(b3.y-b4.y)*(pt.x -b4.x)/(b3.x-b4.x) +b4.y) 
		{
			pnt.push_back(pt);
			s+=pt.x;
		}
		circle(cdst,pt,5, Scalar(0,255,0),5, 8, 0);
	}
	s/=pnt.size();
	line( cdst, Point(s,cdst.rows), up, Scalar(255,0,255), 8, CV_AA);
	line( cdst, b3, b4, Scalar(0,255,0), 8, CV_AA);
	line( cdst, b1, b2, Scalar(0,255,0), 8, CV_AA);
	namedWindow("det",WINDOW_NORMAL);
	imshow("det", cdst);
	//waitKey();
	//return  src(Rect(up.x,0, cdst.cols -lim.x-2, cdst.rows-1)).clone();
	return  src(Rect(0,0,s,cdst.rows)).clone();
}
vector<Point> max_search(int n, vector<vector<quad>> quadVector)
{
		Point ind1=Point(0,0), ind2=Point(0,0),ind3 =Point(0,0);
		int max2=quadVector[0][0].s;
		int max1=quadVector[0][0].s;
		int max3=quadVector[0][0].s;
		vector<Point> r;
		for (int i=1; i<n/2; i++)
			for( int j = 0; j <n; j++)
			{
				i--;
				cout<<i<<" "<<j<<endl;
				if (i>0&& i<(n/2-2) &&  j>0 && j<n-1 && quadVector[i][j].s>quadVector[i-1][j].s &&quadVector[i][j].s>quadVector[i-1][j-1].s&& quadVector[i][j].s>quadVector[i-1][j+1].s&&
					quadVector[i][j].s>quadVector[i][j+1].s && quadVector[i][j].s>quadVector[i][j-1].s && quadVector[i][j].s>quadVector[i+1][j+1].s && quadVector[i][j].s>quadVector[i+1][j-1].s && quadVector[i][j].s>quadVector[i+1][j].s ||
					i==0 && j>0&& j<n-1 && quadVector[i][j].s>quadVector[i+1][j].s&& quadVector[i][j].s>quadVector[i][j+1].s&& quadVector[i][j].s>quadVector[i][j-1].s ||
					i==(n/2-2) && j>0&& j<n-1 && quadVector[i][j].s>quadVector[i-1][j].s&& quadVector[i][j].s>quadVector[i][j+1].s&& quadVector[i][j].s>quadVector[i][j-1].s ||
					i>0 && i<(n/2-2) && j==0 && quadVector[i][j].s>quadVector[i-1][j].s&& quadVector[i][j].s>quadVector[i][j+1].s&& quadVector[i][j].s>quadVector[i+1][j].s && quadVector[i][j].s>quadVector[i][n-1].s||
					i>0 && i<(n/2-2) && j==n-1 && quadVector[i][j].s>quadVector[i-1][j].s&& quadVector[i][j].s>quadVector[i][j-1].s&& quadVector[i][j].s>quadVector[i+1][j].s && quadVector[i][j].s>quadVector[i][0].s)
				{
					if (quadVector[i][j].s>max1)
					{   
						max3 =max2;
						ind3 =ind2;
						max2 =max1;
						ind2 =ind1;
						max1 =quadVector[i][j].s;
						ind1 =Point(i,j);
					}
					else 
						if (quadVector[i][j].s>max2)
						{
							max3 =max2;
							ind3 =ind2;
							max2 = quadVector[i][j].s;
							ind2 = Point(i,j);
						}
						else 
							if(quadVector[i][j].s>max3)
							{
								max3 = quadVector[i][j].s;
								ind3 = Point(i,j);
							}
				}
				i++; 
			}
			
			double x,y, x1,y1,a1,b, x2,y2;
			x =(quadVector[ind1.x][ind1.y].sum_x/quadVector[ind1.x][ind1.y].s);
			y =(quadVector[ind1.x][ind1.y].sum_y/quadVector[ind1.x][ind1.y].s);
			cout<<"x "<<x<<" y "<<y<<endl;
			r.push_back(Point(x,y));

			x1=(quadVector[ind2.x][ind2.y].sum_x/quadVector[ind2.x][ind2.y].s);
			y1=(quadVector[ind2.x][ind2.y].sum_y/quadVector[ind2.x][ind2.y].s);
			cout<<"x1 "<<x1<<" y1 "<<y1<<endl;
			r.push_back(Point(x1,y1));

			x2 =(quadVector[ind3.x][ind3.y].sum_x/quadVector[ind3.x][ind3.y].s);
			y2 =(quadVector[ind3.x][ind3.y].sum_y/quadVector[ind3.x][ind3.y].s);
			cout<<"x2 "<<x2<<" y2 "<<y2<<endl;
			r.push_back(Point(x2,y2));

			cout<<ind1.x<<" "<<ind1.y<<endl;
			cout<<ind2.x<<" "<<ind2.y<<endl;
			return r;
}
int main(int argc, char** argv)
{
	const char* filename = argc >= 2 ? argv[1] : "test1.jpeg";
//	Mat src = imread(filename,0);    
	Mat src =image_return( filename);
	Point pt3,pt2;
	Mat dst, abs_dst,gray,cdst;
	GaussianBlur( src,abs_dst, Size(3,3), 1, 0, BORDER_DEFAULT );
	Laplacian(abs_dst, dst, CV_16S, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs(dst, dst);
	threshold(dst,dst,80,180,THRESH_BINARY);
	cvtColor( dst, cdst, CV_GRAY2BGR); 
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 50, 20, 10 );
	cout<<dst.cols<<" "<<dst.rows<<endl;
	double h;
	vector<Point> a_begin,a_end;
	for( size_t i = 0; i < lines.size(); i++ )
	{   
		Vec4i l = lines[i];
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 4, CV_AA);

	}	
	const int n=30;    // number of meridians
	const int nn =n*n; 
	struct sphere sph; 
	// centre coord (x,y) 
	sph.x =int( cdst.cols /2);
	sph.y =int( cdst.rows /2);
	cout<< cdst.cols <<" "<< cdst.rows<<endl;
	sph.d = min(cdst.cols,cdst.rows);
	double a=1.5*PI;
	int step =400;
	int ind;
	vector<Point> index_list,i_l,ix_r1, ix_r,i_m,ix_l,ix_l1;
	vector<vector<Point>> coord,coord_x0,coord_xrows;
	quad q;
	vector <vector<quad>> quadVector;
	//
	// заполнение массива четырехугольников значениями координат углов
	//
	for (int i=1; i<n/2; i++) 
	{
		quadVector.push_back(vector<quad>(n, q));
		for (int j=0; j<n; j++)
		{
			// sphere projection 
			q.s = 0;
			q.sum_x=0;
			q.sum_y=0;
			q.a = inters(sph,i,j,n);  
			circle(cdst,q.a,5, Scalar(0,255,0),5, 8, 0);

			q.b = inters(sph,i,j+1,n);  
			q.c = inters(sph,i+1,j,n);
			q.d = inters(sph,i+1,j+1,n); 

			line( cdst, q.a, q.b, Scalar( 150,0,150 ),  2, 8 );
			line( cdst, q.a, q.c, Scalar( 150,0,150 ),  2, 8 );
			line( cdst, q.d, q.c, Scalar( 150,0,150 ),  2, 8 );
			line( cdst, q.d, q.b, Scalar( 150,0,150 ),  2, 8 );
			quadVector[i-1][j]=q;
			cout<<i-1<<" "<<j<<endl;
		}
	}

	// regularization  of quadVector by y-coord 

	for( int j = 0; j <n; j++)
		for (int i=1; i<n/2; i++)
		{
			i--;
			//отдельно запоминаем те quadVector, которые лежат выше изображения
			if (quadVector[i][j].a.y <=0||quadVector[i][j].b.y<=0||
				quadVector[i][j].c.y<=0||quadVector[i][j].d.y<=0)
			{	
				ix_l.push_back(Point(i,j));

			}				

			//отдельно запоминаем те quadVector, которые лежат ниже изображения
			if (quadVector[i][j].a.y>=cdst.rows ||quadVector[i][j].c.y>=cdst.rows||
				quadVector[i][j].b.y>=cdst.rows ||quadVector[i][j].d.y>=cdst.rows)
			{			
				ix_r.push_back(Point(i,j));			
			}	
			i++;
		}
		// 
		//  coord_x0- vector that saves coordinates of quadVector(with any_point.y<0)
		coord_x0.push_back(ix_l);
		ix_l.clear();

		coord_x0.push_back(ix_r);
		ix_r.clear();
		i_l.clear();

		// filling coord (the main part of classification)


		for (int k = 1; k<=(cdst.rows/step); k++)
		{
			for( int j = 0; j <n; j++)
				for (int i=1; i<n/2; i++)
				{
					i--;
					if (quadVector[i][j].a.y <=k*step && quadVector[i][j].a.y>=(k-1)*step || quadVector[i][j].c.y <k*step && quadVector[i][j].c.y>=(k-1)*step ||
						quadVector[i][j].b.y <=k*step && quadVector[i][j].b.y>=(k-1)*step || quadVector[i][j].d.y <k*step && quadVector[i][j].d.y>=(k-1)*step)
						i_l.push_back(Point(i,j));
					if (quadVector[i][j].a.y>=(cdst.cols/step)*step ||quadVector[i][j].c.y>=(cdst.cols/step)*step ||
						quadVector[i][j].b.y>=(cdst.cols/step)*step ||quadVector[i][j].d.y>=(cdst.cols/step)*step )
						i_m.push_back(Point(i,j));
					i++;
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

		int ned =0;	
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

				i_l.clear();
				i_m.clear(); //create additional vector
				ix_r.clear();
				if (pt2.y<=0)
				{
					i_l=coord_x0[0];
					i_m=coord[0];
				}
				else
					if (pt2.y>=cdst.rows)
					{
						i_l =coord_x0[1];
						i_m =coord[coord.size()-1];
					}
					else
						//point lies inside the image
					{
						ind =ceil((double( pt2.y/step)));
						i_l=coord[ind];
						if (ind==0)
						{ 
							i_m=coord_x0[0];
							ix_r=coord[ind+1];
						}
						else 
							if (ind==coord.size()-1)
							{
								i_m=coord[ind-1];
								ix_r=coord_x0[1];
							}
							else 
							{
								i_m=coord[ind-1];
								ix_r=coord[ind+1];
							}

					}
					bool per=1;

					for (size_t k= 0; k <i_l.size(); k++) 
					{
						if (belong_to_quad(quadVector[i_l[k].x][i_l[k].y],pt2)) // checking  point belonging to the quadVector
						{
							quadVector[i_l[k].x][i_l[k].y].s++;
							quadVector[i_l[k].x][i_l[k].y].sum_x+=pt2.x;
							quadVector[i_l[k].x][i_l[k].y].sum_y+=pt2.y;
							per =0;
							break;					
						}
					}

					// if flag per==1 we continue search in neighbor strip
					if (per ==1 && i_m.size()>0)
						for (size_t k= 0; k <i_m.size(); k++) 
						{
							if (belong_to_quad(quadVector[i_m[k].x][i_m[k].y],pt2)) // checking  point belonging to the quadVector
							{
								quadVector[i_m[k].x][i_m[k].y].s++;
								quadVector[i_m[k].x][i_m[k].y].sum_x+=pt2.x;
								quadVector[i_m[k].x][i_m[k].y].sum_y+=pt2.y;
								per =0;
								break;					
							}
						}
						if (per ==1 && ix_r.size()>0)
							for (size_t k= 0; k <ix_r.size(); k++) 
							{
								if (belong_to_quad(quadVector[ix_r[k].x][ix_r[k].y],pt2)) // checking  point belonging to the quadVector
								{
									quadVector[ix_r[k].x][ix_r[k].y].s++;
									quadVector[ix_r[k].x][ix_r[k].y].sum_x+=pt2.x;
									quadVector[ix_r[k].x][ix_r[k].y].sum_y+=pt2.y;
									per =0;
									break;					
								}
							}

							if (per ==1) 
							{
								ned++;
								circle(cdst,pt2,5, Scalar(0,150,150),5, 8, 0);
							} //points that are not found (they appear when eps is low in belonging fuction)
			}   

		}
		//maximum(s) search 
		//
		vector<Point> r;
		r=max_search(n, quadVector);
		circle(cdst,r[0],5, Scalar(255,0,0),5, 8, 0);
		circle(cdst,r[1],5, Scalar(255,0,0),5, 8, 0);

		// convert matrix
		//CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
		Mat warp_matrix(3,3,CV_32FC1);
		IplImage *src1=0, *dst1=0;
		//src1 = cvLoadImage(filename,1);

		//		setting one element 
		//		void  cvmSet( CvMat* mat, int row, int col, double value );
		//   
		//  calculate the equation of vanishing line using cross product
		double x,y,x1,y1,a1,b;
		x=r[0].x; 
		y=r[0].y;
		x1=r[1].x; 
		y1=r[1].y;
		b = x*y1 - x1*y;
		a1= (y-y1)/b;
		b= (x1-x)/b;
		//  a1*x - y+ b = 0
		//  lead to this form:  a1/b *x  - y/b + 1 = 0

		//matrix H

		warp_matrix.at<float>(0,0) =  x; 
		warp_matrix.at<float>(1,0) =  y; 
		warp_matrix.at<float>(2,0) =  1;                // a*x +b*y +c =0
		warp_matrix.at<float>(0,1) =  x1; 
		warp_matrix.at<float>(1,1) =  y1; 
		warp_matrix.at<float>(2,1) =  1; 
		warp_matrix.at<float>(0,2) =  a1; 
		warp_matrix.at<float>(1,2) =  b; 
		warp_matrix.at<float>(2,2) =  1; 

		//	cout<<ned<<" ned"<<endl;
		// convert prospects
		std::cout << warp_matrix << std::endl;
		warpPerspective(src, dst, warp_matrix, cv::Size(3000,3000));

		namedWindow("warpPerspective",WINDOW_NORMAL);
		imshow("warpPerspective", dst);
		waitKey();

		namedWindow("detected lines0",WINDOW_NORMAL);
		imshow("detected lines0", cdst);
		imwrite("detected lines0.jpg", cdst);

		waitKey();
		return 0;
}
