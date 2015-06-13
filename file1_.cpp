

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#define PI 3.14159265
#include <opencv\cv.h>
#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <fstream>

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
	long long int sum_x, sum_y;				// суммарное значение по координате
	
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

vector<double> line_coord ( Point c1, Point c2)
{
	double x1 = c1.x, x2 =c2.x, y1 = c1.y, y2 =c2.y;
	vector<double> v;
	double a,b,c,d;
	
	a=y1 - y2;
	b=x2 - x1;	
	c= x1*y2 - x2*y1;               //a* x +b*y + c =0 
	                                  	//a*x + c*y +1=0
	d=sqrt(a*a+b*b);

	v.push_back(a/d);
	v.push_back(b/d);
	v.push_back(c/d);

	return v;
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
Point VP(vector<vector<double>> vv)
{
	Mat A(vv.size(),3,CV_64FC1);
	Mat w, u, vt;
	double a;
	Point p;
	for (size_t i=0;i<vv.size();i++)
	{
		A.at<double>(i,0) = vv[i][0]; 
		A.at<double>(i,1) = vv[i][1];
		A.at<double>(i,2) = vv[i][2];  
	}
	SVD::compute(A, w, u, vt);
	int rows = vt.rows;
//	cout<<vt.at<double>(rows-1,0)<<" "<<vt.at<double>(rows-1,1)<<" "<<vt.at<double>(rows-1,2)<<endl;
	a=vt.at<double>(rows-1,0)/ vt.at<double>(rows-1,2); 
	cout<<a<<" "; p.x=a;
	a=vt.at<double>(rows-1,1)/ vt.at<double>(rows-1,2); p.y=a;
	cout<<a<<endl;
	return p;
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
Point centre(vector<Point> v) // centre search of one cell
{
	int k_min;
	vector<double> dist;
	double s=0,s_min;
	for (size_t i=0; i<v.size();i++)
	{
		for(size_t j=0;j<v.size(); j++)
		{
			s+=sqrt(double((v[i].x-v[j].x)*(v[i].x-v[j].x)+ (v[i].y-v[j].y)*(v[i].y-v[j].y)));
		}
		dist.push_back(s);
		s=0;
	} 
	s_min=dist[1];
	for (size_t i=1; i<dist.size();i++)
		if (dist[i]<s_min)
		{
			k_min=i;
			s_min=dist[i];
		}
	return v[k_min];	
}
vector<Point> max_search(int n, vector<vector<quad>> quadVector)
{
		Point ind1=Point(10,10);
		int max1=quadVector[1][1].s;
		vector<Point> r;
		for (int i=0; i<n/2-1; i++)
			for( int j = 0; j <n; j++)
			{
				if (i>0&& i<(n/2-2) &&  j>0 && j<n-1 && quadVector[i][j].s>quadVector[i-1][j].s &&quadVector[i][j].s>quadVector[i-1][j-1].s&& quadVector[i][j].s>quadVector[i-1][j+1].s&&
					quadVector[i][j].s>quadVector[i][j+1].s && quadVector[i][j].s>quadVector[i][j-1].s && quadVector[i][j].s>quadVector[i+1][j+1].s && quadVector[i][j].s>quadVector[i+1][j-1].s && quadVector[i][j].s>quadVector[i+1][j].s ||
					i==0 && j>0&& j<n-1 && quadVector[i][j].s>quadVector[i+1][j].s&& quadVector[i][j].s>quadVector[i][j+1].s&& quadVector[i][j].s>quadVector[i][j-1].s ||
					i==(n/2-2) && j>0&& j<n-1 && quadVector[i][j].s>quadVector[i-1][j].s&& quadVector[i][j].s>quadVector[i][j+1].s&& quadVector[i][j].s>quadVector[i][j-1].s ||
					i>0 && i<(n/2-2) && j==0 && quadVector[i][j].s>quadVector[i-1][j].s&& quadVector[i][j].s>quadVector[i][j+1].s&& quadVector[i][j].s>quadVector[i+1][j].s && quadVector[i][j].s>quadVector[i][n-1].s||
					i>0 && i<(n/2-2) && j==n-1 && quadVector[i][j].s>quadVector[i-1][j].s&& quadVector[i][j].s>quadVector[i][j-1].s&& quadVector[i][j].s>quadVector[i+1][j].s && quadVector[i][j].s>quadVector[i][0].s)
				{
					if (quadVector[i][j].s>max1)
					{   
						max1 =quadVector[i][j].s;
						ind1 =Point(i,j);
					}
				}
			}
			
			double x,y;
			x =(quadVector[ind1.x][ind1.y].sum_x/quadVector[ind1.x][ind1.y].s);
			y =(quadVector[ind1.x][ind1.y].sum_y/quadVector[ind1.x][ind1.y].s);
			cout<<"x "<<x<<" y "<<y<<endl;
			r.push_back(Point(x,y));
			cout<<"coord "<<ind1.x<<" "<<ind1.y<<endl;
			r.push_back(ind1);
			return r;
}
vector<Point> body_function (vector<Vec4i> lines, Mat cdst, int k, int k_num)
{
	
	cout<<"sizes of image "<<cdst.cols<<" "<<cdst.rows<<endl;
	Point pt3,pt2;
	vector<vector<double>> vv;
	vector<Point> a_begin,a_end,xy;
	for( size_t i = 0; i < lines.size(); i++ )
	{   
		Vec4i l = lines[i];
		if (k==k_num)
		line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
	}	
	imwrite("detected_body_func3.jpg", cdst);
	const int n=30;    // number of meridians
	const int nn =n*n; 
	struct sphere sph; 
	// centre coord (x,y) 
	sph.x =int( cdst.cols /2);
	sph.y =int( cdst.rows /2);
	sph.d = min(cdst.cols,cdst.rows);
	double a=1.5*PI;
	int ind;
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

			q.b = inters(sph,i,j+1,n);  
			q.c = inters(sph,i+1,j,n);
			q.d = inters(sph,i+1,j+1,n); 
			if (k==k_num)
			{
			//	circle(cdst,q.a,5, Scalar(0,255,0),5, 8, 0);
				line( cdst, q.a, q.b, Scalar( 150,0,150 ),  2, 8 );
				line( cdst, q.a, q.c, Scalar( 150,0,150 ),  2, 8 );
				line( cdst, q.d, q.c, Scalar( 150,0,150 ),  2, 8 );
				line( cdst, q.d, q.b, Scalar( 150,0,150 ),  2, 8 );
			}
			
			quadVector[i-1][j]=q;

		}
	}
	cout<<"number of lines in leader-cell "<<lines.size()<<endl;
	bool bbreak=0;
		for( auto i = lines.begin(); i!=lines.end(); i++)
		{
			Vec4i l = *i;     
			for(auto j =i+1; j != lines.end(); j++) 
			{
				Vec4i p = *j; 
				pt2=planeInters(Point(l[0],l[1]),Point(l[2],l[3]),Point(p[0],p[1]),Point(p[2],p[3])); // точка пересечения двух прямых в плоскости 
				bbreak=0;
				for (int i=0; i<n/2-1; i++) 
				{
					for (int j=0; j<n; j++)
					if (belong_to_quad(quadVector[i][j],pt2)) // checking  point belonging to the quadVector
					{
							quadVector[i][j].s++;
							quadVector[i][j].sum_x+=pt2.x;
							quadVector[i][j].sum_y+=pt2.y;	

							bbreak =1;
							break;					
					}	
					if (bbreak==1)
					break;
				}   
			}
		}
		imwrite("detected_body_func3.jpg", cdst);
		//maximum(s) search 
		//
		vector<double> v;
		vector<Point> r,r1;
		r=max_search(n, quadVector);

		if (k==1)
		{
			ofstream fout("cppstudioH50.txt"); // создаём объект класса ofstream для записи и связываем его с файлом cppstudio.txt
			for (int i=0; i<n/2-1; i++) 
			{
				for (int j=0; j<n; j++)
				{
					fout<<quadVector[i][j].s<<" ";
				}
				fout<<endl;
			}
			fout.close();
			return r;
		}
		else
		{
			vector<Vec4i> lines1;
			for( auto i = lines.begin(); i!=lines.end(); i++)
			{
				Vec4i l = *i;   
				bbreak=0;
				for(auto j =lines.begin(); j != lines.end(); j++) 
					if ( i!=j)
					{  
						Vec4i p = *j; 
						pt2=planeInters(Point(l[0],l[1]),Point(l[2],l[3]),Point(p[0],p[1]),Point(p[2],p[3])); // точка пересечения двух прямых в плоскости 
						if (belong_to_quad(quadVector[r[1].x][r[1].y],pt2)) // checking  point belonging to the quadVector
						{
							bbreak =1;
							xy.push_back(pt2);
							v=line_coord(Point(l[0],l[1]),Point(l[2],l[3]));  //сборка линий для вычисления точки схода
							vv.push_back(v);
						//	v=line_coord(Point(p[0],p[1]),Point(p[2],p[3]));  // не знаю, будут ли линии повторяться?
						//	vv.push_back(v);
							break;					
						}	
					}  
				if (bbreak==0)
					lines1.push_back(l);

				else
					if (k==4)
						line( cdst, Point(l[0],l[1]),Point(l[2],l[3]), Scalar( 255,0,0),  2, 4 );
					else if(k==3)
						line( cdst, Point(l[0],l[1]),Point(l[2],l[3]), Scalar( 255,255,0),  2, 4 );
						else
							line( cdst, Point(l[0],l[1]),Point(l[2],l[3]), Scalar( 0,215,255),  2, 4 );

			}
			cout<<"vv.size "<<vv.size()<<endl;
			pt3=VP(vv);
			cout<<"VP "<<pt3<<endl;
			pt2=centre(xy);
			cout<<"centre of cluster "<<pt2<<endl;
			imwrite("detected_body_func3.jpg", cdst);
			r1 =body_function( lines1, cdst,k-1,k_num);
		//	r1.push_back(pt3);
			r1.push_back(r[0]); r1.push_back(r[1]);
			if (k==2)
			{
				namedWindow("detected3",WINDOW_NORMAL);
				imshow("detected3", cdst);
				imwrite("detected_body_func3.jpg", cdst);
			}
			
			return r1;
		}
 

}

int main(int argc, char** argv)
{
	const char* filename = argc >1 ? argv[1] : "test2.jpg";
	Mat src = imread(filename,0);    
//	Mat src =image_return( filename);
	Mat dst, abs_dst,gray,cdst;
	GaussianBlur( src,abs_dst, Size(3,3), 1, 0, BORDER_DEFAULT );
	Laplacian(abs_dst, dst, CV_16S, 3, 1, 0, BORDER_DEFAULT );
	convertScaleAbs(dst, dst);
	threshold(dst,dst,70,180,THRESH_BINARY);
	imwrite("detected_body_func3.jpg", dst);
	cvtColor( dst, cdst, CV_GRAY2BGR); 
	vector<Vec4i> lines;
	HoughLinesP(dst, lines, 1, CV_PI/180, 50, 55, 5);
	
	vector<Point> r;
	r=body_function (lines, cdst,3,3);
	// convert matrix
	//CvMat* warp_matrix = cvCreateMat(3,3,CV_32FC1);
	Mat warp_matrix(3,3,CV_32FC1);
	IplImage *src1=0, *dst1=0;
	//src1 = cvLoadImage(filename,1);

	//		setting one element 
	//		void  cvmSet( CvMat* mat, int row, int col, double value );
	//   
	//  calculate the equation of vanishing line using cross product
	double x,y,x1,y1,a1,b,c;
	x=r[0].x; 
	y=r[0].y;
	x1=r[2].x; 
	y1=r[2].y;

	a1= (y-y1);
	b= (x1-x);
	c = x*y1 - x1*y;
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
	warp_matrix.at<float>(2,2) =  c; 

	//	cout<<ned<<" ned"<<endl;
	// convert prospects
	std::cout << warp_matrix << std::endl;
	warpPerspective(src, dst, warp_matrix, cv::Size(3000,3000));

	namedWindow("warpPerspective",WINDOW_NORMAL);
	imshow("warpPerspective", dst);
	waitKey();

	waitKey();
	return 0;
}
