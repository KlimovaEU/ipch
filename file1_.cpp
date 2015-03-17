#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>
#include <iostream>
#include <cmath>
#define PI 3.14159265

using namespace cv;
using namespace std;
struct sphere 
{ 
	int x,y,d;
};

struct quad 
{
	Point a, b, c, d; 
	int s; 
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
	s1 = squareTriangle(q.a, q.b, q.c)+squareTriangle(q.a, q.b,q.d);
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
	sph.d = 1000;
	double a=1.5*PI;
	int step =50;
	int ind;
	vector<quad> quadVector;
	vector<int> index_list,i_l;
	vector<vector<int>> coord;
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
			q.a = inters(sph,i,j,n);  
			q.b = inters(sph,i,j+1,n);  
			q.c = inters(sph,i+1,j,n);
			q.d = inters(sph,i+1,j+1,n);  
			quadVector.push_back(q);
		}
	}
	for( size_t i = 0; i <quadVector.size(); i++)
	{
		if (quadVector[i].a.y <0)
			i_l.push_back(i);
		if (quadVector[i].a.y>cdst.rows)
			index_list.push_back(i);		
	}
	coord.push_back(i_l);
	i_l.clear();
	for (int k = 1; k<cdst.rows/step; k++)
	{
		for( size_t i = 0; i <quadVector.size(); i++)
		{
			if (quadVector[i].a.y <k*step && quadVector[i].a.y>(k-1)*step)
				i_l.push_back(i);		
		}
		coord.push_back(i_l);
		i_l.clear();
	}
	coord.push_back(index_list);
	index_list.clear();
	//dst.release();
	//
	//поиск пересечений
	//
	
	for( auto i = lines.begin(); i!=lines.end(); i++)
	{
			Vec4i l = *i;     
			for(auto j =i+1; j != lines.end(); j++) 
			{
				Vec4i p = *j;
				pt2=planeInters(Point(l[0],l[1]),Point(l[2],l[3]),Point(p[0],p[1]),Point(p[2],p[3])); // точка пересечения двух прямых в плоскости 
				
				if (pt2.y<0)
					ind = 0;
				else 
					if (pt2.y>cdst.rows)
						ind=coord.size()-1;
					else ind =ceil((double( pt2.y/step)));
					i_l=coord[ind];
				for (size_t k= 0; k <i_l.size(); k++) 
				{
					if (belong_to_quad(quadVector[i_l[k]],pt2)) // проверка принадлежности точки четырехугольнику
					{
							quadVector[i_l[k]].s++;
							break;
					}
				}
			 }               
	}
	//поиск максимумов
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

	cout<<"intersect_number  coord_x   coord_y"<<endl;
	cout<<max1<<" "<<(quadVector[ind1].a.x+quadVector[ind1].b.x+quadVector[ind1].c.x +quadVector[ind1].d.x)/4<<" "<<(quadVector[ind1].a.y+quadVector[ind1].b.y+quadVector[ind1].c.y +quadVector[ind1].d.y)/4 <<endl;
	cout<<max2<<" "<<(quadVector[ind2].a.x+quadVector[ind2].b.x+quadVector[ind2].c.x +quadVector[ind2].d.x)/4<<" "<<(quadVector[ind2].a.y+quadVector[ind2].b.y+quadVector[ind2].c.y +quadVector[ind2].d.y)/4 <<endl;
	cout<<max3<<" "<<(quadVector[ind3].a.x+quadVector[ind3].b.x+quadVector[ind3].c.x +quadVector[ind3].d.x)/4<<" "<<(quadVector[ind3].a.y+quadVector[ind3].b.y+quadVector[ind3].c.y +quadVector[ind3].d.y)/4 <<endl;

	namedWindow("detected lines",WINDOW_NORMAL);
	imshow("detected lines", cdst);
	imwrite("detected lines.jpg", cdst);
	waitKey();
	return 0;
}