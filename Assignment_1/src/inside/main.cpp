////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v, const Point &p) {  //determinant
	// TODO
	double d=(u.real()-p.real())*(v.imag()-p.imag())-(u.imag()-p.imag())*(v.real()-p.real());
	return d;
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d, Point &ans) {
	// TODO
	double h1,h2;
	double dis_a_b=(b.real()-a.real())*(b.real()-a.real())+(b.imag()-a.imag())*(b.imag()-a.imag());

	if(d.imag()/d.real()==b.imag()/b.real() ||d.imag()/d.real()==a.imag()/a.real()) return false;//To know if the intersect is on the vertex

	if(det(c,d,a)*det(c,d,b)<0 && det(a,b,c)*det(a,b,d)<0){//intersect
	    h1=sqrt(det(d,b,a)*det(d,b,a)/dis_a_b);
		h2=sqrt(det(c,b,a)*det(c,b,a)/dis_a_b);
        double x=(1-h1/(h1+h2))*(d.real()-c.real())+c.real();
		double y=(1-h1/(h1+h2))*(d.imag()-c.imag())+c.imag();
		ans= std::complex<double> (x,y);
		return true;
	}
	return false;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	// TODO
	Point outside(0, 0);
	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	// TODO
	int intersects;
	Point ans;
	for(int i=0;i<poly.size();i++){
		if(i==poly.size()-1){
			if(intersect_segment(poly.front(),poly.back(),outside,query,ans)){
				++intersects;
			}
		}
		else{
			if(intersect_segment(poly.at(i),poly.at(i+1),outside,query,ans)){
				++intersects;
			}
		}
	}
	if(intersects%2==0) return false;
	return true;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	// TODO
	if(!in.is_open()){
		std::cerr<<"Error opening file"<<std::endl;
	}
	std::string dummyline;
	std::getline(in,dummyline);

	std::string s=" ";
	while(std::getline(in,s)){
		size_t pos=s.find(" ");
		double x=std::stod (s.substr(0,pos));
		s=s.substr(pos+1,s.size());
		pos=s.find(" ");
		double y=std::stod (s.substr(0,pos));
		Point p(x,y);
		points.push_back(p);
	}
	return points;
}

Polygon load_obj(const std::string &filename) {
	std::ifstream in(filename);
	// TODO
	Polygon obj;
	std::vector<Point> temp;
	std::string s=" ";
	while(std::getline(in,s)){
		size_t pos=s.find(" ");
		std::string ele=s.substr(0,pos);
		if(ele=="v"){
			s=s.substr(pos+1,s.size());
		    pos=s.find(" ");
		    double x=std::stod (s.substr(0,pos));
			s=s.substr(pos+1,s.size());
		    pos=s.find(" ");
			double y=std::stod (s.substr(0,pos));
			Point p(x,y);
			temp.push_back(p);
		}
		else if(ele=="f"){
			while(pos!=s.npos){
				s=s.substr(pos+1,s.size());
				pos=s.find(" ");
		        double n=std::stod (s.substr(0,pos));
				obj.push_back(temp.at(n-1));
			}		
		}
	}
	return obj;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	// TODO
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	out<<points.size()<<std::endl;
	for (const auto &v : points) {
		out << v.real() << ' ' << v.imag() << " 0\n";
	}
	/*
	for (size_t i = 0; i < points.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%points.size() << "\n";
	}*/
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
