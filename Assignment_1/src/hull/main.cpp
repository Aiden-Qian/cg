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

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		// TODO
		if(det(p1,p2,p0)<0){
			return true;
		}
		else if(det(p1,p2,p0)==0 && ((p2.real()-p0.real())*(p2.real()-p0.real())+(p2.imag()-p0.imag())*(p2.imag()-p0.imag()))<((p1.real()-p0.real())*(p1.real()-p0.real())+(p1.imag()-p0.imag())*(p1.imag()-p0.imag()))){
			return true;
		}
		return false;
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	// TODO
	int val=(b.real()-a.real())*(c.imag()-a.imag())-(c.real()-a.real())*(b.imag()-a.imag());
	if(val>0) return true;
	return false;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	// TODO
	double dis;
	double low_dis=points[0].imag()*points[0].imag()+points[0].real()*points[0].real();;
	int index;
	for(int i=0;i<points.size();i++){
		dis=points[i].imag()*points[i].imag()+points[i].real()*points[i].real();
		if(dis<low_dis){
			low_dis=dis;
			index=i;
		}
	}
	order.p0 = points.at(index);//p0 is (158.068,413.998)

	Polygon hull;
    hull.push_back(order.p0);
	points.erase(points.begin()+index);
	
	std::sort(points.begin(), points.end(), order);
	// TODO
	hull.push_back(points.back());
	points.pop_back();

	while(!points.empty()){
		// use salientAngle(a, b, c) here
		while(salientAngle(hull.at(hull.size()-2),hull.back(),points.back())== 0 && hull.size()>1){
		    hull.pop_back();
			if (hull.size()==1){
				break;
			}
		}
		hull.push_back(points.back());
		points.pop_back();
	}
	
	return hull;
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

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}

/*There are some differences between cout and cerr. 
std::cerr is attached to the standard error device, which is also a display screen
*/
