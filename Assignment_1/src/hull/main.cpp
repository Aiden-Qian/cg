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

double inline det(const Point &u, const Point &v) {
	// TODO
	return 0;
}

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		// TODO
		return true;
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	// TODO
	return false;
}

////////////////////////////////////////////////////////////////////////////////

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;
	// TODO
	order.p0 = Point(0, 0);
	std::sort(points.begin(), points.end(), order);
	Polygon hull;
	// TODO
	// use salientAngle(a, b, c) here
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
	char n;
    in.get(n);
	std::string s=" ";
	while(std::getline(in,s)){
		std::cout<<s<<std::endl;
		size_t pos=s.find(" ");
		double x=std::stod (s.substr(0,pos));
		s=s.substr(pos+1,s.size());
		pos=s.find(" ");
		double y=std::stod (s.substr(pos+1, pos));
		Point a(x,y);
		points.push_back(a);
	}

	return points;
}
/*
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
*/
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	//save_obj(argv[2], hull);
	return 0;
}

/*There are some differences between cout and cerr. 
std::cerr is attached to the standard error device, which is also a display screen
*/
