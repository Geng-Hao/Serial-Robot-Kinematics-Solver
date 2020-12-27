#ifndef Vec_H
#define Vec_H

#include"stdlib.h"


class Vec {

private:
	double x;
	double y;
	double z;

public:

	Vec() : x(0), y(0), z(0) {};
	Vec(double x, double y, double z) : x(x), y(y), z(z) {}; //Constructor
	Vec(const Vec&); // Copy constructor



	/*Set Function*/

	void setXYZ(double, double, double);
	void setX(double);
	void setY(double);
	void setZ(double);
	
	/*Get Function*/
	double getX() const;
	double getY() const;
	double getZ() const;


	/*Print Function*/
	
	void print() const;

	/*operator overloading*/
	Vec operator+ (const Vec&) const;
	Vec operator-(const Vec&) const;
	Vec operator*(double) const;
	Vec operator/(double) const;

	
	bool operator==(const Vec&) const;
	
	/*Vector Computation*/
	
	double dot(const Vec&) const;
	Vec cross(const Vec&) const; 
	double norm2() const;
	
	

};






#endif
