
#include"Vec.h"
#include<iostream>
#include<cmath>




/*Copy Constructor*/

Vec::Vec(const Vec& p) {


	x = p.x;
	y = p.y;
	z = p.z;


}




/*Set Function*/


void Vec::setXYZ(double x, double y, double z) {

	this->x = x;
	this->y = y;
	this->z = z;

}

void Vec::setX(double x) {

	this->x = x;
}

void Vec::setY(double y) {

	this->y = y;
}

void Vec::setZ(double z) {

	this->z = z;

}


/*Get Function*/

double Vec::getX() const{

	return this->x;
}

double Vec::getY() const{

	return this->y;
}

double Vec::getZ() const{

	return this->z;
}


/*Print Function*/

void Vec::print() const{

  std::cout<<"("<<this->x<<","<<this->y<<","<<this->z<<")"<<std::endl;


}


/*operator overloading*/

Vec Vec::operator+(const Vec& a) const{

	Vec c;

	c.x = (*this).x + a.x;
	c.y = (*this).y + a.y;
	c.z = (*this).z + a.z;

	return c;


}


Vec Vec::operator-(const Vec& a) const{

	Vec c;

	c.x = (*this).x - a.x;
	c.y = (*this).y - a.y;
	c.z = (*this).z - a.z;

	return c;


}

Vec Vec::operator*(double d) const{

	Vec c;

	c.x = (*this).x * d;
	c.y = (*this).y * d;
	c.z = (*this).z * d;

	return c;



}


Vec Vec::operator/(double d) const{

	Vec c;

	c.x = (*this).x / d;
	c.y = (*this).y / d;
	c.z = (*this).z / d;

	return c;



}

/*
Vec Vec::operator=(const Vec& p) {
    
	Vec c(p);

	return c;

}
*/

bool Vec::operator==(const Vec& p) const{
    
	if(this->x==p.x){
	  if(this->y==p.y){
	   if(this->z==p.z)
	     return true;
	   else
	     return false;
	  }
	  else
	   return false;
	  
	
	}

	return false;

}

/*Vector Computation*/
	
double Vec::dot(const Vec& p) const{
	
  return (this->x*p.x+this->y*p.y+this->z*p.z);

}

Vec Vec::cross(const Vec& p) const{

  Vec V;
  V.x = this->y*p.z-this->z*p.y;
  V.y = this->z*p.x-this->x*p.z;
  V.z = this->x*p.y-this->y*p.x;
 
  return V;
}
	
double Vec::norm2() const{
  
  return sqrt(pow(this->x,2)+pow(this->y,2)+pow(this->z,2));

}

