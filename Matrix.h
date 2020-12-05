#ifndef Matrix_H
#define Matrix_H

#include"stdlib.h"
#include"Vec.h"

#define n_Row 4
#define n_Col 4

using namespace std;

class Matrix {

private:

	double mat[n_Row][n_Col];

public:

	Matrix();
	Matrix(double [][n_Col]); //Constructor
 	Matrix(double**); //Constructor
	Matrix(const Matrix&); // Copy constructor



	/*Set Function*/

	void set(int, int, double); // row col val
	void setX(const Vec&);
	void setY(const Vec&);
	void setZ(const Vec&);
	void setP(const Vec&);
	
	/*Get Function*/
	double get(int,int) const; // row col
	Vec getX() const;
	Vec getY() const;
	Vec getZ() const;
	Vec getP() const;

        /*Print Function*/
        
        void print() const;

	/*operator overloading*/
	Matrix operator+(const Matrix&);
	Matrix operator-(const Matrix&);
	Matrix operator*(double);
	Matrix operator*(const Matrix&);
	//Matrix& operator<<(const Matrix&);
	//bool operator==(const Matrix&);
	
	/*Inverse Matrix*/
	
	//virtual Matrix inv(const Matrix&);
	
	
	
	

};






#endif
