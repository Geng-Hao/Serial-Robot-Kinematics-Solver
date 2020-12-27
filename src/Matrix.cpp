
#include"Matrix.h"
#include<iostream>

/*Copy Constructor*/


Matrix::Matrix(){

 for(int i= 0; i<n_Row; i++){
 
   for(int j = 0; j<n_Col; j++)
     *(*(mat+i)+j)=0;
 
 }
 
 *(*(mat+3)+3)=1;




}

/*  Matrix m(arr);  */
Matrix::Matrix(double m[][n_Col]){


 for(int i= 0; i<n_Row; i++){
 
   for(int j = 0; j<n_Col; j++)
     mat[i][j]=m[i][j];
 
 }


} //Constructor


/* Matrix m((double**)arr); */
Matrix::Matrix(double** m){


 for(int i= 0; i<n_Row; i++){
 
   for(int j = 0; j<n_Col; j++)
     mat[i][j]=*((double *)m +i*n_Col +j );
 
 }

} //Constructor


Matrix::Matrix(const Matrix& m) {


 for(int i= 0; i<n_Row; i++){
 
   for(int j = 0; j<n_Col; j++)
     mat[i][j]=m.mat[i][j];
 
 }


}




/*Set Function*/


void Matrix::set(int r, int c, double val) {

	if(r>=n_Row || c>=n_Col)
	 std::cout<<"Out of the boundary!"<<std::endl;
	else
	 *(*(this->mat+r) + c) = val;

}

void Matrix::setX(const Vec& X) {

	this->mat[0][0] = X.getX();
	this->mat[1][0] = X.getY();
	this->mat[2][0] = X.getZ();
}

void Matrix::setY(const Vec& Y) {

	this->mat[0][1] = Y.getX();
	this->mat[1][1] = Y.getY();
	this->mat[2][1] = Y.getZ();
}

void Matrix::setZ(const Vec& Z) {

	this->mat[0][2] = Z.getX();
	this->mat[1][2] = Z.getY();
	this->mat[2][2] = Z.getZ();

}

void Matrix::setP(const Vec& P) {

	this->mat[0][3] = P.getX();
	this->mat[1][3] = P.getY();
	this->mat[2][3] = P.getZ();

}


/*Get Function*/

double Matrix::get(int r,int c) const{ // row col
 
	return *(*(this->mat+r) + c); // r and c are not determined const, so mat[r][c] is illegal;

}


Vec Matrix::getX() const{
 
 	Vec p;
 	p.setX(this->mat[0][0]);
	p.setY(this->mat[1][0]);
 	p.setZ(this->mat[2][0]);

	return p;
}

Vec Matrix::getY() const{

	Vec p;
 	p.setX(this->mat[0][1]);
	p.setY(this->mat[1][1]);
 	p.setZ(this->mat[2][1]);

	return p;
}

Vec Matrix::getZ() const{

	Vec p;
 	p.setX(this->mat[0][2]);
	p.setY(this->mat[1][2]);
 	p.setZ(this->mat[2][2]);

	return p;
}

Vec Matrix::getP() const{

	Vec p;
 	p.setX(this->mat[0][3]);
	p.setY(this->mat[1][3]);
 	p.setZ(this->mat[2][3]);

	return p;
}


/*Print Function*/

void Matrix::print() const{


 for(int i=0; i<n_Row; i++){
 
  for(int j=0; j<n_Col; j++){
   std::cout<<(*this).get(i,j)<<" ";
  }
  std::cout<<std::endl;
 }


}



/*operator overloading*/

Matrix Matrix::operator+(const Matrix& a) {

	Matrix c;
	
	for(int i=0; i<n_Row; i++){
	  for(int j=0; j<n_Col; j++)
	   c.set(i,j,(this->mat[i][j]+a.mat[i][j]));
	}
	
	return c;


}


Matrix Matrix::operator-(const Matrix& a) {

	Matrix c;
	
	for(int i=0; i<n_Row; i++){
	  for(int j=0; j<n_Col; j++)
	   c.set(i,j,(this->mat[i][j]-a.mat[i][j]));
	}
	
	return c;

}

Matrix Matrix::operator*(double k) {

	Matrix c;

	for(int i=0; i<n_Row; i++){
	  for(int j=0; j<n_Col; j++)
	   c.set(i,j,(this->mat[i][j]*k));
	}
	
	return c;

}

Matrix Matrix::operator*(const Matrix& a) {

	Matrix c;
	double tmp;
	
	for(int i=0; i<n_Row; i++){
	
	 for(int j=0; j<n_Col; j++){
	  tmp = 0;
	  for(int k=0;k<n_Row;k++) tmp = tmp + (this->mat[i][k])*(a.mat[k][j]);
	  c.mat[i][j] = tmp;
	 
	 }
	
	}

	return c;

}

/*
Matrix& Matrix::operator<<(const Matrix& p) {
    
	for(int i= 0; i<n_Row; i++){
 
   		for(int j = 0; j<n_Col; j++){
     		this->mat[i][j]=p.mat[i][j];
 
 		}


	}

	return (*this);

}
*/

/*
bool Matrix::operator==(const Matrix& p) {
    
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

*/


