#include "stdlib.h"
#include <iostream>
#include "Robot_IK_Solver.h"


 using namespace std;

int main(){
 
 
 /*DH Parame*/
 double alpha[6] = {0,-PI/2,0,-PI/2,PI/2,-PI/2};
 double a[6] = {0,0,431.8,19,0,0};
 double d[6] = {0,0,125,432,0,0};
 double theta[6] = {0,0,0,0,0,0};
 double l[6] = {-10,-10,-10,-10,-10,-10};
 double u[6] = {10,10,10,10,10,10};

 IK_Solver Puma560(alpha,a,d,theta,l,u);
 
 Puma560.print();
 Vec p = Puma560.end_effector_position(theta); 
 Vec Ox = Puma560.end_effector_orientationX(Puma560.getTheta()); 
 Vec Oy = Puma560.end_effector_orientationY(Puma560.getTheta()); 
 Vec Oz = Puma560.end_effector_orientationZ(Puma560.getTheta()); 
 cout<<"Current end-effector position:"; p.print();
 cout<<"Current end-effector xaxis:"; Ox.print();
 cout<<"Current end-effector yaxis:"; Oy.print();
 cout<<"Current end-effector zaxis:"; Oz.print();
 
 cout<<"Current end-effector position:"; p.print();
 
 
 double x,y,z;
 cout<<endl<<endl<<"Please enter the desired position."<<endl;
 cout<<"x:"; cin>>x;
 cout<<"y:"; cin>>y;
 cout<<"z:"; cin>>z;
 cout<<endl<<"============================"<<endl;
 Vec pd(x,y,z); 
 
 Puma560.Inv_Kine(pd,-PI,0,0,0.000001,1,100);
 cout<<endl;
 Puma560.print();
 p = Puma560.end_effector_position(Puma560.getTheta()); 
 Ox = Puma560.end_effector_orientationX(Puma560.getTheta()); 
 Oy = Puma560.end_effector_orientationY(Puma560.getTheta()); 
 Oz = Puma560.end_effector_orientationZ(Puma560.getTheta()); 
 cout<<endl<<"Current end-effector position:"; p.print();
 cout<<"Current end-effector xaxis:"; Ox.print();
 cout<<"Current end-effector yaxis:"; Oy.print();
 cout<<"Current end-effector zaxis:"; Oz.print();

 return 0;

}
