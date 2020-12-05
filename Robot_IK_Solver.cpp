
#include"Robot_IK_Solver.h"

using namespace std;



/*Constructor*/

IK_Solver::IK_Solver() {

 for(int i=0; i<joint_number;i++){
 
  
  *(alpha+i) =0;
  *(a+i) =0;
  *(d+i) =0;
  *(theta+i) =0;
  *(l+i) =0;
  *(u+i) =0;
  
  
 }
 

}

IK_Solver::IK_Solver(double* alpha_ptr,double* a_ptr,double* d_ptr,double* theta_ptr,double* l_ptr, double* u_ptr) {

 for(int i=0; i<joint_number;i++){
  
  *(alpha+i) =*(alpha_ptr+i);
  *(a+i) =*(a_ptr+i);
  *(d+i) =*(d_ptr+i);
  *(theta+i) =*(theta_ptr+i);
  *(l+i) = *(l_ptr+i);
  *(u+i) =*(u_ptr+i);
  
 }
 

}


/*Private Function*/


Matrix IK_Solver::getT(int joint){

	double T_ptr[4][4] = {{cos(this->theta[joint-1]), -sin(this->theta[joint-1]), 0, this->a[joint-1]} , {sin(this->theta[joint-1])*cos(this->alpha[joint-1]),    cos(this->theta[joint-1])*cos(this->alpha[joint-1]), -sin(this->alpha[joint-1]), -sin(this->alpha[joint-1])*this->d[joint-1]} ,{sin(this->theta[joint-1])*sin(this->alpha[joint-1]), cos(this->theta[joint-1])*sin(this->alpha[joint-1]), cos(this->alpha[joint-1]), cos(this->alpha[joint-1])*this->d[joint-1]} ,{0,0,0,1}};
	
	Matrix T(T_ptr);
	
	//T.print();
	
	return T;

}


Vec IK_Solver::xaxis(int joint){

	Vec x;
	
	double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Matrix T(I);
	

	for(int i=0;i<joint;i++)
	 T = T*(this->getT(i+1));
	 
	x.setX(T.get(0,0));
	x.setY(T.get(1,0));
	x.setZ(T.get(2,0));
	
	double l = x.norm2();
	x.setX(x.getX()/l);
	x.setY(x.getY()/l);
	x.setZ(x.getZ()/l);
	
	
	return x;

}


Vec IK_Solver::yaxis(int joint){

	Vec y;
	
	double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Matrix T(I);
	

	for(int i=0;i<joint;i++)
	 T = T*(this->getT(i+1));
	 
	y.setX(T.get(0,1));
	y.setY(T.get(1,1));
	y.setZ(T.get(2,1));
	
	double l = y.norm2();
	y.setX(y.getX()/l);
	y.setY(y.getY()/l);
	y.setZ(y.getZ()/l);
	
	
	return y;

}


Vec IK_Solver::zaxis(int joint){

	Vec z;
	
	double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Matrix T(I);
	

	for(int i=0;i<joint;i++)
	 T = T*(this->getT(i+1));
	 
	z.setX(T.get(0,2));
	z.setY(T.get(1,2));
	z.setZ(T.get(2,2));
	
	double l = z.norm2();
	z.setX(z.getX()/l);
	z.setY(z.getY()/l);
	z.setZ(z.getZ()/l);
	
	
	return z;

}



Vec IK_Solver::joint_position(int joint){

	Vec p;
	
	double I[4][4] = {{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
	Matrix T(I);
	
	for(int i=0;i<joint;i++)
 	 T = T*(this->getT(i+1));
	
	
	
	p.setX(T.get(0,3));
	p.setY(T.get(1,3));
	p.setZ(T.get(2,3));
	
	return p;

}

void IK_Solver::CCD(const Vec& pd, const Vec& u1d, const Vec& u2d, const Vec& u3d, double* wp_ptr, double* wo_ptr){


        /* initial end-effector position */
	Vec ph = joint_position(joint_number); 
	
	/* initial end-effector orientation */
	Vec u1h = xaxis(joint_number); 
	Vec u2h = yaxis(joint_number);
	Vec u3h = zaxis(joint_number);
	
	Vec p_i, Pih, Pid;
	
	
	
	
	
	Vec Zi;
	double k1=0,k2=0,k3=0,g1=0,g2=0,g3=0,g=0;
	double phi1=0,phi2=0,phi3=0, phi=0;
	
	
	for(int i=(joint_number); i>=1;i--){
	
	 p_i = joint_position(i);
	 Pih = ph - p_i;
	 Pid = pd - p_i;
	 
	 u1h = xaxis(joint_number);
	 u2h = yaxis(joint_number);
	 u3h = zaxis(joint_number);
	 
	 Zi = zaxis(i);
	 k1 = (*wp_ptr)*(Pid.dot(Zi))*(Pih.dot(Zi))+(*wo_ptr)*(u1d.dot(Zi)*u1h.dot(Zi)+u2d.dot(Zi)*u2h.dot(Zi)+u3d.dot(Zi)*u3h.dot(Zi));
 	 k2 = Pid.dot(Pih)*(*wp_ptr)+(u1d.dot(u1h)+u2d.dot(u2h)+u3d.dot(u3h))*(*wo_ptr);
  	 k3 = Zi.dot(Pih.cross(Pid)*(*wp_ptr)+(u1h.cross(u1d)+u2h.cross(u2d)+u3h.cross(u3d))*(*wo_ptr));
  	 
  	 //------------------------------------------------
  	 phi1=atan(k3/(k2-k1)); //phi1
	//-------------------------------------------------

  	 phi2=phi1+PI; 

  	 phi3=phi1-PI; 
	 
	 
  	 g1=k1*(1-cos(phi1))+k2*cos(phi1)+k3*sin(phi1);
  	 g2=k1*(1-cos(phi2))+k2*cos(phi2)+k3*sin(phi2);
  	 g3=k1*(1-cos(phi3))+k2*cos(phi3)+k3*sin(phi3);
  	 
  	 /* find the g = max(g1,g2,g3)*/
  	 g = g1;
  	 if(g2>g) g = g2;
  	 if(g3>g) g = g3;
  	 
  	 if(g == g1)
  	  phi=phi1;
  	 else if(g == g2)
  	  phi=phi2;
  	 else if(g == g3)
  	  phi=phi3;
  	 else
  	  phi=0;
  	 
  	 if (((k1-k2)*cos(phi)-k3*sin(phi))<0 && phi<PI && phi>-PI){
  	 
  	 	if(this->theta[i-1]+phi<= this->l[i-1]) 
       		this->theta[i-1]= this->l[i-1];
   		else if(theta[i-1]+phi>= this->u[i-1]) 
      			this->theta[i-1]= this->u[i-1];
  		else
       		this->theta[i-1]=this->theta[i-1]+phi;
  	 
  	 }
  
    
       
         ph = joint_position(joint_number);
	
	}

}

/*Public Function*/

void IK_Solver::Inv_Kine(const Vec& pd, double alpha, double beta, double garmma, double delta, double wp, double wo){
 
  
  // initial end-effector position
  Vec ph = joint_position(joint_number); 
  
  // initial end-effector orientation
  Vec u1h = xaxis(joint_number);
  Vec u2h = yaxis(joint_number);
  Vec u3h = zaxis(joint_number);
   
	
  /*Compute the orientation vector from Euler angles*/
  
  Vec u1d, u2d, u3d;
  
  double rx[4][4] = {{1,0,0,0},{0,cos(alpha),-sin(alpha),0},{0,sin(alpha),cos(alpha),0},{0,0,0,1}};
  double ry[4][4] = {{cos(beta),0,sin(beta),0},{0,1,0,0},{-sin(beta),0,cos(beta),0},{0,0,0,1}};
  double rz[4][4] = {{cos(garmma),-sin(garmma),0,0},{sin(garmma),cos(garmma),0,0},{0,0,1,0},{0,0,0,1}};
	
  Matrix Rx(rx), Ry(ry), Rz(rz);
  Matrix Od = Rx*Ry*Rz;
  
  u1d = Od.getX(); 
  u2d = Od.getY();
  u3d = Od.getZ();

  
  /* Optimization Process */
  
  double cost;	
  
  printf("The CCD Algorithm is RUNNING......\n");
	
  do{
   CCD(pd,u1d,u2d,u3d,&wp,&wo);
   ph = joint_position(joint_number);
   
   u1h = xaxis(joint_number);
   u2h = yaxis(joint_number);
   u3h = zaxis(joint_number);
   
   cost = ((ph-pd).norm2()+pow(u1d.dot(u1h)-1,2)+pow(u2d.dot(u2h)-1,2)+pow(u3d.dot(u3h)-1,2));	
   
   printf("Current Cost:%lf\n",cost);
   
  }while(cost>delta);
  
  printf("Optimization Completed\n");

}

Vec IK_Solver::end_effector_position(double* theta_ptr){

 for(int i=0; i<joint_number;i++)
  *(this->theta+i) =*(theta_ptr+i);
  
 return this->joint_position(joint_number);

}

Vec IK_Solver::end_effector_orientationX(double* theta_ptr){

 for(int i=0; i<joint_number;i++)
  *(this->theta+i) =*(theta_ptr+i);
  
 return this->xaxis(joint_number);

}

Vec IK_Solver::end_effector_orientationY(double* theta_ptr){

 for(int i=0; i<joint_number;i++)
  *(this->theta+i) =*(theta_ptr+i);
  
 return this->yaxis(joint_number);

}

Vec IK_Solver::end_effector_orientationZ(double* theta_ptr){

 for(int i=0; i<joint_number;i++)
  *(this->theta+i) =*(theta_ptr+i);
  
 return this->zaxis(joint_number);

}

double* IK_Solver::getTheta(){

  
 return this->theta;

}


void IK_Solver::print(){

	
	cout<<"alpha:["<<this->alpha[0]<<","<<this->alpha[1]<<","<<this->alpha[2]<<","<<this->alpha[3]<<","<<this->alpha[4]<<","<<this->alpha[5]<<"]"<<endl;
	cout<<"a:["<<this->a[0]<<","<<this->a[1]<<","<<this->a[2]<<","<<this->a[3]<<","<<this->a[4]<<","<<this->a[5]<<"]"<<endl;
	cout<<"d:["<<this->d[0]<<","<<this->d[1]<<","<<this->d[2]<<","<<this->d[3]<<","<<this->d[4]<<","<<this->d[5]<<"]"<<endl;
	cout<<"theta:["<<this->theta[0]<<","<<this->theta[1]<<","<<this->theta[2]<<","<<this->theta[3]<<","<<this->theta[4]<<","<<this->theta[5]<<"]"<<endl;
	
}
