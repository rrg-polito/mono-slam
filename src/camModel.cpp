#include "camModel.hpp"


void CamModel::setParam(camConfig camParams) {
	fx = camParams.fx;
	fy = camParams.fy;
	u0 = camParams.u0;
	v0 = camParams.v0;
	k1 = camParams.k1;
	k2 = camParams.k2;
	k3 = camParams.k3;
	p1 = camParams.p1;
	p2 = camParams.p2;
	
}


Matrix2f CamModel::diff_distort_undistort(Vector2f hn) {


//  to compute jacobian of distoration and inverse if undistoration
//  matched with sympy[OK]
//  Jacobian of normalized hn to the function of distoration 

	const float r_2 = hn(0)*hn(0) + hn(1)*hn(1);
	const float Lrn = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;

	Vector2f hn_contr;
    hn_contr << hn(1),hn(0);

    Vector2f pv;
    pv << p1,p2;

    Vector2f pv_contr;
    pv_contr << p2,p1;

    Matrix2f j_matrix;
    j_matrix << p2*hn(0), 0, 0, p1*hn(1);

    const float f = k1 + 2*k2*r_2 + 3*k3*r_2*r_2;

    Matrix2f Jacobian_Distort_Undistort = Lrn*Matrix2f::Identity() + 2*f*hn*hn.transpose() +
                                          2*pv*hn_contr.transpose() + 2*pv_contr*hn.transpose() +
                                          4*j_matrix;
    return Jacobian_Distort_Undistort;

}



CamModel::CamModel(float _fx, float _fy, float _u0, float _v0, float _k1, float _k2, float _k3, float _p1, float _p2):
        fx(_fx),
        fy(_fy),
        u0(_u0),
        v0(_v0),
        k1(_k1),
        k2(_k2),
        k3(_k3),
        p1(_p1),
        p2(_p2)
    {
    

    }
    


Vector2f CamModel::projectAndDistort(Vector3f h, MatrixXf &J) {

    // Note: here f is 1 and used fx, fy = 1/dx;1/dy 
    float x = h(0);
    float y = h(1);
    float z = h(2);
    
    // Compute point retina undistor
    float inv_z = 1/z;

    float x1 = x/z;
    float y1 = y/z;
    
    MatrixXf Jacobian_normalized2_3d(2,3);
    Jacobian_normalized2_3d << 1/z, 0, -x/z/z, 0, 1/z, -y/z/z;

    // same as -x/(z**2) 
    // Compute point retina distort

    float r_2 = x1*x1+y1*y1;
    float l = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;

    
    float x2 = x1*l + 2*p1*x1*y1 + p2*(r_2 + 2*x1*x1);
    float y2 = y1*l + 2*p2*x1*y1 + p1*(r_2 + 2*y1*y1);


    Vector2f hn;
    hn << x1,y1;

    Vector2f hd;
    hd << fx*x2 + u0, fy*y2 + v0; 

    MatrixXf Jacobian_Pixel_Retina_normalized(2,2);
    Jacobian_Pixel_Retina_normalized << fx, 0, 0, fy;
    
    
    // ************************************ //
    // jacobian of distoration multiply with jacob of project??
    // TO SEARCH?    
    J = Jacobian_Pixel_Retina_normalized*diff_distort_undistort(hn)*Jacobian_normalized2_3d;
    
    return hd;
}

Vector2f CamModel::projectAndDistort(Vector3f h) {
    

    float x = h(0);
    float y = h(1);
    float z = h(2);
    
    
    // Compute point retina undistor
    float inv_z = 1/z;

    float x1 = x/z;
    float y1 = y/z;



    
    // Compute point retina distort
    // same as matlab and opencv

    float r_2 = x1*x1+y1*y1;
    float l = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;

    
    float x2 = x1*l + 2*p1*x1*y1 + p2*(r_2 + 2*x1*x1);
    float y2 = y1*l + 2*p2*x1*y1 + p1*(r_2 + 2*y1*y1);

    Vector2f hd;
    hd << fx*x2 + u0, fy*y2 + v0;

    return hd;
}

Vector3f CamModel::UndistortAndDeproject(Vector2f hd, MatrixXf &J){
    
    // Compute retina distort
    float x2 = (hd(0) - u0)/fx;
    float y2 = (hd(1) - v0)/fy;

    MatrixXf Jacobian_normalizedpoint_to_point(2,2);
    Jacobian_normalizedpoint_to_point << 1/fx, 0, 0, 1/fy;

    // Compute retina undistort
    
    
    
    float x1 = x2;
    float y1 = y2;

    float r_2;
    float l;

    float dx,dy;


    int n_iter = 50;
    // a param
    // foe distorate correction
    for (int i = 0; i < n_iter; ++i) {
    	r_2 = x1*x1 + y1*y1;
    	l = 1 + k1*r_2 + k2*r_2*r_2 + k3*r_2*r_2*r_2;

        dx = 2*p1*x1*y1 + p2*(r_2 + 2*x1*x1);
        dy = 2*p2*x1*y1 + p1*(r_2 + 2*y1*y1);

        x1 = (x2 - dx)/l;
        y1 = (y2 - dy)/l;
    }


	
    Vector2f hn;
    hn << x1,y1;
    
    Vector3f hC;
    hC << x1, y1, 1;
    
    MatrixXf Jacobian_Unproject_to_point(3,2);
    Jacobian_Unproject_to_point << 1, 0, 0, 1, 0, 0;
 
    // jacobian of undistoration multiply with jacob of deproject??
    // TO SEARCH?   
    J = Jacobian_Unproject_to_point*diff_distort_undistort(hn).inverse()*Jacobian_normalizedpoint_to_point;

    return hC;
}



