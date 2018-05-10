//
//  test.cpp
//  
//
//  Created by Minori Manabe on 2018/04/19.
//

#include "test.hpp"
#include "math.h"

// 1. マクロを定義．
#define EIGEN_NO_DEBUG // コード内のassertを無効化．
#define EIGEN_DONT_VECTORIZE // SIMDを無効化．
#define EIGEN_DONT_PARALLELIZE // 並列を無効化．
#define EIGEN_MPL2_ONLY // LGPLライセンスのコードを使わない．

// 2. Eigenをインクルード．
#include "iostream"
#include "Eigen/Core"
#include "Eigen/LU"
using namespace Eigen;

class inv_kinematic{
public:
    MatrixXd T06;
    float a1;
    float a2;
    float d4;
    
    MatrixXd Inv(){
        MatrixXd ph;
        
        float px = T06(3,0);
        float py = T06(3,1);
        float pz = T06(3,2);
        float r11 = T06(0,0);
        float r12 = T06(0,1);
        float r13 = T06(0,2);
        float r21 = T06(1,0);
        float r22 = T06(1,1);
        float r23 = T06(1,2);
        float r31 = T06(2,0);
        float r32 = T06(2,1);
        float r33 = T06(2,2);
        
        float c1 = cos(ph1);
        float s1 = sin(ph1);
        float c3 = cos(ph3);
        float s3 = sin(ph3);
        float c4 = cos(ph4);
        float s4 = sin(ph4);
        float c5 = cos(ph5);
        float s5 = sin(ph5);
        
        float c23 = cos(ph2 + ph3);
        float s23 = sin(ph2 + ph3)
        
        float ph1 = atan2(px,py);
        float ph3 = acos((c1*px+s1*py-a1)^2 + pz^2 - a2^2 - d4^2) / (2 * a2 * d4));
        float ph2 = atan2(-a2*s3*pz + (c1*px + s1*py - a1)*(a2*c3 + d4), -(a2*c3 + d4)*pz - a2*s3*(c1*px + s1*py - a1)) - ph3;
        float ph4 = atan2(-r13*s1 + r23*c1, r13*c1*c23 + r23*s1*c23 - r33*s23;
        float ph5 = atan2(r13*(c1*c23*c4 - s1*s4) + r23*(s1*c23*s4 + c1*s4) + r33*(-s23*c4), r13*(c1*s23) + r23*(s1*s23)+ r33*c23);
        float ph6 = atan2(-r11*(c1*c23*s4 + s1*c4) - r21*(s1*c23*s4 - c1*c4) + r31*s23*s4, r11*((c1*c23*c4 - s1*s4)*c5 - c1*s23*s5) + r21*((s1*c23*c4 + c1*s4)*c5 - s1*s23*s5) + r31*(-s23*c4*c5 - c23*s5))
                          
    
//    cout << r11 << "\t" << r12 << "\t" << r13 << "\t" << px << endl;
//    cout << r21 << "\t" << r22 << "\t" << r33 << "\t" << py << endl;
//    cout << r31 << "\t" << r32 << "\t" << r33 << "\t" << pz << endl;
//    cout << 0 << "\t" << 0 << "\t" << 0 << "\t" << 1 <<endl;
   //下のカンマオペレータの書き方の方が楽
                          ph<< ph1,ph2,ph3,ph4,ph5,ph6;
        
        return ph;
        
    }
};

int main()
{
    MatrixXd T;
    
    //T06,a1,a2,d4
    inv_kinematic inv1 = {0,M_PI/2,-M_PI/2,0,0,0,20,165,165};
    
    T = inv1.Inv();
    
    std::cout << T << std::endl;
}
