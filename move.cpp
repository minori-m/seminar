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

class kinematic{
public:
    float p1;
    float p2;
    float p3;
    float p4;
    float p5;
    float p6;
    float d4;
    float a1;
    float a2;

    
    MatrixXd Kin(){
        MatrixXd m;
        m =  MatrixXd::Zero(4,4);
        
        float r11 = cos(p1)*(cos(p2+p3)*(cos(p4)*cos(p5)*cos(p6)-sin(p4)*sin(p6))-sin(p2+p3)*sin(p5)*cos(p6))-sin(p1)*(sin(p4)*cos(p5)*cos(p6)+cos(p4)*sin(p6));
    float r21 = sin(p1)*(cos(p2+p3)*(cos(p4)*cos(p5)*cos(p6)-sin(p4)*sin(p6))-sin(p2+p3)*sin(p5)*cos(p6))+cos(p1)*(sin(p4)*cos(p5)*cos(p6)+cos(p4)*sin(p6));
    float r31 = -sin(p2+p3)*(cos(p4)*cos(p5)*cos(p6)-sin(p4)*sin(p6))-cos(p2+p3)*sin(p5)*cos(p6);
    float r12 = cos(p1)*(-cos(p2+p3)*(cos(p4)*cos(p5)*sin(p6)+sin(p4)*cos(p6))+sin(p2+p3)*sin(p5)*sin(p6))-sin(p1)*(-sin(p4)*cos(p5)*sin(p6)+cos(p4)*cos(p6));
    float r22 = sin(p1)*(-cos(p2+p3)*(cos(p4)*cos(p5)*sin(p6)+sin(p4)*cos(p6))+sin(p2+p3)*sin(p5)*sin(p6))+cos(p1)*(-sin(p4)*cos(p5)*sin(p6)+cos(p4)*cos(p6));
    float r32 = sin(p2+p3)*(cos(p4)*cos(p5)*sin(p6)+sin(p4)*cos(p6))+cos(p2+p3)*sin(p5)*sin(p6);
    float r13 = cos(p1)*(cos(p2+p3)*cos(p4)*sin(p5)+sin(p2+p3)*cos(p5))-sin(p1)*sin(p4)*sin(p5);
    float r23 = sin(p1)*(cos(p2+p3)*cos(p4)*sin(p5)+sin(p2+p3)*cos(p5))+cos(p1)*sin(p4)*sin(p5);
    float r33 = -sin(p2+p3)*cos(p4)*sin(p5)+cos(p2+p3)*cos(p5);
    float px = cos(p1)*(sin(p2+p3)*d4+sin(p2)*a2+a1);
    float py = sin(p1)*(sin(p2+p3)*d4+sin(p2)*a2+a1);
    float pz = cos(p2+p3)*d4+cos(p2)*a2;
//    cout << r11 << "\t" << r12 << "\t" << r13 << "\t" << px << endl;
//    cout << r21 << "\t" << r22 << "\t" << r33 << "\t" << py << endl;
//    cout << r31 << "\t" << r32 << "\t" << r33 << "\t" << pz << endl;
//    cout << 0 << "\t" << 0 << "\t" << 0 << "\t" << 1 <<endl;
   //下のカンマオペレータの書き方の方が楽
        m<< r11,r12,r13,px,
        r21,r22,r23,py,
        r31,r32,r33,pz,
        0,0,0,1;
        
        return m;
        
    }
};

int main()
{
    MatrixXd T;
    Vector4d P0;
    //x,y,z,1
    P0 << 0,0,0,1;
    //p1,p2,p3,...,p6,d4,a1,a2
    kinematic kin1 = {0,M_PI/2,-M_PI/2,0,0,0,20,165,165};
    
    T = kin1.Kin();
    
    std::cout << T << std::endl;
    std::cout << T*P0 << std::endl;
    
    //std::cout << t2 << std::endl;
}
