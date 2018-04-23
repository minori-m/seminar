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
using Eigen::MatrixXd;

class trans{
public:
    double alpha;
    double a;
    double d;
    double ph;
    
    MatrixXd T(){
        MatrixXd m;
        m =  MatrixXd::Zero(4,4);
        //std::cout << m << std::endl;
        m(0,0) = cos(ph);
        m(0,1) = -sin(ph);
        m(0,3) = a;
        
        m(1,0) = sin(ph) * cos(alpha);
        m(1,1) = cos(ph) * cos(alpha);
        m(1,2) = -sin(alpha);
        m(1,3) = -sin(alpha) * d;
        
        m(2,0) = sin(ph) * sin(alpha);
        m(2,1) = cos(ph) * sin(alpha);
        m(2,2) = cos(alpha);
        m(2,3) = cos(alpha)*d;
        
        m(3,3) = 1;
        
        return m;
    }
};


int main()
{
//
    trans trans1 = {1,1,1,1};
    trans trans2 = {-M_PI/2,1,0,0};
    trans trans3 = {0,0,0,0};
    trans trans4 = {0,0,0,0};
    trans trans5 = {0,0,0,0};
    trans trans6 = {0,0,0,0};
    
    MatrixXd t,t1,t2,t3,t4,t5,t6;
    t1 = trans1.T();
    t2 = trans2.T();
    t3 = trans3.T();
    t4 = trans4.T();
    t5 = trans5.T();
    t6 = trans6.T();
    
    
    t = t1 * t2 * t3 * t4 * t5 * t6;
    
    std::cout << t << std::endl;
    //std::cout << t2 << std::endl;
}
