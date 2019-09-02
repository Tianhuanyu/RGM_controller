// 三次插值函数，输入的是（t,theta）的一系列点
// 输出的是各个三次样条的二次偏导数
// 设计函数，输入时间t，输出对应的theta角

//组合化，变成六个角度
#ifndef KDL_SFUNCCUBE_HPP
#define KDL_SFUNCCUBE_HPP



#include "frames.hpp"

#include <vector>
#include <iostream>
#include <Eigen/Core>
#include "chain.hpp"
#include "chainiksolverpos_rgm.hpp"
#include "RGMtrajControl.hpp"




namespace KDL
{
    class Jointsfunc
    {
        public:
            Jointsfunc(const std::vector<Eigen::Vector2d> input_point);
            //输出三次曲线的插值
            double calculate_cube_theta(const double time);
            ~Jointsfunc();
        private:
            const std::vector<Eigen::Vector2d> _input_point;
            Eigen::VectorXd _para_m;
            Eigen::MatrixXd _para_H;
            Eigen::VectorXd _para_y;
            const int _point_number;

    };

    class chainsfunc
    {
        public:
            
            chainsfunc(const KDL::Chain& chain ,const std::vector<KDL::Frame> frames,const std::vector<double> times,const KDL::JntArray& q_init);
            JntArray calculate_cube_q(const double ttime);
            ~chainsfunc();
            std::vector<double> _times;

        
        private:
            Chain _chain;

            
            std::vector<Eigen::Vector2d> pt1;
            Jointsfunc* jc1;
            std::vector<Eigen::Vector2d> pt2;
            Jointsfunc* jc2;
            std::vector<Eigen::Vector2d> pt3;
            Jointsfunc* jc3;
            std::vector<Eigen::Vector2d> pt4;
            Jointsfunc* jc4;
            std::vector<Eigen::Vector2d> pt5;
            Jointsfunc* jc5;
            std::vector<Eigen::Vector2d> pt6;
            Jointsfunc* jc6;
            std::vector<KDL::Frame> _frames;
            int vector_count;

            
            ChainIkSolverPos_rgm iksolver;
    };


}

typedef KDL::chainsfunc chainsfunc;

static std::vector<chainsfunc *> chainsfunc_vector;


#ifdef __cplusplus
extern "C"{
#endif


void rgm_sfunc_init_wrap(int* handle,LinkQueue_pvt* pQueue,int32_t ap1,int32_t ap2,int32_t ap3,int32_t ap4,int32_t ap5,int32_t ap6);

int calculate_cube_q_wrap(int handle,const double ttime, int* Tp1, int* Tp2, int* Tp3, int* Tp4, int* Tp5, int* Tp6);

void rgm_fCtrl_dele_wrap(int* handle);

#ifdef __cplusplus
} 
#endif

#endif
