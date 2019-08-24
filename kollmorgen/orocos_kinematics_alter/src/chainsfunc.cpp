
#include "chainsfunc.hpp"
#include <Eigen/LU>

namespace KDL{

Jointsfunc::Jointsfunc(const std::vector<Eigen::Vector2d> input_point):
    _input_point(input_point),
    _point_number(input_point.size())
    {
        if (_point_number < 2){
            std::cout<<"Number not enough!"<<std::endl;
            return;
        }

        _para_H.resize(_point_number,_point_number);
        _para_y.resize(_point_number);

        _para_H = Eigen::MatrixXd::Zero(_point_number,_point_number);
        _para_y = Eigen::VectorXd::Zero(_point_number);

        
        //搞定几个矩阵
        int iter = 0;
        for(iter=0 ;iter < _point_number; iter++){
            //利用一个循环，求出一个向量和一个矩阵
            if((iter == 0)||(iter == _point_number-1)){
                //矩阵的第一行,最后一行 对应编写，向量的第一行，最后一行 对应编写
                //std::cout<<"123123123"<<std::endl;
                _para_H(iter,iter) = 1;
                _para_y(iter)=0;
            }
            else{
                //std::cout<<"45456456"<<std::endl;
                double h_temp = _input_point.at(iter+1)(0)-_input_point.at(iter)(0);
                //std::cout<<"3"<<std::endl;
                double h_des = _input_point.at(iter)(0)-_input_point.at(iter-1)(0);
                //std::cout<<"4"<<std::endl;
                _para_H(iter,iter)= 2*(h_temp+h_des);
                _para_H(iter,iter-1) = h_des;
                _para_H(iter,iter+1) = h_temp;

                _para_y(iter)= (_input_point.at(iter+1)(1)-_input_point.at(iter)(1))/h_temp-(_input_point.at(iter)(1)-_input_point.at(iter-1)(1))/h_des;
            }
        }

        _para_m.resize(1+_point_number);
        _para_m = Eigen::VectorXd::Zero(1+_point_number);

        _para_m = _para_H.lu().solve(6*_para_y);
        

    }

    double Jointsfunc::calculate_cube_theta(const double time){
        int insert_number = 0;
        if(time <= _input_point.at(0)(0)){
            return _input_point.at(0)(1);
        }
        else if(time >= _input_point.at(_point_number-1)(0)){
            return _input_point.at(_point_number-1)(1);
        }
        else{
            for(insert_number=0;insert_number<_point_number-2;++insert_number){
                if((time > _input_point.at(insert_number)(0))&&(time < _input_point.at(insert_number+1)(0)))
                    break;
            }
        }
        
        double hi = _input_point.at(insert_number+1)(0)-_input_point.at(insert_number)(0);
        double mi = _para_m(insert_number);
        double mip1 = _para_m(insert_number+1);
        double yi = _input_point.at(insert_number)(1);
        double yip1 = _input_point.at(insert_number+1)(1);
        double a =_input_point.at(insert_number)(1);
        double b =(yip1-yi)/hi-hi/2*mi-hi/6*(mip1-mi);
        double c= mi/2;
        double d =(mip1-mi)/(6*hi);
        double ti = _input_point.at(insert_number)(0);
        //std::cout<<"ti = "<<(ti)<<" b = "<<b<<" c="<<c<<" d="<<d<<std::endl;

        return a+b*(time-ti)+c*(time-ti)*(time-ti)+d*(time-ti)*(time-ti)*(time-ti);


    }

    Jointsfunc::~Jointsfunc(){
        
    }


    chainsfunc::chainsfunc(const KDL::Chain& chain ,const std::vector<KDL::Frame> frames,const std::vector<double> times,const KDL::JntArray& q_init):
    _frames(frames),
    _chain(chain),
    iksolver(_chain),
    vector_count(_frames.size()),
    _times(times)
    {
        if(_times.size()!= _frames.size())
        {
            std::cout<<"Error frames and times should have same size!!"<<std::endl;
            return;
        }
        if(q_init.rows()!= 6 ){
            std::cout<<"Error q should have 6 dimination!!"<<std::endl;
            return;
        }

        // 通过逆运动学，将frames 转换为关节角
        KDL::JntArray q_init_iter = JntArray(q_init);
        KDL::JntArray q_output(6);
        int iter = 0;
        Eigen::Vector2d v1_temp(0,0);
        Eigen::Vector2d v2_temp(0,0);
        Eigen::Vector2d v3_temp(0,0);
        Eigen::Vector2d v4_temp(0,0);
        Eigen::Vector2d v5_temp(0,0);
        Eigen::Vector2d v6_temp(0,0);
        std::cout<<"run to here 111"<<std::endl;
        double a;

        for(iter=0;iter <vector_count;iter++ ){

            //
            iksolver.CartToJnt(q_init_iter,_frames.at(iter),q_output);
            q_init_iter = q_output;
            //将关节角转换为theta 队列
           
            v1_temp(0) = _times.at(iter); 
            
            v1_temp(1) = q_output(0);
            
            pt1.push_back(v1_temp);
            v2_temp(0) = _times.at(iter); v2_temp(1) = q_output(1);
            pt2.push_back(v2_temp);
            v3_temp(0) = _times.at(iter); v3_temp(1) = q_output(2);
            pt3.push_back(v3_temp);
            v4_temp(0) = _times.at(iter); v4_temp(1) = q_output(3);
            pt4.push_back(v4_temp);
            v5_temp(0) = _times.at(iter); v5_temp(1) = q_output(4);
            pt5.push_back(v5_temp);
            v6_temp(0) = _times.at(iter); v6_temp(1) = q_output(5);
            pt6.push_back(v6_temp);
            

        }
        
        std::cout<<"run to here 222"<<std::endl;
        //初始化6个jc,生成6组中间点
        jc1 = new Jointsfunc(pt1);
        jc2 = new Jointsfunc(pt2);
        jc3 = new Jointsfunc(pt3);
        jc4 = new Jointsfunc(pt4);
        jc5 = new Jointsfunc(pt5);
        jc6 = new Jointsfunc(pt6);
        

    }

    JntArray chainsfunc::calculate_cube_q(const double ttime){
        JntArray q(6);
        q(0) = jc1->calculate_cube_theta(ttime);
        q(1) = jc2->calculate_cube_theta(ttime);
        q(2) = jc3->calculate_cube_theta(ttime);
        q(3) = jc4->calculate_cube_theta(ttime);
        q(4) = jc5->calculate_cube_theta(ttime);
        q(5) = jc6->calculate_cube_theta(ttime);

        return q;
    }


    chainsfunc::~chainsfunc(){
        delete jc1;
        delete jc2;
        delete jc3;
        delete jc4;
        delete jc5;
        delete jc6;
    }





}