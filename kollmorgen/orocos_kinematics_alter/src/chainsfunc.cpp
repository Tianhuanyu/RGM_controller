
#include "chainsfunc.hpp"
#include <Eigen/LU>
#include "models.hpp"

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

void rgm_sfunc_init_wrap(int* handle,LinkQueue_pvt* pQueue,int32_t ap1,int32_t ap2,int32_t ap3,int32_t ap4,int32_t ap5,int32_t ap6){
     
    //chainsfunc* p =NULL;
    std::vector<KDL::Frame> vc_frames;
    std::vector<double> vc_times;
    KDL::Frame frame_temp;
    double x=0;
    double y=0;
    double z=0;

    double qx=0;
    double qy=0;
    double qz=0;
    double qw=0;

    while(pQueue->front!=NULL){
        //读取队头
        x = pQueue->front->tcp_frame.pos[0];
        y = pQueue->front->tcp_frame.pos[1];
        z = pQueue->front->tcp_frame.pos[2];
        //读取四元数
        qx = pQueue->front->tcp_frame.orientation[0];
        qy = pQueue->front->tcp_frame.orientation[1];
        qz = pQueue->front->tcp_frame.orientation[2];
        qw = pQueue->front->tcp_frame.orientation[3];

        //数据转换为KDL Frame形式，并进入队列
        KDL::Rotation r;
        r = r.Quaternion(qx,qy,qz,qw);
       vc_frames.push_back(KDL::Frame(r,KDL::Vector(x,y,z)));
       vc_times.push_back((double)(pQueue->front->Time*1.0));
        

        //读完去掉队头
        deletepvtQueue(pQueue);
    }

     

    //KDL::chainsfunc(vc_frames,vc_times)
    KDL::JntArray q_init(6);
    // q_init(5) = (double)(ap1  -AP1ZERO)/DRIVER_COUNT;
    // q_init(4) = (double)(ap2  -AP2ZERO)/DRIVER_COUNT;
    // q_init(3) = (double)(-ap3 +AP3ZERO)/DRIVER_COUNT;
    // q_init(2) = (double)(ap4  -AP4ZERO)/DRIVER_COUNT;
    // q_init(1) = (double)(-ap5 +AP5ZERO)/DRIVER_COUNT;
    // q_init(0) = (double)(ap6  -AP6ZERO)/DRIVER_COUNT;

    q_init(5) = (ap1  -AP1ZERO)/DRIVER_COUNT*2.0*M_PI ;
    q_init(4) = (ap2  -AP2ZERO)/DRIVER_COUNT*2.0*M_PI ;
    q_init(3) = (-ap3 +AP3ZERO)/DRIVER_COUNT*2.0*M_PI +M_PI;
    q_init(2) = (ap4  -AP4ZERO)/DRIVER_COUNT*2.0*M_PI ;
    q_init(1) = (-ap5 +AP5ZERO)/DRIVER_COUNT*2.0*M_PI +M_PI;
    q_init(0) = (ap6  -AP6ZERO)/DRIVER_COUNT*2.0*M_PI ;

    chainsfunc * p = NULL;
    KDL::Chain ur5 = KDL::universal_robot5();
    p = new chainsfunc(ur5,vc_frames,vc_times,q_init);
    
    chainsfunc_vector.push_back(p);
    
    *handle = 0;

}

int calculate_cube_q_wrap(int handle,const double ttime, int* Tp1, int* Tp2, int* Tp3, int* Tp4, int* Tp5, int* Tp6){
    double time_start =chainsfunc_vector[handle]->_times.front();
    double time_end =chainsfunc_vector[handle]->_times.back();
    
    if((ttime<=time_end)&&(time_start<= ttime )){

        KDL::JntArray q_output=chainsfunc_vector[handle]->calculate_cube_q(ttime);
        *Tp6 = (int)(AP6ZERO+ q_output(0)      *DRIVER_COUNT/(2.0*M_PI));
        *Tp5 = (int)(AP5ZERO-(q_output(1)-M_PI)*DRIVER_COUNT/(2.0*M_PI));
        *Tp4 = (int)(AP4ZERO+ q_output(2)      *DRIVER_COUNT/(2.0*M_PI));
        *Tp3 = (int)(AP3ZERO-(q_output(3)-M_PI)*DRIVER_COUNT/(2.0*M_PI));
        *Tp2 = (int)(AP2ZERO+ q_output(4)      *DRIVER_COUNT/(2.0*M_PI));
        *Tp1 = (int)(AP1ZERO+ q_output(5)      *DRIVER_COUNT/(2.0*M_PI));
            return 0;
    }
    else{
        std::cout<<"Exceed Time Queue! Return."<<std::endl;
        return -1;
    }
    
}


void rgm_fCtrl_dele_wrap(int* handle){
    chainsfunc_vector.clear();
    *handle = -1;
}