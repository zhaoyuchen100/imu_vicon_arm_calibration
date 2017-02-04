#include <include/imu_vicon_arm_calibration_gui/mainwindow.h>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    joint_state_pub = false;
    stop = false;
    start_white_buffer = false;
    start_black_buffer = false;
    start_orange_buffer = false;
    capture_global_frame_w = false;
    capture_global_frame_b = false;
    capture_global_frame_o = false;
    init();
    ros_timer = new QTimer;
    connect(ros_timer, SIGNAL(timeout()),this,SLOT(on_ros_spin()),Qt::UniqueConnection);
    ros_timer->start(1);
    frame_count_v_w = 0;
    frame_count_v_b = 0;
    frame_count_v_o = 0;
}

MainWindow::~MainWindow()
{
    delete ui;
    delete sync;
    delete sync_v_o;
    delete sync_v_w;
    delete sync_v_b;
    delete ros_timer;
}

void MainWindow::on_ros_spin()
{
    ros::spinOnce();
}

void MainWindow::init()
{
    sub_black.subscribe(n_,"myo_black_imu",10);
    sub_white.subscribe(n_,"myo_white_imu",10);
    sub_orange.subscribe(n_,"orange_imu",10);
    sub_vicon_w.subscribe(n_,"vicon/myo_w/myo_w",10);
    sub_vicon_b.subscribe(n_,"vicon/myo_b/myo_b",10);
    sub_vicon_o.subscribe(n_,"vicon/hand/hand",10);
    sync = new Synchronizer<MySyncPolicy>(MySyncPolicy(10), sub_black, sub_white, sub_orange);
    sync_v_w = new Synchronizer<MySyncPolicy_v_w>(MySyncPolicy_v_w(10), sub_white, sub_vicon_w);
    sync_v_b = new Synchronizer<MySyncPolicy_v_b>(MySyncPolicy_v_b(10), sub_black, sub_vicon_b);
    sync_v_o = new Synchronizer<MySyncPolicy_v_o>(MySyncPolicy_v_o(10), sub_orange, sub_vicon_o);

    sync->registerCallback(boost::bind(&MainWindow::sync_imu_data_cb,this, _1, _2, _3));
    sync_v_w->registerCallback(boost::bind(&MainWindow::sync_white_imu_vicon_cb,this, _1, _2));
    sync_v_b->registerCallback(boost::bind(&MainWindow::sync_black_imu_vicon_cb,this, _1, _2));
    sync_v_o->registerCallback(boost::bind(&MainWindow::sync_orange_imu_vicon_cb,this, _1, _2));
    joint_tfs_pub = n_.advertise<gui_data_collection::TfArrayStamped>("joint_tfs", 1);
    Euler_should_jnt_pub = n_.advertise<geometry_msgs::Vector3Stamped>("Euler_ypr_shoulder", 1);
    Euler_elbow_jnt_pub = n_.advertise<geometry_msgs::Vector3Stamped>("Euler_ypr_elbow", 1);
    Euler_hand_jnt_pub = n_.advertise<geometry_msgs::Vector3Stamped>("Euler_ypr_hand", 1);
}
void MainWindow::loop_rate_calculate()
{
    static int i;
    static double sampl_counter;
    static ros::Time begin;
    if(i==0)
    {
        sampl_counter ++;
        begin = ros::Time::now();
        i++;
    }
    else
    {
        sampl_counter ++;
        ros::Time now = ros::Time::now();
        if (now.toSec()-begin.toSec()>2)
        {
            //qDebug()<< "sampling rate= :" << sampl_counter/(now.toSec()-begin.toSec());
            begin = now;
            sampl_counter = 0;
        }
    }
}
void MainWindow::sync_imu_data_cb(const sensor_msgs::ImuConstPtr &msg_black, const sensor_msgs::ImuConstPtr &msg_white,const sensor_msgs::ImuConstPtr &msg_orange)
{
    loop_rate_calculate();
    Eigen::Quaternion<double> R_Gb2_Bb2 = Eigen::Quaternion<double>(msg_black.get()->orientation.w,msg_black.get()->orientation.x,msg_black.get()->orientation.y,msg_black.get()->orientation.z);
    Eigen::Quaternion<double> R_Gw2_Bw2 = Eigen::Quaternion<double>(msg_white.get()->orientation.w,msg_white.get()->orientation.x,msg_white.get()->orientation.y,msg_white.get()->orientation.z);
    Eigen::Quaternion<double> R_Go2_Bo2 = Eigen::Quaternion<double>(msg_orange.get()->orientation.w,msg_orange.get()->orientation.x,msg_orange.get()->orientation.y,msg_orange.get()->orientation.z);
    v_b = Eigen::Vector3d(msg_black.get()->linear_acceleration.x,msg_black.get()->linear_acceleration.y,msg_black.get()->linear_acceleration.z);
    v_w = Eigen::Vector3d(msg_white.get()->linear_acceleration.x,msg_white.get()->linear_acceleration.y,msg_white.get()->linear_acceleration.z);
    v_o = Eigen::Vector3d(msg_orange.get()->linear_acceleration.x,msg_orange.get()->linear_acceleration.y,msg_orange.get()->linear_acceleration.z);
    if (joint_state_pub ==true && stop == false)
    {
        Eigen::Quaternion<double> Shoulder_L_qua = R_Gv_Gb2*R_Gb2_Bb2*R_BBb2_Bb2.inverse(); //R_Gv_Gb2 is where recalibration happended
        Eigen::Quaternion<double> Elbow_L_qua = R_Gv_Gw2*R_Gw2_Bw2*R_BBw2_Bw2.inverse();
        Eigen::Quaternion<double> Hand_L_qua = R_Gv_Go2*R_Go2_Bo2*R_BBo2_Bo2.inverse()*Eigen::AngleAxisd(M_PI,Eigen::Vector3d::UnitZ());

        Eigen::Quaternion<double> shoulder_J_tf = (R_Gv_Bb0).inverse()*(Shoulder_L_qua);
        Eigen::Quaternion<double> elbow_J_tf = (Shoulder_L_qua).inverse()*Elbow_L_qua;
        Eigen::Quaternion<double> wrist_J_tf = (Elbow_L_qua).inverse()*Hand_L_qua;

        br_tf_cb(shoulder_J_tf,elbow_J_tf,wrist_J_tf);

        double vector_angle1;double vector_angle2;
        vector_angle1 = calulate_vector_angle(v_b,v_w);
        vector_angle2= calulate_vector_angle(v_b,v_o);
        ui->textBrowser_joint_state_pub->setText(text);
        text.clear();
        text += "Shoulder and Elbow gravity vector angle:";
        text += QString::number(vector_angle1-vector_angle1_init)+"\n";
        text += "Elbow and Hand gravity vector angle:"   ;
        text += QString::number(vector_angle2-vector_angle2_init)+"\n";
        ui->textBrowser_zeros_pose_tf->setText(text);
    }
}
void MainWindow::sync_white_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_white, const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    Eigen::Quaternion<double> R_Gv_Bv;
    Eigen::Quaternion<double> R_Gw2_Bw2 = Eigen::Quaternion<double>(msg_white.get()->orientation.w,msg_white.get()->orientation.x,msg_white.get()->orientation.y,msg_white.get()->orientation.z);
    R_Gv_Bv = Eigen::Quaternion<double>(vicon_msg.get()->transform.rotation.w,vicon_msg.get()->transform.rotation.x,vicon_msg.get()->transform.rotation.y,vicon_msg.get()->transform.rotation.z);
    //qDebug()<<"inside white!!";

    if(start_white_buffer == true)
    {
        qDebug()<<vicon_buff.count();
        if (frame_count_v_w%2 == 0)
        {
            imu_buff.append(R_Gw2_Bw2);
            vicon_buff.append(R_Gv_Bv);
        }
        if (vicon_buff.count()==400)
        {
            Eigen::Matrix3d mat;
            imu_vicon_body_alginment(imu_buff,vicon_buff,mat);
            R_Bw_Bvw = Eigen::Quaterniond(mat);
            qDebug()<<R_Bw_Bvw.w()<<","<<R_Bw_Bvw.x()<<","<<R_Bw_Bvw.y()<<","<<R_Bw_Bvw.z()<<"\n";
            debug_matrix(R_Bw_Bvw); // debug here!!
            start_white_buffer = false;
        }
    }
    if (capture_global_frame_w == true) // maybe periodically here !!)
    {
        R_Gv_Gw2 = R_Gv_Bv*R_Bw_Bvw.inverse()*R_Gw2_Bw2.inverse();
        //calculate_new_R_GV_Gimu(QString("white"),R_Bw_Bvw,R_Gw2_Bw2,R_Gv_Bv,R_Gv_Gw2);
        R_Gv_Bw0 = R_Gv_Gw2*R_Gw2_Bw2;
        R_BBw2_Bw2 = (R_Gv_Bw0).inverse()*(R_Gv_Gw2*R_Gw2_Bw2);  //here we assume sensor and human body are aligned !!
        capture_global_frame_w = false;
    }
    if(joint_state_pub ==true && frame_count_v_w%20 == 0)
    {
        calculate_new_R_GV_Gimu(QString("white"),R_Bw_Bvw,R_Gw2_Bw2,R_Gv_Bv,R_Gv_Gw2);
        frame_count_v_w = 0;
    }
    frame_count_v_w = frame_count_v_w+1;
}
void MainWindow::sync_black_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_black, const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    Eigen::Quaternion<double> R_Gv_Bv;
    Eigen::Quaternion<double> R_Gb2_Bb2 = Eigen::Quaternion<double>(msg_black.get()->orientation.w,msg_black.get()->orientation.x,msg_black.get()->orientation.y,msg_black.get()->orientation.z);
    R_Gv_Bv = Eigen::Quaternion<double>(vicon_msg.get()->transform.rotation.w,vicon_msg.get()->transform.rotation.x,vicon_msg.get()->transform.rotation.y,vicon_msg.get()->transform.rotation.z);
    //qDebug()<<"inside black!!";

    if(start_black_buffer == true)
    {
        qDebug()<<vicon_buff.count();
            if (frame_count_v_b%2 == 0)
            {
                imu_buff.append(R_Gb2_Bb2);
                vicon_buff.append(R_Gv_Bv);
            }
        if (vicon_buff.count()==400)
        {
            Eigen::Matrix3d mat;
            imu_vicon_body_alginment(imu_buff,vicon_buff,mat);
            R_Bb_Bvb = Eigen::Quaterniond(mat);
            R_Bb_Bvb = R_Bb_Bvb.normalized();
            qDebug()<<R_Bb_Bvb.w()<<","<<R_Bb_Bvb.x()<<","<<R_Bb_Bvb.y()<<","<<R_Bb_Bvb.z()<<"\n";
            debug_matrix(R_Bb_Bvb); // debug here!!
            start_black_buffer = false;
        }
    }
    if (capture_global_frame_b == true ) // maybe periodically here !!
    {
        R_Gv_Gb2 = R_Gv_Bv*R_Bb_Bvb.inverse()*R_Gb2_Bb2.inverse();
        //calculate_new_R_GV_Gimu(QString("black"),R_Bb_Bvb,R_Gb2_Bb2,R_Gv_Bv,R_Gv_Gb2);
        R_Gv_Bb0 = R_Gv_Gb2*R_Gb2_Bb2;
        R_BBb2_Bb2 = (R_Gv_Bb0).inverse()*(R_Gv_Gb2*R_Gb2_Bb2);   //here we assume sensor and human body are aligned !!
        capture_global_frame_b = false;
    }
    if(joint_state_pub ==true && frame_count_v_b%20 == 0)
    {
        calculate_new_R_GV_Gimu(QString("black"),R_Bb_Bvb,R_Gb2_Bb2,R_Gv_Bv,R_Gv_Gb2);
        frame_count_v_b = 0;
    }
    frame_count_v_b = frame_count_v_b+1;
}
void MainWindow::sync_orange_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_orange, const geometry_msgs::TransformStampedConstPtr &vicon_msg)
{
    Eigen::Quaternion<double> R_Gv_Bv;
    Eigen::Quaternion<double> R_Go2_Bo2 = Eigen::Quaternion<double>(msg_orange.get()->orientation.w,msg_orange.get()->orientation.x,msg_orange.get()->orientation.y,msg_orange.get()->orientation.z);
    R_Gv_Bv = Eigen::Quaternion<double>(vicon_msg.get()->transform.rotation.w,vicon_msg.get()->transform.rotation.x,vicon_msg.get()->transform.rotation.y,vicon_msg.get()->transform.rotation.z);
    //qDebug()<<"inside orange!!";

    if(start_orange_buffer == true)
    {
            if (frame_count_v_o%2 == 0)
            {
                imu_buff.append(R_Go2_Bo2);
                vicon_buff.append(R_Gv_Bv);
            }
        if (vicon_buff.count()==400)
        {
            Eigen::Matrix3d mat;
            imu_vicon_body_alginment(imu_buff,vicon_buff,mat);
            R_Bo_Bvo = Eigen::Quaterniond(mat);
            qDebug()<<R_Bo_Bvo.w()<<","<<R_Bo_Bvo.x()<<","<<R_Bo_Bvo.y()<<","<<R_Bo_Bvo.z()<<"\n";
            debug_matrix(R_Bo_Bvo); // debug here!!
            start_orange_buffer = false;
        }
    }
    if (capture_global_frame_o == true) // maybe periodically here !!)
    {
        R_Gv_Go2 = R_Gv_Bv*R_Bo_Bvo.inverse()*R_Go2_Bo2.inverse();
        //calculate_new_R_GV_Gimu(QString("orange"),R_Bo_Bvo,R_Go2_Bo2,R_Gv_Bv,R_Gv_Go2);
        R_Gv_Bo0 = R_Gv_Go2*R_Go2_Bo2;
        R_BBo2_Bo2 = (R_Gv_Bo0).inverse()*(R_Gv_Go2*R_Go2_Bo2);  //here we assume sensor and human body are aligned !!
        capture_global_frame_o = false;
    }
    if(joint_state_pub ==true && frame_count_v_o%30 == 0)
    {
        calculate_new_R_GV_Gimu(QString("orange"),R_Bo_Bvo,R_Go2_Bo2,R_Gv_Bv,R_Gv_Go2);
        frame_count_v_o = 0;
    }
    frame_count_v_o = frame_count_v_o+1;
}
Eigen::Vector3d MainWindow::euler_angle_convert(double &x, double &y, double &z, double &w)
{
        Eigen::Vector3d euler_angles;
        double roll;double pitch;double yaw;
        roll = atan2(double(2) * (w * x + y * z),
                     double(1) - double(2) * (x * x + y * y));
        pitch = asin(qMax(double(-1), qMin(double(1), double(2) * (w * y - z * x))));
        yaw = atan2(double(2) * (w * z + x * y),
                    double(1) - double(2) *(y* y + z * z));
        euler_angles[0] = roll;
        euler_angles[1] = pitch;
        euler_angles[2] = yaw;
        return euler_angles;
}

double MainWindow::calulate_vector_angle(Eigen::Vector3d v1, Eigen::Vector3d v2)
{
    double angle;
    angle = acos(v1.dot(v2)/(v1.norm()*v2.norm()))*180.0/M_PI;
    return angle;
}
void MainWindow::debug_matrix(Eigen::Quaternion<double> quat)
{
    qDebug()<<quat.toRotationMatrix()(0,0)<<","<<quat.toRotationMatrix()(0,1)<<","<<quat.toRotationMatrix()(0,2)<<"\n"
              <<quat.toRotationMatrix()(1,0)<<","<<quat.toRotationMatrix()(1,1)<<","<<quat.toRotationMatrix()(1,2)<<"\n"
             <<quat.toRotationMatrix()(2,0)<<","<<quat.toRotationMatrix()(2,1)<<","<<quat.toRotationMatrix()(2,2)<<"\n";
}
bool MainWindow::listen_to_vicon(QString vicon_id,Eigen::Quaternion<double>& quat_vicon)
{
    QString msg;    tf::StampedTransform tf_tmp;
    if (vicon_id == QString("white"))
    {
        msg.append("/vicon/myo_w/myo_w");
    }
    if (vicon_id == QString("black"))
    {
        msg.append("/vicon/myo_b/myo_b");
    }
    if (vicon_id == QString("orange"))
    {
        msg.append("/vicon/hand/hand");
    }
    ros::Time now = ros::Time::now();
    if (listener.waitForTransform(msg.toStdString().c_str(), "/world",
                                  now,ros::Duration(0.1)))
    {
        Eigen::Quaternion<double> vicon_data;
        try{
        listener.lookupTransform(msg.toStdString().c_str(), "/world",
                                  ros::Time(0), tf_tmp);
        vicon_data=Eigen::Quaternion<double>(tf_tmp.getRotation().w(),tf_tmp.getRotation().x(),tf_tmp.getRotation().y(),tf_tmp.getRotation().z());
        }
        catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.001).sleep();
        }
        quat_vicon = vicon_data;
        return true;
    }
    else
    {
        return false;
    }

}
void MainWindow::imu_vicon_body_alginment(QVector<Eigen::Quaternion<double> > imu_buff_, QVector<Eigen::Quaternion<double> > vicon_buffer_,Eigen::Matrix3d& mat_X)
{
    int count = 0;
    // A stands for imu; B stands for vicon;
    Eigen::Matrix3d A_log;Eigen::Matrix3d B_log;Eigen::Quaternion<double> A;Eigen::Quaternion<double> B;
    Eigen::Matrix3d M = Eigen::MatrixXd::Zero(3,3);
    for (int i=1;i<imu_buff_.size()-1;i++)
    {
      Eigen::Matrix3d M_; Eigen::Vector3d A_vec; Eigen::Vector3d B_vec;
      A = imu_buff_[i-1].inverse()*imu_buff_[i];
      calculate_log(A.toRotationMatrix(),A_log);
      B = vicon_buffer_[i-1].inverse()*vicon_buffer_[i];
      calculate_log(B.toRotationMatrix(),B_log);
      double norm_A;double norm_B;
      norm_A = A_log.norm();
      norm_B = B_log.norm();
      if (fabs(norm_A-norm_B) < 0.01)
      {
          count = count +1;
          extract_w(A_log,A_vec);
          extract_w(B_log,B_vec);
          M_ = B_vec*A_vec.transpose();
          M = M + M_;
          //ROS_INFO("frame count:= %d",count);
          if (count == 10)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 15)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 20)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 25)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 30)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 35)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 40)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 45)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 50)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
          if (count == 55)
          {
              Eigen::Matrix3d MAT_tmp = M;
              Eigen::Matrix3d mat_X_tmp = ((MAT_tmp.transpose()*MAT_tmp).inverse()).sqrt()*MAT_tmp.transpose();
              Eigen::Quaternion<double> quat_tmp = Eigen::Quaterniond(mat_X_tmp);
              qDebug()<<"frame count:="<<count<< "\n"<<quat_tmp.w()<<","<<quat_tmp.x()<<","<<quat_tmp.y()<<","<<quat_tmp.z()<<"\n";
              debug_matrix(quat_tmp); // debug here!!
          }
      }
      else
      {
          M_ = Eigen::MatrixXd::Zero(3,3);
          M = M +M_;
      }
    }
    mat_X = ((M.transpose()*M).inverse()).sqrt()*M.transpose();
}
void MainWindow::calculate_log(Eigen::Matrix3d mat, Eigen::Matrix3d& mat_log)
{
    double psi;
    psi = acos(((mat(0,0)+mat(1,1)+mat(2,2))-1)/2);
    mat_log = psi/(2*sin(psi))*(mat-mat.transpose());
}
void MainWindow::calculate_new_R_GV_Gimu(QString vicon_id, Eigen::Quaternion<double> R_B_Bv,Eigen::Quaternion<double> R_Gimu_B2,Eigen::Quaternion<double> R_Gv_Bv,Eigen::Quaternion<double> &R_Gv_Gimu2)
{
    // R_Gimu_B2 : new measurement from IMU.
    // R_Gv_Bv: new measurement from Vicon.
    // R_Gv_Gimu1: old sensor in vicon frame tf global;
    Eigen::Quaternion<double> R_Gv_Gimu2_;
    R_Gv_Gimu2_ = R_Gv_Bv*R_B_Bv.inverse()*R_Gimu_B2.inverse();
    //qDebug()<<"tf alignment whie is:"<<"\n";
    //debug_matrix(R_Gv_Gimu2_);
    double tmp_quat = R_Gv_Gimu2.dot(R_Gv_Gimu2_);
    //Eigen::Vector3d vec = tmp_quat.toRotationMatrix().eulerAngles(2,1,0);
    double angle = acos(tmp_quat)*180/M_PI;
    qDebug()<<"Inertial frame deviation angel in degree is :"<<angle<<"\n";
    if (1<angle && angle<10) // 1 degree <angle<10 degree
    {
        qDebug()<<"Inertia frame reset!!!";
        R_Gv_Gimu2 = R_Gv_Gimu2_;
    }
    else
    {
        R_Gv_Gimu2 = R_Gv_Gimu2; // debug here!!
    }
}
void MainWindow::br_tf_cb(Eigen::Quaternion<double> shoulder_J_tf,Eigen::Quaternion<double> elbow_J_tf,Eigen::Quaternion<double> wrist_J_tf)
{
    tf::Transform transform1;
    tf::Transform transform2;
    tf::Transform transform3;
    tf::Transform transform4;
    tf::Transform transform5;
    tf::Transform transform6;
    tf::Quaternion q;
    gui_data_collection::TfArrayStamped tfs;
    geometry_msgs::Vector3Stamped shoulder_euler_vec;
    geometry_msgs::Vector3Stamped elbow_euler_vec;
    geometry_msgs::Vector3Stamped hand_euler_vec;
    geometry_msgs::Quaternion q_tmp;
    ros::Time now_ = ros::Time::now();
    tfs.header.stamp = now_;
    shoulder_euler_vec.header.stamp = now_;
    elbow_euler_vec.header.stamp = now_;
    hand_euler_vec.header.stamp = now_;
    tfs.tfs.resize(3);
    transform1.setOrigin( tf::Vector3(0,0,0) );
    q.setX(shoulder_J_tf.x());q.setY(shoulder_J_tf.y());q.setZ(shoulder_J_tf.z());q.setW(shoulder_J_tf.w());
    transform1.setRotation(q);
    q_tmp.x = q.getX();q_tmp.y = q.getY();q_tmp.z = q.getZ();q_tmp.w = q.getW();
    tfs.tfs[0] = q_tmp;
    Eigen::Vector3d vec = shoulder_J_tf.toRotationMatrix().eulerAngles(2,1,0);
    shoulder_euler_vec.vector.x = vec[0];shoulder_euler_vec.vector.y = vec[1];shoulder_euler_vec.vector.z = vec[2];
    transform2.setOrigin( tf::Vector3(0.31315, 2.0664E-05,0) );//
    q.setX(elbow_J_tf.x());q.setY(elbow_J_tf.y());q.setZ(elbow_J_tf.z());q.setW(elbow_J_tf.w());
    transform2.setRotation(q);
    q_tmp.x = q.getX();q_tmp.y = q.getY();q_tmp.z = q.getZ();q_tmp.w = q.getW();
    tfs.tfs[1] = q_tmp;
    vec = elbow_J_tf.toRotationMatrix().eulerAngles(2,1,0);
    elbow_euler_vec.vector.x = vec[0];elbow_euler_vec.vector.y = vec[1];elbow_euler_vec.vector.z = vec[2];
    transform3.setOrigin( tf::Vector3(0.25152,-0.0072209, 0.0) );
    q.setX(wrist_J_tf.x());q.setY(wrist_J_tf.y());q.setZ(wrist_J_tf.z());q.setW(wrist_J_tf.w());
    transform3.setRotation(q);
    q_tmp.x = q.getX();q_tmp.y = q.getY();q_tmp.z = q.getZ();q_tmp.w = q.getW();
    tfs.tfs[2] = q_tmp;
    vec = wrist_J_tf.toRotationMatrix().eulerAngles(2,1,0);
    hand_euler_vec.vector.x = vec[0];hand_euler_vec.vector.y = vec[1];hand_euler_vec.vector.z = vec[2];
    transform4.setOrigin( tf::Vector3(-0.395348085615453, 0.222880076997622, 1.01) );
    q.setX(0);q.setY(0.70711);q.setZ(0);q.setW(0.70711);
    transform4.setRotation(q);
    transform5.setOrigin( tf::Vector3(0.31315, 2.0664E-05,0) );
    q.setX(0);q.setY(-0.70711);q.setZ(0);q.setW(0.70711);
    transform5.setRotation(q);
    transform6.setOrigin( tf::Vector3(0.25152,-0.0072209, 0.0) );
    q.setX(0.70711);q.setY(0);q.setZ(0);q.setW(0.70711);
    transform6.setRotation(q);
    joint_tfs_pub.publish(tfs);
    Euler_should_jnt_pub.publish(shoulder_euler_vec);
    Euler_elbow_jnt_pub.publish(elbow_euler_vec);
    Euler_hand_jnt_pub.publish(hand_euler_vec);
    br.sendTransform(tf::StampedTransform(transform4, ros::Time::now(),"base_link" ,"shoulder" ));
    br.sendTransform(tf::StampedTransform(transform5, ros::Time::now(),"shoulder" ,"Elbow" ));
    br.sendTransform(tf::StampedTransform(transform6, ros::Time::now(),"Elbow" ,"Wrist" ));
    br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(),"shoulder" ,"Upper_arm" ));
    br.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "Upper_arm", "Lower_arm"));
    br.sendTransform(tf::StampedTransform(transform3, ros::Time::now(),  "Lower_arm", "Hand"));

}
void MainWindow::extract_w(Eigen::Matrix3d mat, Eigen::Vector3d &vec)
{
    vec[0] = mat(2,1);
    vec[1] = mat(0,2);
    vec[2] = mat(1,0);
}
void MainWindow::on_pushButton_joint_state_pub_clicked()
{
    joint_state_pub = true;
}
void MainWindow::on_pushButton_capture_global_frame_clicked()
{
    capture_global_frame_w = true;
    capture_global_frame_b = true;
    capture_global_frame_o = true;
    text += QString("Setting Static Nature Pose ....");
    text += "\n";
    ui->textBrowser_global_frame->setText(text);
    stop = false;
}
void MainWindow::on_pushButton_stop_clicked()
{
    stop = true;
    capture_global_frame_w = false;
    capture_global_frame_b = false;
    capture_global_frame_o = false;
}
void MainWindow::on_pushButton_init_pose_clicked()
{
    vector_angle1_init = calulate_vector_angle(v_b,v_w);
    vector_angle2_init= calulate_vector_angle(v_b,v_o);

}
void MainWindow::on_pushButton_vicon_white_alignment_clicked()
{
    vicon_buff.clear();
    imu_buff.clear();
//    vicon_buff.resize(300);
//    imu_buff.resize(300);
    start_white_buffer = true;
}
void MainWindow::on_pushButton_Vicon_black_alignment_clicked()
{
    vicon_buff.clear();
    imu_buff.clear();
//    vicon_buff.resize(300);
//    imu_buff.resize(300);
    start_black_buffer = true;
}
void MainWindow::on_pushButton_Vicon_orange_alignment_clicked()
{
    vicon_buff.clear();
    imu_buff.clear();
//    vicon_buff.resize(300);
//    imu_buff.resize(300);
    start_orange_buffer = true;
}
