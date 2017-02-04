#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include <cmath>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QLCDNumber>
#include <QTimer>
#include <ros/ros.h>
#include <QtCore/QDebug>
#include <QFile>
#include <QTextStream>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <gui_data_collection/TfArrayStamped.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/src/Geometry/EulerAngles.h>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <../../build/imu_vicon_arm_calibration/ui_imu_vicon_arm_calibration_Widget_test.h>
using namespace message_filters;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::Imu,sensor_msgs::Imu> MySyncPolicy;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy_v_w;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy_v_b;
typedef sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TransformStamped> MySyncPolicy_v_o;
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void init();
    void loop_rate_calculate();
    void sync_imu_data_cb(const sensor_msgs::ImuConstPtr &msg_black, const sensor_msgs::ImuConstPtr &msg_white,const sensor_msgs::ImuConstPtr &msg_orange);
    void sync_white_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_white,const geometry_msgs::TransformStampedConstPtr &vicon_msg);
    void sync_black_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_black,const geometry_msgs::TransformStampedConstPtr &vicon_msg);
    void sync_orange_imu_vicon_cb(const sensor_msgs::ImuConstPtr &msg_orange,const geometry_msgs::TransformStampedConstPtr &vicon_msg);
    Eigen::Vector3d euler_angle_convert(double& x,double& y,double& z,double& w);
    double calulate_vector_angle(Eigen::Vector3d v1, Eigen::Vector3d v2);
    void debug_matrix(Eigen::Quaternion<double> quat);
    bool listen_to_vicon(QString vicon_id, Eigen::Quaternion<double> &quat_vicon);
    void imu_vicon_body_alginment(QVector<Eigen::Quaternion<double> > imu_buff_, QVector<Eigen::Quaternion<double> > vicon_buffer_,Eigen::Matrix3d& mat_X);
    void calculate_log(Eigen::Matrix3d mat, Eigen::Matrix3d &mat_log);
    void calculate_new_R_GV_Gimu(QString vicon_id, Eigen::Quaternion<double> R_B_Bv,Eigen::Quaternion<double> R_Gimu_B2,Eigen::Quaternion<double> R_Gv_Bv,Eigen::Quaternion<double> &R_Gv_Gimu2);
    void br_tf_cb(Eigen::Quaternion<double> shoulder_J_tf,Eigen::Quaternion<double> elbow_J_tf,Eigen::Quaternion<double> wrist_J_tf);
    void extract_w(Eigen::Matrix3d mat, Eigen::Vector3d& vec);
signals:

public slots:


private slots:

    void on_pushButton_joint_state_pub_clicked();

    void on_ros_spin();

    void on_pushButton_capture_global_frame_clicked();

    void on_pushButton_stop_clicked();

    void on_pushButton_init_pose_clicked();

    void on_pushButton_vicon_white_alignment_clicked();

    void on_pushButton_Vicon_black_alignment_clicked();

    void on_pushButton_Vicon_orange_alignment_clicked();

private:
    Ui::MainWindow *ui;
    message_filters::Subscriber<sensor_msgs::Imu> sub_black;
    message_filters::Subscriber<sensor_msgs::Imu> sub_white;
    message_filters::Subscriber<sensor_msgs::Imu> sub_orange;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_w;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_b;
    message_filters::Subscriber<geometry_msgs::TransformStamped> sub_vicon_o;

    tf::TransformListener listener;
    ros::Publisher joint_tfs_pub;
    ros::Publisher Euler_should_jnt_pub;
    ros::Publisher Euler_elbow_jnt_pub;
    ros::Publisher Euler_hand_jnt_pub;

    Synchronizer<MySyncPolicy>* sync;
    Synchronizer<MySyncPolicy_v_w>* sync_v_w;
    Synchronizer<MySyncPolicy_v_b>* sync_v_b;
    Synchronizer<MySyncPolicy_v_o>* sync_v_o;
    Eigen::Vector3d v_b;
    Eigen::Vector3d v_w;
    Eigen::Vector3d v_o;
    int frame_count_v_w;
    int frame_count_v_b;
    int frame_count_v_o;
    double vector_angle1_init;double vector_angle2_init;
    bool stop;
    bool joint_state_pub;
    bool start_white_buffer;
    bool start_black_buffer;
    bool start_orange_buffer;
    bool align_imu_global_frame;
    bool capture_global_frame_w;
    bool capture_global_frame_b;
    bool capture_global_frame_o;
    tf::TransformBroadcaster br;
    QTimer* ros_timer;
    QString text;
    QVector<Eigen::Quaternion<double> > vicon_buff;
    QVector<Eigen::Quaternion<double> > imu_buff;
    Eigen::Quaternion<double> R_Bw_Bvw; // vicon body frame in imu body frame
    Eigen::Quaternion<double> R_Bb_Bvb;
    Eigen::Quaternion<double> R_Bo_Bvo;

    Eigen::Quaternion<double> R_Gv_Gw2;// updated new global imu frame in vicon frame
    Eigen::Quaternion<double> R_Gv_Gb2;
    Eigen::Quaternion<double> R_Gv_Go2;


public:
    Eigen::Quaternion<double> R_Gb_Gw;// decline to use later maybe;
    Eigen::Quaternion<double> R_Gb_Go;// decline to use later maybe;
    Eigen::Quaternion<double> R_Gv_Bw0;// new IMU measurement in vicon frame
    Eigen::Quaternion<double> R_Gv_Bb0;
    Eigen::Quaternion<double> R_Gv_Bo0;

    Eigen::Quaternion<double> R_Gv_BBw2; //body frame in global
    Eigen::Quaternion<double> R_BBw2_Bw2; // sensor to body frame tf
    Eigen::Quaternion<double> R_Gv_BBb2;
    Eigen::Quaternion<double> R_BBb2_Bb2;
    Eigen::Quaternion<double> R_Gv_BBo2;
    Eigen::Quaternion<double> R_BBo2_Bo2;


protected:
     ros::NodeHandle n_;
};

#endif // MAINWINDOW_H
