#include "motion_compensation.h"
#define PI 3.141592

// using namespace motion_compensation

MatrixXd FK_MTM(MatrixXd JointAngles)
{

}

VectorXf motion_compensation::randv(int x)
{
  //  random function goes from -1 to 1
  VectorXf X=VectorXf::Random(x);
  return X.cwiseAbs();
}

void motion_compensation::moveRobot(MatrixXd theta_psm, MatrixXd theta_mtm, MatrixXd theta_pris)
{

//  cout<<theta<<endl;
 // while (ros::ok())
 // {
    std_msgs::Float64 psm_msg1,psm_msg2,psm_msg3,psm_msg4,psm_msg5,psm_msg6;
    std_msgs::Float64 mtm_msg1,mtm_msg2,mtm_msg3,mtm_msg4,mtm_msg5,mtm_msg6,mtm_msg7,pris_msg;
    // msg1.data = 0;//theta(0,0);
    // msg2.data = 0;//theta(0,1);
    // msg3.data = 0;//theta(0,2);
    // msg4.data = 0;//theta(0,3);
    // msg5.data = 0;//theta(0,4);
    // msg6.data = 0;//theta(0,5);

    psm_msg1.data = theta_psm(0,0);
    psm_msg2.data = theta_psm(0,1);
    psm_msg3.data = theta_psm(0,2);
    psm_msg4.data = theta_psm(0,3);
    psm_msg5.data = theta_psm(0,4);
    psm_msg6.data = theta_psm(0,5);

    mtm_msg1.data = theta_mtm(0,0);
    mtm_msg2.data = theta_mtm(0,1);
    mtm_msg3.data = theta_mtm(0,2);
    mtm_msg4.data = theta_mtm(0,3);
    mtm_msg5.data = theta_mtm(0,4);
    mtm_msg6.data = theta_mtm(0,5);
    mtm_msg7.data = theta_mtm(0,6);

    pris_msg.data = theta_pris(0,0);
    // ROS_INFO("%f", msg1.data);
    // ROS_INFO("%f", msg2.data);
    // ROS_INFO("%f", msg3.data);
    // ROS_INFO("%f", msg4.data);
    // ROS_INFO("%f", msg5.data);
    // ROS_INFO("%f", msg6.data);

    psm_pub1.publish(psm_msg1);
    psm_pub2.publish(psm_msg2);
    psm_pub3.publish(psm_msg3);
    psm_pub4.publish(psm_msg4);
    psm_pub5.publish(psm_msg5);
    psm_pub6.publish(psm_msg6);


    mtm_pub1.publish(mtm_msg1);
    mtm_pub2.publish(mtm_msg2);
    mtm_pub3.publish(mtm_msg3);
    mtm_pub4.publish(mtm_msg4);
    mtm_pub5.publish(mtm_msg5);
    mtm_pub6.publish(mtm_msg6);
    mtm_pub7.publish(mtm_msg7);

    pris_pub1.publish(pris_msg);
  //}

}


int main(int argc, char**argv)
{
  MatrixXd angles_psm(1,6),angles_mtm(1,7),angles_pris(1,1);
  ros::init(argc, argv, "motion_compensation");
  ros::NodeHandle n;
  motion_compensation obj(n);
  VectorXf c = obj.randv(101),f = VectorXf::LinSpaced(101,10.0f,20.0f), phi = PI *(obj.randv(101)-obj.randv(101)),t = VectorXf::LinSpaced(10001,0.0f,100.0f);
  MatrixXd P(2,2),Pred_P(2,2),kalman_A(2,2),I(2,2);
  kalman_A<<1,0.1,
            0,1;
  I=MatrixXd::Identity(2,2);
  VectorXf A(10001);
  VectorXf Meas(10001);
  MatrixXd D(2,1),H(1,2),Mean(2,1),Pred_Mean(2,1),Kg(2,1);
  MatrixXd Tip(10001,1);
  MatrixXd R(1,1);
  MatrixXd temp(1,2);
  MatrixXd temp2(1,1);
  MatrixXd temp3(1,1);

  D(0,0)=0.1;
  D(1,0)=1;

  H(0,0)=1;
  H(0,1)=0;

  int qdt=100;
  R(0,0)=4.219;

  for (int i=0;i<101;i++)
  {
    for (int j=0;j<10001;j++)
    A(j)=A(j)+c(i)*sin(2*PI*f(i)*t(j)+phi(i));
  }

  Meas=A+5*(obj.randv(10001)-obj.randv(10001));

  Mean(0,0)=A(0,0);
  Mean(1,0)=0;

  Tip(0,0)=0;
  P<<1000,0,
     0,100;


  for (int iter=1; iter<10001; iter++)
  {
    Pred_Mean=kalman_A*Mean;
    Pred_P=kalman_A*P*kalman_A.transpose()+D*qdt*D.transpose();
    temp=H*Pred_P;
    temp2=temp*H.transpose()+R;
    Kg=Pred_P*H.transpose()/temp2(0,0);
    temp3(0,0)=Meas(iter);
    Mean=Pred_Mean+Kg*(temp3-H*Pred_Mean);
    P=(I-Kg*H)*Pred_P;
    Tip(iter,0)=Mean(0,0);
  }

  ros::Rate loop_rate(10);

  for (int count=0;count<10001;count++)
  {
    double S1=A(count)/20;
    double S2=Tip(count)/20;
    for(int j=0;j<6;j++)
    {
      if (j==5)
      {
        angles_psm(0,j)=0.12-0.12*S2;
      }
      else
      {
        angles_psm(0,j) = 0;
      }

    }
    for(int j=0;j<7;j++)
    {
      angles_mtm(0,j) = 0;
    }

    angles_pris(0,0)=0.12*S1;
    // for (int i=0; i<10; i++)
    // {
      obj.moveRobot(angles_psm, angles_mtm, angles_pris);
      loop_rate.sleep();
    //}
  }
    // ros::spin();
}
