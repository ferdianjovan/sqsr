// "qtc_node.cpp"
// This program subscribes to the position of two agents in morse (global position) to build the QTC representations.
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <cmath>
using namespace std;

double const PI = 3.14159265358979323846;

bool PreviousDataAvailable = false;
bool Pt1_Received = false, Pt2_Received = false; //Flags of upgraded data
geometry_msgs::Point Pt1_0, Pt2_0; // Previous data (position / no-orientation)
geometry_msgs::Point Pt1_1, Pt2_1; // Current  data (position / no-orientation)

// QTC
string qtc_state1 = ""; // Current  qtc state

void BuildQTC();
bool TestPointEquality( geometry_msgs::Point p1, geometry_msgs::Point p2 ); //True if equal
double SubstractAngle( double A, double B ); // A - B
void UpdateData();
double EuclideanDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 );
void SubstractPointTo( geometry_msgs::Point p1, geometry_msgs::Point p2 ); // p1 = p1 - p2

void Pt1_Callback ( const geometry_msgs::PoseStamped::ConstPtr& msg ){
  Pt1_Received = true;
  Pt1_1 = ( msg->pose ).position;
}
void Pt2_Callback ( const geometry_msgs::PoseStamped::ConstPtr& msg ){
  Pt2_Received = true;
  Pt2_1 = ( msg->pose ).position;
}


int main( int argc, char **argv ){

  ros::init(argc, argv, "qtc_node");

  ros::NodeHandle n;
  ros::Subscriber node_sub1 = n.subscribe( "/lenka/lpose", 2, Pt1_Callback );
  ros::Subscriber node_sub2 = n.subscribe( "/marco/mpose", 2, Pt2_Callback );

  while( ros::ok() ){
    if( Pt1_Received && Pt2_Received ){
      Pt1_Received = false; //|-> Wait for updated data
      Pt2_Received = false; //|
      if( PreviousDataAvailable ){ // We have current and previous states to compare movement, try to build QTC.
	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%
	cout << "------------------" << endl;
	BuildQTC();
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%

	UpdateData();
	PreviousDataAvailable = true;
      }
      else{ // First data received: "update previous state and continue".
	UpdateData();
	PreviousDataAvailable = true;
	continue;
      }

    }
    ros::spinOnce();
  }
  return 0;
}


void BuildQTC(){
  geometry_msgs::Point PL0, PL1, PK0, PK1;
  PL0 = Pt1_0;
  PL1 = Pt1_1;
  PK0 = Pt2_0;
  PK1 = Pt2_1;

  double LK_Distance = EuclideanDistance( PL0, PK0 );
  double LK_Angle    = atan2( PK0.y - PL0.y, PK0.x - PL0.x );  //Two dimensional

  cout << "PL0[" << PL0.x << "," << PL0.y << "," << PL0.z << "]" << endl;
  cout << "PL1[" << PL1.x << "," << PL1.y << "," << PL1.z << "]" << endl;
  cout << "PK0[" << PK0.x << "," << PK0.y << "," << PK0.z << "]" << endl;
  cout << "PK1[" << PK1.x << "," << PK1.y << "," << PK1.z << "]" << endl;
  cout << "LK_Distance = " << LK_Distance << endl;
  cout << "LK_Angle    = " << LK_Angle << endl;

  qtc_state1 = "";

  // 1) Movement of K with respect to L
  if( EuclideanDistance( PL0, PK1 ) <  LK_Distance ) 
    qtc_state1 += "-";
  else if( EuclideanDistance( PL0, PK1 ) == LK_Distance ) 
    qtc_state1 += "0";
  else
    qtc_state1 += "+";

  // 2) Movement of L with respect to K
  if( EuclideanDistance( PK0, PL1 ) <  LK_Distance ) 
    qtc_state1 += "-";
  else if( EuclideanDistance( PK0, PL1 ) == LK_Distance ) 
    qtc_state1 += "0";
  else
    qtc_state1 += "+";

  // 3) Movement of K with respect to RL
  double K_Angle = atan2( PK1.y - PK0.y, PK1.x - PK0.x );
  K_Angle = SubstractAngle( K_Angle, LK_Angle );
  if( fabs( K_Angle - PI ) < 0.00001 || fabs( PI + K_Angle ) < 0.00001 )
    qtc_state1 += "0";
  else if( K_Angle > 0 )
    qtc_state1 += "-";
  else
    qtc_state1 += "+";

  // 4) Movement of L with respect to RL
  double L_Angle = atan2( PL1.y - PL0.y, PL1.x - PL0.x );    
  K_Angle = SubstractAngle( L_Angle, LK_Angle );
  if( fabs( L_Angle - PI ) < 0.00001 || fabs( PI + L_Angle ) < 0.00001 )
    qtc_state1 += "0";
  else if( L_Angle > 0 )
    qtc_state1 += "-";
  else
    qtc_state1 += "+";

  cout << qtc_state1 << endl;
}


double SubstractAngle( double A, double B ){ // A - B
  double answer = A - B;
  while( fabs( answer ) > PI ){
    if( answer > 0 )
      answer -= 2*PI;
    else
      answer += 2*PI;
  }
  return answer;
}


void SubstractPointTo( geometry_msgs::Point p1, geometry_msgs::Point p2 ){ // p1 = p1 - p2
  p1.x -= p2.x;
  p1.y -= p2.y;
  p1.z -= p2.z;
}


double EuclideanDistance( geometry_msgs::Point p1, geometry_msgs::Point p2 ){
  return sqrt( pow( p1.x - p2.x, 2 ) + pow( p1.y - p2.y, 2 ) + pow( p1.z - p2.z, 2 ) );
}


bool TestPointEquality( geometry_msgs::Point p1, geometry_msgs::Point p2 ){ //True if equal
  bool answer = true;
  answer &= ( p1.x == p2.x );
  answer &= ( p1.y == p2.y );
  answer &= ( p1.z == p2.z );
  return answer; 
}


void UpdateData(){
  Pt1_0 = Pt1_1;
  Pt2_0 = Pt2_1;
}


