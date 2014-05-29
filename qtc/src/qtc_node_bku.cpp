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
string qtc_state0 = ""; // Previous qtc state
string qtc_state1 = ""; // Current  qtc state
bool PreviousQTCAvailable = false;

//void BuildQTC(string *qtc_state,  geometry_msgs::Point Pl0, geometry_msgs::Point Pl1, geometry_msgs::Point Pk0, geometry_msgs::Point Pk1 );
void BuildQTC();
bool TestPointEquality( geometry_msgs::Point p1, geometry_msgs::Point p2 ); //True if equal
double SubstractAngle( double A, double B ); // A - B
void UpdateData();
void ClonePointTo( geometry_msgs::Point P_origin, geometry_msgs::Point P );
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
  ros::Subscriber node_sub2 = n.subscribe( "/ferdi/fpose", 2, Pt2_Callback );

  while( ros::ok() ){
    if( Pt1_Received && Pt2_Received ){
      Pt1_Received = false; //|-> Wait for updated data
      Pt2_Received = false; //|
      if( PreviousDataAvailable ){ // We have current and previous states to compare movement, try to build QTC.
	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%
	cout << "------------------" << endl;
	cout << "BUILDING QTC!!!" << endl;
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

//void BuildQTC(string *qtc_state,  geometry_msgs::Point Pl0, geometry_msgs::Point Pl1, geometry_msgs::Point Pk0, geometry_msgs::Point Pk1 ){
void BuildQTC(){
  geometry_msgs::Point PL0, PL1, PK0, PK1;
  ClonePointTo( Pt1_0, PL0 );
  ClonePointTo( Pt1_1, PL1 );
  ClonePointTo( Pt2_0, PK0 );
  ClonePointTo( Pt2_1, PK1 );

  // bool PL_IsMoving = !TestPointEquality( PL0, PL1 ); //|--> If some point is not moving, we need to avoid some operations (e.g. atan2)
  // bool PK_IsMoving = !TestPointEquality( PK0, PK1 ); //|
  // bool PLK0_AreEqs = !TestPointEquality( PK0, PL0 ); //////|--> Check if both points are not equal (also, to avoid some operations).
  // bool PLK1_AreEqs = !TestPointEquality( PK1, PL1 ); //////|

  // // FRAME TRANSFORMATION
  
  // // 1) Set PL0 origin
  // SubstractPointTo( PL1, PL0 );
  // SubstractPointTo( PK0, PL0 );
  // SubstractPointTo( PK1, PL0 );
  // SubstractPointTo( PL0, PL0 ); //Origin (should be the last)


  double LK_Distance = EuclideanDistance( PL0, PK0 );
  double LK_Angle    = atan2( PK0.y - PL0.y, PK0.x - PL0.x );  //Two dimensional

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
    qtc_state1 = "-";
  else
    qtc_state1 = "+";
  


  // 4) Movement of L with respect to RL
  double L_Angle = atan2( PL1.y - PL0.y, PL1.x - PL0.x );    
  K_Angle = SubstractAngle( L_Angle, LK_Angle );
  if( fabs( L_Angle - PI ) < 0.00001 || fabs( PI + L_Angle ) < 0.00001 )
    qtc_state1 += "0";
  else if( L_Angle > 0 )
    qtc_state1 = "-";
  else
    qtc_state1 = "+";

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
  ClonePointTo( Pt1_1, Pt1_0 );
  ClonePointTo( Pt2_1, Pt2_0 );
}

void ClonePointTo( geometry_msgs::Point P_origin, geometry_msgs::Point P ){
  P.x = P_origin.x;
  P.y = P_origin.y;
  P.z = P_origin.z;
}

