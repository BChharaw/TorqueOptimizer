// MTE 119 Project 2.cpp
//


#include <iostream>
#include <cmath>
#include <chrono>


using namespace std;


// General Constants
const double grav = 9.81;
const double degToRadConv = 0.01745329251;
const double radTodegConv = 57.2957795131;
// These below are constants which prevent sin and cos functions from adding to runtime
const double sin60 = 0.8660254038;
const double cos60 = 0.5;
const double sinorcos45 = 0.7071067812;


// Position constants for tests
const double pos1 [2] = {0.75,0.1};
const double pos2 [2] = {0.5, 0.5};
const double pos3 [2] = {0.2, 0.6};


//Class which stores required angles of members based on parallelogram
struct AngleOfMembs {
   double angle1c1;
   double angle2c1;
   double angle3c1;
   double angle1c2;
   double angle2c2;
   double angle3c2;
};


//Class which stores final angles for all three cases
struct TestAngles {
   double test1 [3];
   double test2 [3];
   double test3 [3];
};
//Length of members class
struct LengthOfMembs {
   double l1;
   double l2;
   double l3;
};
//Point class
struct Point {
   double x, y;
};


//Takes in three lengths and three angles in radians along with a true/false print var and returns true if all valid
bool testValues(double l1, double l2, double l3, double q1, double q2, double q3, bool print) {
   if (l1 <= 0, l2 <= 0, l3 <= 0){
       return false;
   }
   if ((l1 + l2 + l3) <= 1){
       return false;
   }
   if ((q1) < 0 || q1 > 180 )
   {
       return false;
   }
   //below ensures that none of the members are below the ground
   if (l1*sin(q1*degToRadConv) < 0)
   {
       return false;
   }
   if (l1*sin(q1*degToRadConv)+ l2*sin(q2*degToRadConv) < 0)
   {
       return false;
   }
   if ((l1*sin(q1*degToRadConv))+(l2*sin(q2*degToRadConv))+(l3*sin(q3*degToRadConv)) < 0){


       return false;
   }
   //below checks that the required positions can be reached
   if ((l1 + l2) <= pow((pow((pos1[1] + (l3 * sin60)), 2) + pow(pos1[0] - (l3 * cos60), 2)), 0.5)){
       return false;


   }
   if ((l1 + l2) <= pow(pow(pos2[1], 2) + pow((pos2[0] - l3), 2), 0.5)){
       return false;


   }
   if ((l1 + l2) <= pow(pow(pos3[1] - (l3 * sinorcos45), 2) + pow(pos3[0] - (l3 * sinorcos45), 2), 0.5)){
       return false;


   }
   //for the sake of testing if the segments overlap q1 cannot equal 90 degrees, if we see that q1 is very close to 90 in the optimization we can calculate that case by hand
   if (q1 == 90){
       return false;
   }
   //the below code uses slopes to check if no overlapping is happening
   Point p1 = {0,0};
   Point p2 = {l1*cos(q1*degToRadConv),l1*sin(q1*degToRadConv)};
   Point p3 = {(l1*cos(q1*degToRadConv))+(l2*cos(q2*degToRadConv)),l1*sin(q1*degToRadConv)+(l2*sin(q2*degToRadConv))};
   Point p4 = { (l1*cos(q1*degToRadConv))+(l2*cos(q2*degToRadConv))+(l3*cos(q3*degToRadConv)),(l1*sin(q1*degToRadConv))+(l2*sin(q2*degToRadConv))+(l3*sin(q3*degToRadConv))};


   if (print){
       cout << "(" << p1.x << "," <<p1.y << ") (" << p2.x << "," <<p2.y << ") ("<< p3.x << "," <<p3.y << ") ("<< p4.x << "," <<p4.y << ")" << endl;
   }
   if (q1 < 90 && p4.y < p2.y && p3.x < p2.x){
       if (p4.y/p4.x <= (p2.y)/(p2.x) && p3.y/p3.x >= (p2.y)/(p2.x) ){
           return false;
       }
       if (p4.y/p4.x <= (p2.y)/(p2.x)  && p3.y/p3.x <= 0){
           return false;
       }
       if (p4.y/p4.x >= (p2.y)/(p2.x) && p3.y/p3.x <= (p2.y)/(p2.x)){
           return false;
       }
   }
   if (q1 > 90 && p2.x < p4.x && p3.y < p2.y && (p4.y)/(p4.x) >= (p3.y)/(p3.x)){


       return false;
   }
   return true;
}


//This function takes in three lengths and three angles and spits out the torque at the base motor
double torque(double l1, double l2, double l3, double q1, double q2, double q3)
{
   //checks if valid, if isn't return ridiculously right torque to cause it to not be saved as the lowest torque
   if (!testValues(l1, l2, l3, q1, q2, q3, false))
       return 9999;
   //torque at base motor due to 5kg weight
   double T4 = (l1*cos(q1*degToRadConv) + (l2*cos(q2*degToRadConv)) + (l3 * cos(q3*degToRadConv))) *grav*5;
   //torque at base motor due to third arm
   double T3 = (l1*cos(q1*degToRadConv) + l2*cos(q2*degToRadConv) + l3 * cos(q3*degToRadConv)/2) *grav*l3;
   //torque at base motor due to second arm
   double T2 = (l1*cos(q1*degToRadConv) + l2*cos(q2*degToRadConv)/2) * grav*2*l2;
   //torque at base motor due to first arm
   double T1 = (l1*l1) * 2 * grav * cos(q1*degToRadConv);
   //return sum of all the torques
   return T1+T2+T3+T4;
}
//takes in the lenght of mem1, mem2 and the final point mem2 needs to reach and spits out two possible scenarios. No error checking for if the angles spat out are valid is done
AngleOfMembs anglesReq(double member1, double member2, double PX, double PY) {
   //distance from origin the two coupled arms are required to reach
   double required = pow(pow(PX,2) + pow(PY,2), 0.5);
   // Calculate angle from triangle to ground, with atan limitations in mind
   double angleOfTriToGroundDeg = atan(PY / PX);
   if (PX < 0){
       angleOfTriToGroundDeg = angleOfTriToGroundDeg + M_PI;
   }


   // cosine law to calculate internal angles
   double B = acos((member1 * member1 + required * required - member2 * member2) / (2 * member1 * required));
   AngleOfMembs result;
   // two cases which angle 1 could be
   result.angle1c1 = (angleOfTriToGroundDeg + B);
   result.angle1c2 = (angleOfTriToGroundDeg - B);


   // two possible endpoints which member 1 could finish at
   double xMidway1 = member1 * cos(result.angle1c1);
   double yMidway1 = member1 * sin (result.angle1c1);
   double xMidway2 = member1 * cos(result.angle1c2);
   double yMidway2 = member1 * sin (result.angle1c2);


   //calculate difference in distances
   double differenceX1 = PX - xMidway1;
   double differenceY1 = PY - yMidway1;
   double differenceX2 = PX - xMidway2;
   double differenceY2 = PY - yMidway2;


   //calculate angles based on differences
   result.angle2c1 = atan(differenceY1/differenceX1);
   result.angle2c2 = atan(differenceY2/differenceX2);


   //account for atan boundaries
   if (differenceX1 < 0 && differenceY1 < 0) {
       result.angle2c1 = M_PI + result.angle2c1;
   } else if (differenceX1 < 0 && differenceY1 > 0) {
       result.angle2c1 = M_PI + result.angle2c1;
   } else if (differenceX1 > 0 && differenceY1 < 0) {
       result.angle2c1 = M_PI*2 + result.angle2c1;
   }
   if (differenceX2 < 0 && differenceY2 < 0) {
       result.angle2c2 = M_PI + result.angle2c2;
   } else if (differenceX2 < 0 && differenceY2 > 0) {
       result.angle2c2 = M_PI + result.angle2c2;
   } else if (differenceX2 > 0 && differenceY2 < 0) {
       result.angle2c2 = M_PI*2 + result.angle2c2;
   }


   //return result
   result.angle1c1 = result.angle1c1 * radTodegConv;
   result.angle1c2 = result.angle1c2 * radTodegConv;
   result.angle2c1 = result.angle2c1 * radTodegConv;
   result.angle2c2 = result.angle2c2 * radTodegConv;
   return result;
}


double calculateCMPTorque (double l1, double l2, double l3, bool print) {
   //save resulting angles
   AngleOfMembs angRESP1 = anglesReq(l1, l2, pos1[0] - (l3 * cos60), pos1[1] + (l3 * sin60));
   AngleOfMembs angRESP2 = anglesReq(l1, l2, (pos2[0] - l3), pos2[1]);
   AngleOfMembs angRESP3 = anglesReq(l1, l2, pos3[0] - (l3 * sinorcos45), pos3[1] - (l3 * sinorcos45));
   //final angles as stated in document for member 3
   angRESP1.angle3c1  = -60;
   angRESP2.angle3c2 = 0;
   angRESP3.angle3c1 = 45;
   angRESP1.angle3c2  = -60;
   angRESP2.angle3c1 = 0;
   angRESP3.angle3c2 = 45;
   //calculate torques for the 3 cases based on the two angle permutations possible
   double torques1[3];
   double torques2[3];
   torques1[0] = torque(l1, l2, l3, angRESP1.angle1c1, angRESP1.angle2c1, angRESP1.angle3c1);
   torques1[1] = torque(l1, l2, l3, angRESP2.angle1c1, angRESP2.angle2c1, angRESP2.angle3c1);
   torques1[2] = torque(l1, l2, l3, angRESP3.angle1c1, angRESP3.angle2c1, angRESP3.angle3c1);
   torques2[0] = torque(l1, l2, l3, angRESP1.angle1c2, angRESP1.angle2c2, angRESP1.angle3c2);
   torques2[1] = torque(l1, l2, l3, angRESP2.angle1c2, angRESP2.angle2c2, angRESP2.angle3c2);
   torques2[2] = torque(l1, l2, l3, angRESP3.angle1c2, angRESP3.angle2c2, angRESP3.angle3c2);
   TestAngles testedAngles;
   //if you need to print out all of the angles save them
   if (print){
       if (torques1[0] < torques2[0]) {
           testedAngles.test1[0] = angRESP1.angle1c1;
           testedAngles.test1[1] = angRESP1.angle2c1;
           testedAngles.test1[2] = angRESP1.angle3c1;
       } else {
           testedAngles.test1[0] = angRESP1.angle1c2;
           testedAngles.test1[1] = angRESP1.angle2c2;
           testedAngles.test1[2] = angRESP1.angle3c2;
       }
       if (torques1[1] < torques2[1]) {
           testedAngles.test2[0] = angRESP2.angle1c1;
           testedAngles.test2[1] = angRESP2.angle2c1;
           testedAngles.test2[2] = angRESP2.angle3c1;
       } else {
           testedAngles.test2[0] = angRESP2.angle1c2;
           testedAngles.test2[1] = angRESP2.angle2c2;
           testedAngles.test2[2] = angRESP2.angle3c2;
       }
       if (torques1[2] < torques2[2]) {
           testedAngles.test3[0] = angRESP3.angle1c1;
           testedAngles.test3[1] = angRESP3.angle2c1;
           testedAngles.test3[2] = angRESP3.angle3c1;
       } else {
           testedAngles.test3[0] = angRESP3.angle1c2;
           testedAngles.test3[1] = angRESP3.angle2c2;
           testedAngles.test3[2] = angRESP3.angle3c2;
       }
       //calculate min of two torques
       double t1 = min(torques1[0], torques2[0]);
       double t2 = min(torques1[1], torques2[1]);
       double t3 = min(torques1[2], torques2[2]);
       // calculate squared square rood of the sum of the torques for the three cases
       double result = pow(pow(t1,2)+pow(t2,2)+pow(t3,2),0.5);
       cout << "T1: " << t1 << " T2: " << t2 << " T3: " << t3 << endl;
       cout << "Test 1: q1: "<< testedAngles.test1[0] << " q2: " << testedAngles.test1[1] << " q3: " << testedAngles.test1[2]<< endl;
       cout << "Test 2: q1:"<< testedAngles.test2[0] << " q2: " << testedAngles.test2[1] << " q3: " << testedAngles.test2[2] << endl;
       cout << "Test 3: q1:"<< testedAngles.test3[0] << " q2: " << testedAngles.test3[1] << " q3: " << testedAngles.test3[2] << endl << endl;


       testValues(l1,l2,l3, testedAngles.test3[0],testedAngles.test3[1], testedAngles.test3[2], true);
       return result;
   }
   else {
       //calculate min of two torques


       double t1 = min(torques1[0], torques2[0]);
       double t2 = min(torques1[1], torques2[1]);
       double t3 = min(torques1[2], torques2[2]);
       double result = pow(pow(t1,2)+pow(t2,2)+pow(t3,2),0.5);
       // calculate squared square rood of the sum of the torques for the three cases


       return result;


   }
}




int main() {
   //code used for debugging
/*
  string newln = "y";
  do {
      double AA, BB, CC, DD;
      cin >> AA >> BB >> CC >> DD;
      AngleOfMembs MYANGLES = anglesReq(AA, BB, CC, DD);
      cout << "Test 1, q1: " << MYANGLES.angle1c1  << " q2: " << MYANGLES.angle2c1
           << " Test 2, q1: " << MYANGLES.angle1c2  << " q2: " << MYANGLES.angle2c2
           << endl;


      cout << "repeat?: " << endl;
      cin >> newln;
  }
  while (newln != "");
*/
/*
   string newln = "y";
   do {
       double a, b, c;
       cout << "Enter the lengths of the three mmbers: " << endl;
       cin >> a >> b >> c;
       double result=calculateCMPTorque(a, b, c, true);
       cout << "Final Torque is: " << result << endl;


       cout << "repeat?: " << endl;
       cin >> newln;
   }
   while (newln != "");
*/
/*LengthOfMembs testing; // testing values that iterate
   testing.l3, testing.l2, testing.l1 = 0;


   LengthOfMembs best; // best value
  //earlier versions of the code had the centres all set to the radius of convergence and as numbers were printed out by the program we made it the centre of the values we wanted to incremement arround while simultaneously decreasing the increment size and the radius of convergence
   double increment = 0.005;
   double max = 5;
   cout << "Warning this will compute " << pow(2*max/increment ,3)*2 << " cases."<< endl;
   double torque = 99999;


   double numOfValidCases = 0;
   double numOfAllCases = 0;
   auto start_time = chrono::high_resolution_clock::now();
   for (testing.l1 = 0; testing.l1 < max; testing.l1+= increment){
       for (testing.l2 = 0; testing.l2 < max; testing.l2+=increment){
           for (testing.l3 = 0; testing.l3 < max; testing.l3+=increment){
               numOfAllCases++;
               //if (testValues(testing.l1, testing.l2, testing.l3, 1, 1, 1)) {
                   double tested = calculateCMPTorque(testing.l1, testing.l2, testing.l3, false);
                   if (tested < torque){
                       torque = tested;
                       best.l1 = testing.l1;
                       best.l2 = testing.l2;
                       best.l3 = testing.l3;
                       //numOfValidCases++;
                //   }


               }


           }


       }
       //console printing to prevent anxiety that the code crashed again and make me question if engineering is right for me.
       if (int(numOfAllCases/pow(max/increment ,3))%10 == 0) {
           auto mid_time = chrono::high_resolution_clock::now();
           auto elapsed_time = chrono::duration_cast<std::chrono::milliseconds>(mid_time - start_time).count();
           double iterations_per_ms = 2*numOfAllCases / elapsed_time; // Calculate iterations per millisecond
           double percent_complete = 100*(numOfAllCases/pow(max/increment ,3)); // Calculate percentage complete
           double iterations_remaining = pow(max/increment ,3) - numOfAllCases; // Calculate remaining iterations
           double min_remaining = int(10*(iterations_remaining / iterations_per_ms) / 60000.0)/10.0; // Calculate remaining time in seconds


           cout << percent_complete << "% complete " << (min_remaining) << "m" << " left" << endl;
           //cout << (numOfAllCases)/elapsed_time << " it/ms " << double(testing.l1*100/max) <<" % complete" << (((1000*numOfAllCases)/elapsed_time))*(pow(max/increment ,3)-numOfAllCases)/60.0 << " min remaining" << endl;




       }


   }
   //print final values and angles.
   cout << endl << "Final Torque is: " << torque << endl;
   cout << "Members: " << best.l1 << " " << best.l2 << " " << best.l3 << endl;
   calculateCMPTorque(best.l1, best.l2, best.l3, true);
   cout << "There were " << numOfAllCases*2 << " cases tested total." << endl;
   return 0;
*/
   //honing portion is below...
   LengthOfMembs testing; // testing values that iterate
   testing.l3, testing.l2, testing.l1 = 0;


   LengthOfMembs best; // best value
   //earlier versions of the code had the centres all set to the radius of convergence and as numbers were printed out by the program we made it the centre of the values we wanted to incremement arround while simultaneously decreasing the increment size and the radius of convergence
   double increment = 0.000001;
   double centre1 = 0.82535;
   double centre2 = 0.88868;
   double centre3 = 0.59067;
   double radiusOfConvergence = 0.0001;
   //cout << "Please input the increment and radius of convergence: " << endl;
   //cin >> increment >> radiusOfConvergence;
   //calculate lowest torque
   cout << "Warning this will compute " << pow(2*radiusOfConvergence/increment ,3)*2 << " cases."<< endl;
   double torque = 99999;


   double numOfValidCases = 0;
   double numOfAllCases = 0;
   auto start_time = chrono::high_resolution_clock::now();
   for (testing.l1 = centre1-radiusOfConvergence; testing.l1 < centre1+radiusOfConvergence; testing.l1+= increment){
       for (testing.l2 = centre2-radiusOfConvergence; testing.l2 < centre2+radiusOfConvergence; testing.l2+=increment){
           for (testing.l3 = centre3-radiusOfConvergence; testing.l3 < centre3+radiusOfConvergence; testing.l3+=increment){
               numOfAllCases++;
               //if (testValues(testing.l1, testing.l2, testing.l3, 1, 1, 1)) {
               double tested = calculateCMPTorque(testing.l1, testing.l2, testing.l3, false);
               if (tested < torque){
                   torque = tested;
                   best.l1 = testing.l1;
                   best.l2 = testing.l2;
                   best.l3 = testing.l3;
                   //numOfValidCases++;
                   //   }


               }


           }


       }
       //console printing to prevent anxiety that the code crashed again and make me question if engineering is right for me.
       if (int(numOfAllCases/pow(2*radiusOfConvergence/increment ,3))%10 == 0) {
           auto mid_time = chrono::high_resolution_clock::now();
           auto elapsed_time = chrono::duration_cast<std::chrono::milliseconds>(mid_time - start_time).count();
           double iterations_per_ms = 2*numOfAllCases / elapsed_time; // Calculate iterations per millisecond
           double percent_complete = 100*(numOfAllCases/pow(2*radiusOfConvergence/increment ,3)); // Calculate percentage complete
           double iterations_remaining = pow(2*radiusOfConvergence/increment ,3) - numOfAllCases; // Calculate remaining iterations
           double min_remaining = int(10*(iterations_remaining / iterations_per_ms) / 60000.0)/10.0; // Calculate remaining time in seconds


           cout << percent_complete << "% complete " << (min_remaining) << "m" << " left" << endl;
           //cout << (numOfAllCases)/elapsed_time << " it/ms " << double(testing.l1*100/max) <<" % complete" << (((1000*numOfAllCases)/elapsed_time))*(pow(max/increment ,3)-numOfAllCases)/60.0 << " min remaining" << endl;




       }


   }
   //print final values and angles.
   cout << endl << "Final Torque is: " << torque << endl;
   cout << "Members: " << best.l1 << " " << best.l2 << " " << best.l3 << endl;
   calculateCMPTorque(best.l1, best.l2, best.l3, true);
   cout << "There were " << numOfAllCases*2 << " cases tested total." << endl;
   return 0;


}
