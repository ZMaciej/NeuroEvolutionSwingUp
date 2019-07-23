final double g = 9.81;                        //gravity constant [m/s^2]
final double m0 = 0.530168;                   //cart mass [kg]
final double m1 = 0.18669;                    //mass of first arm [kg]
final double L1 = 0.232039;                   //lenght of first arm [m]
final double l1 = 0.15927;                    //distance to center of mass from first arm joint point [m]
final double I1 = 14675.631/(1000*100*100);   //inertia of first arm [kg*m^2] 
final double L2 = 0.260;                      //lenght of second arm [m]
final double I2 = 13518.257/(1000*100*100);   //inertia of second arm [kg*m^2]
final double m2 = 137.952 / 1000;             // mass of second arm[kg]
final double l2 = 12.041 / 100;               //length of second arm[m]
final double eta0 = 0.01;                     //cart viscous friction constant [kg/s]                 //recommended value is about 0.01 
final double eta1 = 0.001;                    //first joint viscous friction constant [(kg*m^2)/s]    //recommended value is about 0.001
final double eta2 = 0.001;                    //second joint viscous friction constant [(kg*m^2)/s]     //recommended value is about 0.001
final double gantry = 1.0;                    //lenght of gantry [m]

/* some constants to simplify the differential equations */

final double A   =  m0+m1+m2;
final double B1  =  m1*l1 + m2*L1;
final double B2  =  m2*l2;
final double C   =  m1*l1*l1 + m2*L1*L1 + I1;
final double D1  =  m2*L1*l2;
final double D2  =  m1*l1*g + m2*L1*g;
final double E   =  m2*l2*l2 +I2;
final double F   =  m2*l2*g;

/*initial values*/
final double x1 = 0;
final double x2 = PI;
final double x3 = PI;
final double x4 = 0;
final double x5 = 0;
final double x6 = 0;
final double max_force = 15;
final double max_weight = 5;
final double max_bias = PI;
/* control gains */
double[] K = {10, -259.7565, 309.8422, 8.3819, -0.7261, 27.0203};
