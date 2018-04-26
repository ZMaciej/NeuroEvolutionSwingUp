final double g = 9.81;                        //gravity constant [m/s^2]
final double m0 = 0.530168;                   //cart mass [kg]
final double m1 = 0.18369;                    //mass of first arm [kg]
final double L1 = 0.232039;                   //lenght of first arm [m]
final double l1 = 0.14527;                    //distance to center of mass from first arm joint point [m]
final double I1 = 1655.092 / 1000000;         //inertia of first arm [kg*m^2] 
final double L2 = 0.257;                      //lenght of second arm [m]
final double I2 = 13518.257 / (1000*100*100); //inertia of first arm [kg*m^2]
final double m2 = 137.952 / 1000;             // mass of second arm[kg]
final double l2 = 12.041 / 100;               //length of second arm[m]
final double eta0 = 0;                        //cart viscous friction constant [kg/s]                 //recommended value is about 0.01 
final double eta1 = 0;                        //first joint viscous friction constant [(kg*m^2)/s]    //recommended value is about 0.001
final double eta2 = 0;                      //second joint viscous friction constant [(kg*m^2)/s]     //recommended value is about 0.001

/* some constants to simplify the differential equations */

final double A   =  m0+m1+m2;
final double B1  =  m1*l1 + m2*L1;
final double B2  =  m2*l2;
final double C   =  m1*l1*l1 + m2*L1*L1 + I1;
final double D1  =  m2*L1*l2;
final double D2  =  m1*l1*g + m2*L1*g;
final double E   =  m2*l2*l2 +I2;
final double F   =  m2*l2*g;

dip pendulum;

void setup()
{
  size(600, 400);
  fill(255);
  stroke(255);
  strokeWeight(5);
  rectMode(CENTER);
  frameRate(50);

  pendulum = new dip(0, 0.00001, 0, 0, 0, 0);
}

void draw()
{
  background(0);
  int k = 15; //how many iterations by one presentation
  for (int i = 0; i<k; i++) {
    pendulum.calc((double)1/(k*50)); //solving next position
  }
  pendulum.show();
  pendulum.energy_update();
  println( pendulum.E[0], pendulum.E[1] );
}

class dip
{
  final int w = 20;      //width of cart
  final int h = 10;      //height of cart
  final int scale = 300; //the scale factor for scaling up the real parameter (described in meters) to suitable value in pixels

  double[] Ep = new double[2];  //potential energy
  double[] Ek = new double[2];  //kinetic energy
  double[] E = new double[2];   //all energy in system (initial value [0], value in process [1])

  RK4 solver;

  public dip(double x1, double x2, double x3, double x4, double x5, double x6) //passing inivtial conditions to solver
  {
    solver = new RK4(x1, x2, x3, x4, x5, x6);
    Ep[0] =  m2 * g * l2 * Math.cos(solver.x[2]) + (m1 * g * l1 + m2 * g * L1) * Math.cos(solver.x[1]);
    Ek[0] = 0.5* m0* Math.pow(solver.x[3], 2) + 0.5* m1*Math.pow(solver.x[3], 2) + m1*solver.x[3]* solver.x[4] *l1*Math.cos(solver.x[1])+ 0.5* m1*Math.pow(solver.x[4], 2) * Math.pow(l1, 2) + 0.5 *I1* Math.pow(solver.x[4], 2) + 0.5* m2*Math.pow(solver.x[3], 2) + 0.5 *m2*Math.pow(solver.x[4], 2)* Math.pow(L1, 2) + 0.5 *m2*Math.pow(solver.x[5], 2) *Math.pow(l2, 2) + m2 * solver.x[3] * solver.x[4] * L1 * Math.cos(solver.x[1]) + m2 * solver.x[3] * solver.x[5] * l2 * Math.cos(solver.x[2]) + m2 * solver.x[4] * L1 * solver.x[5] * l2 * Math.cos(solver.x[1] - solver.x[2]) + 0.5 * I2 * Math.pow(solver.x[5], 2);
    E[0] = Ep[0] + Ek[0];
  }


  public void calc(double h)
  {
    solver.execute(h, 0, 0, 0, 0);
  }
  public void calc(double h, double u)
  {
    solver.execute(h, u, 0, 0, 0);
  }
  public void calc(double h, double u, double z0, double z1, double z2)
  {
    solver.execute(h, u, z0, z1, z2);
  }
  public void energy_update()
  {
    Ep[1] =  m2 * g * l2 * Math.cos(solver.x[2]) + (m1 * g * l1 + m2 * g * L1) * Math.cos(solver.x[1]);
    Ek[1] = 0.5* m0* Math.pow(solver.x[3], 2) + 0.5* m1*Math.pow(solver.x[3], 2) + m1*solver.x[3]* solver.x[4] *l1*Math.cos(solver.x[1])+ 0.5* m1*Math.pow(solver.x[4], 2) * Math.pow(l1, 2) + 0.5 *I1* Math.pow(solver.x[4], 2) + 0.5* m2*Math.pow(solver.x[3], 2) + 0.5 *m2*Math.pow(solver.x[4], 2)* Math.pow(L1, 2) + 0.5 *m2*Math.pow(solver.x[5], 2) *Math.pow(l2, 2) + m2 * solver.x[3] * solver.x[4] * L1 * Math.cos(solver.x[1]) + m2 * solver.x[3] * solver.x[5] * l2 * Math.cos(solver.x[2]) + m2 * solver.x[4] * L1 * solver.x[5] * l2 * Math.cos(solver.x[1] - solver.x[2]) + 0.5 * I2 * Math.pow(solver.x[5], 2);
    E[1] = Ep[1] + Ek[1];
  }

  public void show()
  {
    float[] temp = new float[3];
    pushMatrix();
    translate(width/2, height/2);
    stroke(80, 80, 255);
    fill(80, 80, 255);
    rect(temp[0]=(float)solver.x[0] * scale, 0, w, h);
    translate(temp[0], 0);
    stroke(127, 127, 255);
    line(0.0, 0.0, temp[1]=(float)(L1*Math.sin(PI - solver.x[1]) * scale), temp[2]=(float)(L1*Math.cos(PI - solver.x[1])) * scale);
    translate(temp[1], temp[2]);
    stroke(200, 200, 255);
    line(0.0, 0.0, (float)(L2*Math.sin(PI - solver.x[2]) * scale), (float)(L2*Math.cos(PI - solver.x[2])) * scale);
    popMatrix();
  }
}

class RK4
{
  public double[] x = new double[6];
  double[][] k = new double[6][4];

  public RK4(double x1, double x2, double x3, double x4, double x5, double x6) //solver initial values
  {
    x[0] = x1;
    x[1] = x2;
    x[2] = x3;
    x[3] = x4;
    x[4] = x5;
    x[5] = x6;
  }

  /*
   indexed functions of DIP dynamics, described as
   x1'=f1(x,u,z)
   x2'=f2(x,u,z)
   x3'=f3(x,u,z)
   x4'=f4(x,u,z)
   x5'=f5(x,u,z)
   x6'=f6(x,u,z)
   */
  private double f(int i, double x2, double x3, double x4, double x5, double x6, double u, double z0, double z1, double z2)
  {
    switch(i)
    {
    case 0: 
      return x4;
    case 1: 
      return x5;
    case 2: 
      return x6;
    case 3: 
      return ((u + z0 -x4* eta0 + B1 * Math.sin(x2)* Math.pow(x5, 2) + B2 * Math.sin(x3)* Math.pow(x6, 2)) * ( C * E - Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) ) + (z1 - x5* eta1 - (x5 - x6)* eta2 - D1 * Math.sin(x2-x3)* Math.pow(x6, 2) + D2 * Math.sin(x2))* ( B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - B1 * Math.cos(x2)* E )  + ( z2 -(x6 - x5)* eta2 + D1 * Math.sin(x2 - x3)* Math.pow(x5, 2) + F * Math.sin(x3))* ( B1 * Math.cos(x2)* D1 * Math.cos(x2 - x3) - B2 * Math.cos(x3)* C ) )/ (A * C * E + 2 * B1 * Math.cos(x2)* B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) * E - A * Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) - Math.pow(B2, 2) * Math.pow((Math.cos(x3)), 2) * C);
    case 4: 
      return ((u + z0 -x4* eta0 + B1 * Math.sin(x2)* Math.pow(x5, 2) + B2 * Math.sin(x3) * Math.pow(x6, 2)) * ( B2 * Math.cos(x3)* D1 * Math.cos(x2 - x3) - B1 * Math.cos(x2)* E )  + ( z1 -x5* eta1 - (x5 - x6)*eta2 - D1 * Math.sin(x2-x3)* Math.pow(x6, 2) + D2 * Math.sin(x2))* ( A * E - Math.pow(B2, 2)* Math.pow((Math.cos(x3)), 2)) + (z2-(x6 - x5)* eta2 + D1 * Math.sin(x2 - x3)* Math.pow(x5, 2) + F * Math.sin(x3))* ( B1 * Math.cos(x2)* B2 * Math.cos(x3) - A * D1 * Math.cos(x2 - x3))) /  (A * C * E + 2 * B1 * Math.cos(x2)* B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) * E - A * Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) - Math.pow(B2, 2) * Math.pow((Math.cos(x3)), 2) * C);
    case 5: 
      return ((u + z0 -x4* eta0 + B1 * Math.sin(x2)* Math.pow(x5, 2) + B2 * Math.sin(x3)* Math.pow(x6, 2)) * ( B1 * Math.cos(x2)* D1 * Math.cos(x2 - x3) - B2 * Math.cos(x3)* C ) + ( z1 -x5* eta1 - (x5 - x6)*eta2 - D1 * Math.sin(x2-x3)* Math.pow(x6, 2) + D2 * Math.sin(x2))* ( B1 * Math.cos(x2)* B2 * Math.cos(x3) - A * D1* Math.cos(x2 - x3)) + ( z2 -(x6 - x5)* eta2 + D1 * Math.sin(x2 - x3)* Math.pow(x5, 2) + F * Math.sin(x3))* ( A * C - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) )) /  (A * C * E + 2 * B1 * Math.cos(x2)* B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) * E - A * Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) - Math.pow(B2, 2) * Math.pow((Math.cos(x3)), 2) * C);
    }
    return 0;
  }

  /*  executing Runge–Kutta 4 algorithm to find next position of pendulum */

  void execute(double h, double u, double z0, double z1, double z2)
  {
    int d = 2; //denominator in second and third step of RK4
    boolean z = false; //adding k/d value in second to fourth step of rk4
    for (int j = 0; j<4; j++)
      for (int i = 0; i<6; i++)  //iterating through system of ODE's
      {
        if (j>0)      //second step of RK4
          z = true;
        if (j>2)      //fourth step of RK4
          d=1;
        k[i][j] = h * f(i, x[1] + (z ? (k[1][j-1]/d) : 0), x[2] + (z ? (k[2][j-1]/d) : 0), x[3] + (z ? (k[3][j-1]/d) : 0), x[4] + (z ? (k[4][j-1]/d) : 0), x[5] + (z ? (k[5][j-1]/d) : 0), u, z0, z1, z2); //RK4 main equation, modified to system of ODE's
      }
    for (int i = 0; i<6; i++)
    {
      x[i] = x[i] + (k[i][0] + 2 * k[i][1] + 2 * k[i][2] + k[i][3])/6;
    }
  }
}
