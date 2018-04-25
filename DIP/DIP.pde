final double g = 9.81;   //przyspieszenie ziemskie [m/s^2]
final double m0 = 0.530168;   //masa wózka [kg]
final double m1 = 0.18369;   //masa pierwszej części wahadła [kg]
final double L1 = 0.232039;   //długość pierwszej części wahadła [m]
final double l1 = 0.14527;  //odległość do środka masy od punktu zaczepu pierwszej części wahadła [m]
final double I1 = 1655.092 / 1000000;  //[kg*m^2] moment bezwladnosci ramienia pierwszego 
final double L2 = 0.257;   //długość drugiej części wahadła [m]
final double I2 = 13518.257 / (1000*100*100); //[kg*m^2] m2 = 137.952 / 1000; %[kg]
final double m2 = 137.952 / 1000; //[kg]
final double l2 = 12.041 / 100; //[m]
final double eta0 = 0;   //współczynnik tarcia pomiędzy wózkiem a podłożem [kg/s]
final double eta1 = 0;   //współczynnik tarcia przegubu pierwszego [(kg*m^2)/s]
final double eta2 = 0;   //współczynnik tarcia przegubu drugiego [(kg*m^2)/s]
/* wspolczynniki */

final double A   =  m0+m1+m2;
final double B1 =  m1*l1 + m2*L1;
final double B2 =  m2*l2;
final double C   =  m1*l1*l1 + m2*L1*L1 + I1;
final double D1 =  m2*L1*l2;
final double D2 =  m1*l1*g + m2*L1*g;
final double E   =  m2*l2*l2 +I2;
final double F =  m2*l2*g;

dip zbyszek;

void setup()
{
  size(600, 200);
  frameRate(30);
  fill(255);
  stroke(255);
  strokeWeight(5);
  rectMode(CENTER);
  
  zbyszek = new dip(0, PI/2, 0, 0, 0, 0);
}

void draw()
{
  background(0);
  zbyszek.calc(1/30);
  zbyszek.show();
}

class rk4
{
  public double[] x = new double[6];
  double[][] k = new double[6][4];

  public rk4(double x1, double x2, double x3, double x4, double x5, double x6) //solver initial values
  {
    x[0] = x1;
    x[1] = x2;
    x[2] = x3;
    x[3] = x4;
    x[4] = x5;
    x[5] = x6;
  }

  private double f1(double x4)
  {
    return x4;
  }
  private double f2(double x5)
  {
    return x5;
  }
  private double f3(double x6)
  {
    return x6;
  }
  private double f4(double x2, double x3, double x4, double x5, double x6, double u, double z0, double z1, double z2)
  {
    return ((u + z0 -x4* eta0 + B1 * Math.sin(x2)* Math.pow(x5, 2) + B2 * Math.sin(x3)* Math.pow(x6, 2)) * ( C * E - Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) ) + (z1 - x5* eta1 - (x5 - x6)* eta2 - D1 * Math.sin(x2-x3)* Math.pow(x6, 2) + D2 * Math.sin(x2))* ( B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - B1 * Math.cos(x2)* E )  + ( z2 -(x6 - x5)* eta2 + D1 * Math.sin(x2 - x3)* Math.pow(x5, 2) + F * Math.sin(x3))* ( B1 * Math.cos(x2)* D1 * Math.cos(x2 - x3) - B2 * Math.cos(x3)* C ) )/ (A * C * E + 2 * B1 * Math.cos(x2)* B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) * E - A * Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) - Math.pow(B2, 2) * Math.pow((Math.cos(x3)), 2) * C);
  }
  private double f5(double x2, double x3, double x4, double x5, double x6, double u, double z0, double z1, double z2)
  {
    return ((u + z0 -x4* eta0 + B1 * Math.sin(x2)* Math.pow(x5, 2) + B2 * Math.sin(x3) * Math.pow(x6, 2)) * ( B2 * Math.cos(x3)* D1 * Math.cos(x2 - x3) - B1 * Math.cos(x2)* E )  + ( z1 -x5* eta1 - (x5 - x6)*eta2 - D1 * Math.sin(x2-x3)* Math.pow(x6, 2) + D2 * Math.sin(x2))* ( A * E - Math.pow(B2, 2)* Math.pow((Math.cos(x3)), 2)) + (z2-(x6 - x5)* eta2 + D1 * Math.sin(x2 - x3)* Math.pow(x5, 2) + F * Math.sin(x3))* ( B1 * Math.cos(x2)* B2 * Math.cos(x3) - A * D1 * Math.cos(x2 - x3))) /  (A * C * E + 2 * B1 * Math.cos(x2)* B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) * E - A * Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) - Math.pow(B2, 2) * Math.pow((Math.cos(x3)), 2) * C);
  }
  private double f6(double x2, double x3, double x4, double x5, double x6, double u, double z0, double z1, double z2)
  {
    return ((u + z0 -x4* eta0 + B1 * Math.sin(x2)* Math.pow(x5, 2) + B2 * Math.sin(x3)* Math.pow(x6, 2)) * ( B1 * Math.cos(x2)* D1 * Math.cos(x2 - x3) - B2 * Math.cos(x3)* C ) + ( z1 -x5* eta1 - (x5 - x6)*eta2 - D1 * Math.sin(x2-x3)* Math.pow(x6, 2) + D2 * Math.sin(x2))* ( B1 * Math.cos(x2)* B2 * Math.cos(x3) - A * D1* Math.cos(x2 - x3)) + ( z2 -(x6 - x5)* eta2 + D1 * Math.sin(x2 - x3)* Math.pow(x5, 2) + F * Math.sin(x3))* ( A * C - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) )) /  (A * C * E + 2 * B1 * Math.cos(x2)* B2 * Math.cos(x3)* D1* Math.cos(x2 - x3) - Math.pow(B1, 2) * Math.pow((Math.cos(x2)), 2) * E - A * Math.pow(D1, 2) * Math.pow((Math.cos(x2 - x3)), 2) - Math.pow(B2, 2) * Math.pow((Math.cos(x3)), 2) * C);
  }
  void execute(double h, double u, double z0, double z1, double z2)
  {
    k[0][0] = h * f1(x[3]);
    k[1][0] = h * f2(x[4]);
    k[2][0] = h * f3(x[5]);
    k[3][0] = h * f4(x[1], x[2], x[3], x[4], x[5], u, z0, z1, z2);
    k[4][0] = h * f5(x[1], x[2], x[3], x[4], x[5], u, z0, z1, z2);
    k[5][0] = h * f6(x[1], x[2], x[3], x[4], x[5], u, z0, z1, z2);

    k[0][1] = h * f1(x[3]+(k[3][0]/2));
    k[1][1] = h * f2(x[4]+(k[4][0]/2));
    k[2][1] = h * f3(x[5]+(k[5][0]/2));
    k[3][1] = h * f4(x[1]+(k[1][0]/2), x[2]+(k[2][0]/2), x[3]+(k[3][0]/2), x[4]+(k[4][0]/2), x[5]+(k[5][0]/2), u, z0, z1, z2);
    k[4][1] = h * f5(x[1]+(k[1][0]/2), x[2]+(k[2][0]/2), x[3]+(k[3][0]/2), x[4]+(k[4][0]/2), x[5]+(k[5][0]/2), u, z0, z1, z2);
    k[5][1] = h * f6(x[1]+(k[1][0]/2), x[2]+(k[2][0]/2), x[3]+(k[3][0]/2), x[4]+(k[4][0]/2), x[5]+(k[5][0]/2), u, z0, z1, z2);

    k[0][2] = h * f1(x[3]+(k[3][1]/2));
    k[1][2] = h * f2(x[4]+(k[4][1]/2));
    k[2][2] = h * f3(x[5]+(k[5][1]/2));
    k[3][2] = h * f4(x[1]+(k[1][1]/2), x[2]+(k[2][1]/2), x[3]+(k[3][1]/2), x[4]+(k[4][1]/2), x[5]+(k[5][1]/2), u, z0, z1, z2);
    k[4][2] = h * f5(x[1]+(k[1][1]/2), x[2]+(k[2][1]/2), x[3]+(k[3][1]/2), x[4]+(k[4][1]/2), x[5]+(k[5][1]/2), u, z0, z1, z2);
    k[5][2] = h * f6(x[1]+(k[1][1]/2), x[2]+(k[2][1]/2), x[3]+(k[3][1]/2), x[4]+(k[4][1]/2), x[5]+(k[5][1]/2), u, z0, z1, z2);

    k[0][3] = h * f1(x[3]+k[3][2]);
    k[1][3] = h * f2(x[4]+k[4][2]);
    k[2][3] = h * f3(x[5]+k[5][2]);
    k[3][3] = h * f4(x[1]+k[1][2], x[2]+k[2][2], x[3]+k[3][2], x[4]+k[4][2], x[5]+k[5][2], u, z0, z1, z2);
    k[4][3] = h * f5(x[1]+k[1][2], x[2]+k[2][2], x[3]+k[3][2], x[4]+k[4][2], x[5]+k[5][2], u, z0, z1, z2);
    k[5][3] = h * f6(x[1]+k[1][2], x[2]+k[2][2], x[3]+k[3][2], x[4]+k[4][2], x[5]+k[5][2], u, z0, z1, z2);

    for (int i = 0; i<6; i++)
    {
      x[i] = x[i] + (k[i][0] + 2 * k[i][1] + 2 * k[i][2] + k[i][3])/6;
    }
  }
}

class dip
{
  final int w = 30;
  final int h = 10;
  final int scale = 300;
  rk4 solver;
  public dip(double x1, double x2, double x3, double x4, double x5, double x6)
  {
    solver = new rk4(x1, x2, x3, x4, x5, x6);
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
  public void show()
  {
    float[] temp = new float[2];
    pushMatrix();
    translate(width/2, height/2);
    stroke(255);
    rect((float)solver.x[0], 0, w, h);
    stroke(127,127,255);
    line(0.0, 0.0, temp[0]=(float)(scale*L1*Math.sin(PI - solver.x[1])), temp[1]=(float)(scale*L1*Math.cos(PI - solver.x[1])));
    translate(temp[0], temp[1]);
    stroke(0,0,255);
    line(0.0, 0.0, temp[0]=(float)(scale*L2*Math.sin(PI - solver.x[2])), temp[1]=(float)(scale*L2*Math.cos(PI - solver.x[2])));
    popMatrix();
  }
}
