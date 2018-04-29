class RK4
{
  double[] x = new double[6];
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
    
    for (int i = 1; i<3; i++)
    {
      x[i] = ((x[i]+PI) % TWO_PI) - PI; //modifying the angle range to -PI to PI
    }
  }
}
