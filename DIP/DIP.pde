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
final double eta1 = 0.001;                    //first joint viscous friction constant [(kg*m^2)/s]    //recommended value is about 0.001
final double eta2 = 0.001;                    //second joint viscous friction constant [(kg*m^2)/s]     //recommended value is about 0.001
final double gantry = 1.5;                    //lenght of gantry [m]

/* some constants to simplify the differential equations */

final double A   =  m0+m1+m2;
final double B1  =  m1*l1 + m2*L1;
final double B2  =  m2*l2;
final double C   =  m1*l1*l1 + m2*L1*L1 + I1;
final double D1  =  m2*L1*l2;
final double D2  =  m1*l1*g + m2*L1*g;
final double E   =  m2*l2*l2 +I2;
final double F   =  m2*l2*g;

final int stroke_weight = 5;
dip pendulum;
double u=0;

void setup()
{
  size(600, 400);
  fill(255);
  stroke(255);
  strokeWeight(stroke_weight);
  rectMode(CENTER);
  frameRate(50);

  pendulum = new dip(0, PI, PI, 0, 0, 0, 10);
}

void draw()
{
  background(0);
  int k = 15; //how many iterations by one presentation
  for (int i = 0; i<k; i++) {
    pendulum.calc_neural((double)1/(k*100)); //solving next position
  }
  pendulum.show();
  //pendulum.energy();
}

/* steering pendulum on arrows, use dip.calc(h,u); in draw function*/

//void keyPressed() {
//  if (key == CODED) {
//    if (keyCode == LEFT) {
//      u = -2;
//    } else if (keyCode == RIGHT) {
//      u = 2;
//    }
//  } else {
//    u = 0;
//  }
//}
//void keyReleased() {
//  u = 0;
//}

class dip
{
  final int w = 20;      //width of cart
  final int h = 10;      //height of cart
  final int scale = 300; //the scale factor for scaling up the real parameter (described in meters) to suitable value in pixels
  final double springiness = 0.5; //value in range 0 - 1, it is a springiness of borders at gantry

  boolean out_of_range = false;
  double control_constant;
  double[] Ep = new double[2];  //potential energy
  double[] Ek = new double[2];  //kinetic energy
  double[] E = new double[2];   //all energy in system (initial value [0], value in process [1])

  RK4 solver;
  neuralnetwork brain;

  public dip(double x1, double x2, double x3, double x4, double x5, double x6, double control_range) //passing inivtial conditions to solver
  {
    solver = new RK4(x1, x2, x3, x4, x5, x6);
    brain = new neuralnetwork(6, 10, 1);
    control_constant = Math.random() * control_range;
    Ep[0] =  m2 * g * l2 * Math.cos(solver.x[2]) + (m1 * g * l1 + m2 * g * L1) * Math.cos(solver.x[1]);
    Ek[0] = 0.5* m0* Math.pow(solver.x[3], 2) + 0.5* m1*Math.pow(solver.x[3], 2) + m1*solver.x[3]* solver.x[4] *l1*Math.cos(solver.x[1])+ 0.5* m1*Math.pow(solver.x[4], 2) * Math.pow(l1, 2) + 0.5 *I1* Math.pow(solver.x[4], 2) + 0.5* m2*Math.pow(solver.x[3], 2) + 0.5 *m2*Math.pow(solver.x[4], 2)* Math.pow(L1, 2) + 0.5 *m2*Math.pow(solver.x[5], 2) *Math.pow(l2, 2) + m2 * solver.x[3] * solver.x[4] * L1 * Math.cos(solver.x[1]) + m2 * solver.x[3] * solver.x[5] * l2 * Math.cos(solver.x[2]) + m2 * solver.x[4] * L1 * solver.x[5] * l2 * Math.cos(solver.x[1] - solver.x[2]) + 0.5 * I2 * Math.pow(solver.x[5], 2);
    E[0] = Ep[0] + Ek[0];
  }


  public void calc(double h)
  {
    bound_amendment();
    solver.execute(h, 0, 0, 0, 0);
  }
  public void calc(double h, double u)
  {
    bound_amendment();
    solver.execute(h, u, 0, 0, 0);
  }

  public void calc_neural(double h)
  {
    double[] x_mod;
    x_mod = solver.x.clone();
    for (int i = 1; i<3; i++)
    {
      x_mod[i] = ((x_mod[i]+PI) % TWO_PI) - PI; //modifying the angle range to -PI to PI
    }
    u = control_constant * brain.think(x_mod)[0]; //not very elegant solution, but it allows to maintain consistency in the neuralnetwork class
    bound_amendment();
    solver.execute(h, u, 0, 0, 0);
  }

  /* bouncing off the edge. It is mainly a visual effect (not real physics effect), but also ensures that the pendulum remains on the screen 
  it works badly when the pendulum is constantly trying to go one way */

  private boolean bound_amendment()
  {
    if (solver.x[0] > gantry/2) {
      solver.x[3] =  -springiness * solver.x[3];
      solver.x[0] = gantry/2;
      out_of_range = true;
      calculate_energy();
    } else if (solver.x[0] < -gantry/2) {
      solver.x[3] =  -springiness * solver.x[3];
      solver.x[0] = -gantry/2;
      out_of_range = true;
    }
    return out_of_range;
  }

  private void calculate_energy()
  {
    Ep[1] =  m2 * g * l2 * Math.cos(solver.x[2]) + (m1 * g * l1 + m2 * g * L1) * Math.cos(solver.x[1]);
    Ek[1] = 0.5* m0* Math.pow(solver.x[3], 2) + 0.5* m1*Math.pow(solver.x[3], 2) + m1*solver.x[3]* solver.x[4] *l1*Math.cos(solver.x[1])+ 0.5* m1*Math.pow(solver.x[4], 2) * Math.pow(l1, 2) + 0.5 *I1* Math.pow(solver.x[4], 2) + 0.5* m2*Math.pow(solver.x[3], 2) + 0.5 *m2*Math.pow(solver.x[4], 2)* Math.pow(L1, 2) + 0.5 *m2*Math.pow(solver.x[5], 2) *Math.pow(l2, 2) + m2 * solver.x[3] * solver.x[4] * L1 * Math.cos(solver.x[1]) + m2 * solver.x[3] * solver.x[5] * l2 * Math.cos(solver.x[2]) + m2 * solver.x[4] * L1 * solver.x[5] * l2 * Math.cos(solver.x[1] - solver.x[2]) + 0.5 * I2 * Math.pow(solver.x[5], 2);
    E[1] = Ep[1] + Ek[1];
  }

  public void energy()
  {
    calculate_energy();
    //In the absence of friction we can find out a solver error (in this case energy should stay the same all the time)
    println( "Initial Energy:", (float)E[0], "Actual Energy:", (float)E[1] );
  }

  public void show()
  {
    float[] temp = new float[3];
    pushMatrix();
    translate(width/2, height/2);
    stroke(255);
    line((float)(scale*gantry/2)+w/2+stroke_weight, 3, (float)(scale*gantry/2)+w/2+stroke_weight, 10);
    line(-(float)(scale*gantry/2)-w/2-stroke_weight, 3, -(float)(scale*gantry/2)-w/2-stroke_weight, 10);
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


/* class of neural network with only one hidden layer available
 activation function is ReLU and tanh */

class neuralnetwork
{
  double[][] weights_ih;
  double[][] weights_ho;
  //double[] input;
  double[] hidden;
  double[] output;
  double[] biases_ih;
  double[] biases_ho;
  int input_count = 0;
  int hidden_count = 0;
  int output_count = 0;

  neuralnetwork(int input_count, int hidden_count, int output_count)
  {
    weights_ih = new double[hidden_count][input_count];
    weights_ho = new double[output_count][hidden_count];
    //input = new double[input_count];
    hidden = new double[hidden_count];
    output = new double[output_count];
    biases_ih = new double[hidden_count];
    biases_ho = new double[output_count];
    this.input_count = input_count;
    this.hidden_count = hidden_count;
    this.output_count = output_count;

    randomize(1, 1); //this values are only for testing right now
  }

  double[] think(double[] input)
  {
    double temp;
    if (input.length == input_count) {
      // activate( weights_ih * input + biases_ih )
      for (int i = 0; i<hidden_count; i++)
      {
        temp = 0;
        for (int j = 0; j<input_count; j++)
        {
          temp += input[j] * weights_ih[i][j];
        }
        hidden[i] = temp + biases_ih[i];
      }
      ReLU(hidden);

      // activate( weights_ho * input + biases_ho )
      for (int i = 0; i<output_count; i++)
      {
        temp = 0;
        for (int j = 0; j<hidden_count; j++)
        {
          temp += hidden[j] * weights_ho[i][j];
        }
        output[i] = temp + biases_ho[i];
      }
      tanh(output);
    }
    return output;
  }

  void randomize_weights(double weights_range)
  {
    for (int i = 0; i < weights_ih.length; i++)
      for (int j = 0; j < weights_ih[0].length; j++)
      {
        weights_ih[i][j] = ((Math.random()*2)-1) * weights_range;
      }

    for (int i = 0; i < weights_ho.length; i++)
      for (int j = 0; j < weights_ho[0].length; j++)
      {
        weights_ho[i][j] = ((Math.random()*2)-1) * weights_range;
      }
  }
  void randomize_biases(double biases_range)
  {
    for (int i = 0; i < biases_ih.length; i++)
      biases_ih[i] = ((Math.random()*2)-1) * biases_range;

    for (int i = 0; i < biases_ho.length; i++)
      biases_ho[i] = ((Math.random()*2)-1) * biases_range;
  }
  void randomize( double weights_range, double biases_range)
  {
    randomize_weights(weights_range);
    randomize_biases(biases_range);
  }

  /*ReLU activation*/

  private void ReLU(double[] arr)
  {
    for (int i = 0; i < arr.length; i++)
      if (arr[i]<=0) arr[i]=0;
  }


  /* tanh activation*/

  private void tanh(double[] arr)
  {
    for (int i = 0; i < arr.length; i++)
      arr[i] = Math.tanh(arr[i]);
  }
}


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
  }
}
