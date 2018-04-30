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

/*initial values*/
final double x1 = 0;
final double x2 = PI;
final double x3 = PI;
final double x4 = 0;
final double x5 = 0;
final double x6 = 0;
final double max_force = 10;
final double max_weight = 5;
final double max_bias = PI;

final int stroke_weight = 5;
boolean recording = false;
final int dip_count = 70;
final double generation_time = 5; //time of generation existence [s]
double actual_generation_time = 0;
int generation_counter = 1;
dip[] pendulums = new dip[dip_count];
dip[] next_generation_pendulums = new dip[dip_count];
double u=0;
boolean all_out_of_range;

/* neural network constnts */
final int input_count = 6;
final int hidden_count = 10;
final int output_count = 1;

PFont font;

void setup()
{
  size(600, 400); //x width = dip.scale*gantry+100  dip.scale=300
  fill(255);
  stroke(255);
  strokeWeight(stroke_weight);
  rectMode(CENTER);
  frameRate(50);
  font = createFont("font.ttf", 20);
  for (int i = 0; i < dip_count; i++)
  {
    pendulums[i] = new dip(x1, x2, x3, x4, x5, x6);
  }
}

int k = 30; //how many iterations by one presentation
double h = (double)1/(k*100); //time step for 50 frame per second and 2 times slow motion

void draw()
{
  background(0);

  /* updating the pendulums */
  for (int j =0; j<dip_count; j++)
  {
    for (int i = 0; i<k; i++) {
      pendulums[j].calc_neural(h); //solving next position
    }
    pendulums[j].show();
  }
  textSize(20);
  textFont(font);
  fill(255);
  text(String.format("GENERATION: %d", generation_counter), 40, 20);
  text(String.format("TIME: %.3f", (float)actual_generation_time), 40, 45);
  actual_generation_time += h * k;

  all_out_of_range = true;
  for (int i = 0; i < dip_count; i++)
  {
    if (!pendulums[i].out_of_range)
    {
      all_out_of_range = false;
      break;
    }
  }

  if (actual_generation_time >= generation_time || all_out_of_range)
  {
    next_generation();
    actual_generation_time = 0;
    generation_counter++;
  }



  /* drawing the edges */
  pushMatrix();
  translate(width/2, height/2);
  stroke(255);
  line((float)(pendulums[0].scale*gantry/2) + pendulums[0].c_w/2 + stroke_weight, 0, (float)(pendulums[0].scale*gantry/2) + pendulums[0].c_w/2 + stroke_weight, 10);
  line(-(float)(pendulums[0].scale*gantry/2) - pendulums[0].c_w/2 - stroke_weight, 0, -(float)(pendulums[0].scale*gantry/2) - pendulums[0].c_w/2 - stroke_weight, 10);
  popMatrix();

  /*recording control*/
  if (recording) {
    saveFrame("capture/DIP####.png");
    fill(255, 0, 0);
    stroke(255, 150, 150);
    ellipse(20, 25, 20, 20);
  }
}

/* steering pendulum on arrows, use dip.calc(h,u); in draw function*/

void keyPressed() {
  if (key == 'r' || key == 'R') // start/stop recording
  {
    recording = !recording;
  }
  if (key == 'p' || key == 'P') // play again
  {
    actual_generation_time = 0;
    for (int i =0; i<dip_count; i++)
    {
      pendulums[i].solver.x[0]=x1;
      pendulums[i].solver.x[1]=x2;
      pendulums[i].solver.x[2]=x3;
      pendulums[i].solver.x[3]=x4;
      pendulums[i].solver.x[4]=x5;
      pendulums[i].solver.x[5]=x6;
      pendulums[i].out_of_range = false;
    }
  }
  if (key == 'n' || key == 'N') // new simulation
  {
    for (int i =0; i<dip_count; i++)
    {
      actual_generation_time = 0;
      generation_counter = 1;
      pendulums[i] = new dip(x1, x2, x3, x4, x5, x6);
    }
  }
  //if (key == CODED) {
  //  if (keyCode == LEFT) {
  //    u = -2;
  //  } else if (keyCode == RIGHT) {
  //    u = 2; // some ammount of force
  //  }
  //} else {
  //  u = 0;
  //}
}
void keyReleased() {
  u = 0;
}

class dip
{
  final int c_w = 20;      //width of cart
  final int c_h = 10;      //height of cart
  final int scale = 300; //the scale factor for scaling up the real parameter (described in meters) to suitable value in pixels

  boolean out_of_range = false;
  double control_constant;
  double[] Ep = new double[2];  //potential energy
  double[] Ek = new double[2];  //kinetic energy
  double[] E = new double[2];   //all energy in system (initial value [0], value in process [1])
  double fitness = 0;

  RK4 solver;
  neuralnetwork brain;

  public dip(double x1, double x2, double x3, double x4, double x5, double x6) //passing inivtial conditions to solver
  {
    solver = new RK4(x1, x2, x3, x4, x5, x6);
    brain = new neuralnetwork(input_count, hidden_count, output_count, true);
    control_constant = Math.random() * max_force;
    Ep[0] =  m2 * g * l2 * Math.cos(solver.x[2]) + (m1 * g * l1 + m2 * g * L1) * Math.cos(solver.x[1]);
    Ek[0] = 0.5* m0* Math.pow(solver.x[3], 2) + 0.5* m1*Math.pow(solver.x[3], 2) + m1*solver.x[3]* solver.x[4] *l1*Math.cos(solver.x[1])+ 0.5* m1*Math.pow(solver.x[4], 2) * Math.pow(l1, 2) + 0.5 *I1* Math.pow(solver.x[4], 2) + 0.5* m2*Math.pow(solver.x[3], 2) + 0.5 *m2*Math.pow(solver.x[4], 2)* Math.pow(L1, 2) + 0.5 *m2*Math.pow(solver.x[5], 2) *Math.pow(l2, 2) + m2 * solver.x[3] * solver.x[4] * L1 * Math.cos(solver.x[1]) + m2 * solver.x[3] * solver.x[5] * l2 * Math.cos(solver.x[2]) + m2 * solver.x[4] * L1 * solver.x[5] * l2 * Math.cos(solver.x[1] - solver.x[2]) + 0.5 * I2 * Math.pow(solver.x[5], 2);
    E[0] = Ep[0] + Ek[0];
  }

  public dip(double x1, double x2, double x3, double x4, double x5, double x6, double control_constant, neuralnetwork brain) //passing inivtial conditions to solver
  {
    solver = new RK4(x1, x2, x3, x4, x5, x6);
    this.brain = brain;
    this.control_constant = control_constant;
    Ep[0] =  m2 * g * l2 * Math.cos(solver.x[2]) + (m1 * g * l1 + m2 * g * L1) * Math.cos(solver.x[1]);
    Ek[0] = 0.5* m0* Math.pow(solver.x[3], 2) + 0.5* m1*Math.pow(solver.x[3], 2) + m1*solver.x[3]* solver.x[4] *l1*Math.cos(solver.x[1])+ 0.5* m1*Math.pow(solver.x[4], 2) * Math.pow(l1, 2) + 0.5 *I1* Math.pow(solver.x[4], 2) + 0.5* m2*Math.pow(solver.x[3], 2) + 0.5 *m2*Math.pow(solver.x[4], 2)* Math.pow(L1, 2) + 0.5 *m2*Math.pow(solver.x[5], 2) *Math.pow(l2, 2) + m2 * solver.x[3] * solver.x[4] * L1 * Math.cos(solver.x[1]) + m2 * solver.x[3] * solver.x[5] * l2 * Math.cos(solver.x[2]) + m2 * solver.x[4] * L1 * solver.x[5] * l2 * Math.cos(solver.x[1] - solver.x[2]) + 0.5 * I2 * Math.pow(solver.x[5], 2);
    E[0] = Ep[0] + Ek[0];
  }

  public void calc(double h)
  {
    bound();
    solver.execute(h, 0, 0, 0, 0);
  }
  public void calc(double h, double u)
  {
    bound();
    solver.execute(h, u, 0, 0, 0);
  }

  public void calc_neural(double h)
  {
    u = out_of_range ? 0 : control_constant * brain.think(solver.x)[0]; //not very elegant solution, but it allows to maintain consistency in the neuralnetwork class
    bound();
    solver.execute(h, u, 0, 0, 0);
    calc_fitness(h);
  }

  private void calc_fitness(double h)
  {
    
    /* old fitness function */
    //if (!out_of_range)
    //{
    //  double[] abs = new double[6];
    //  for (int i = 0; i<6; i++)
    //    abs[i] = Math.abs(solver.x[i]);

    //  /* fitness function */
    //  if (abs[1] > HALF_PI || abs[2] > HALF_PI)
    //    fitness += 0;
    //  else if (abs[1] > QUARTER_PI || abs[2] > QUARTER_PI)
    //    fitness += h/(abs[1] + abs[2]);

    //  else if ( abs[1] > 0.1 || abs[2] > 0.1 )
    //    fitness += h/((abs[1] + abs[2])/2 + abs[0] + abs[3]);

    //  else
    //    fitness += 2 * h/(abs[0] + 3*abs[3] + abs[4] + abs[5] + h);
    //}


    /* new fitness function */
    if (!out_of_range)
    {
      double[] abs = new double[6];
      for (int i = 0; i<6; i++)
        abs[i] = Math.abs(solver.x[i]);

      for (int i=0; i<2; i++)
      {
        if (abs[1+i] >= 3*QUARTER_PI)
          fitness += 0;                              //no reward in this zone
        else if (abs[1+i] >= HALF_PI)
          fitness += h * 1/2;                        //half point zone
        else if (abs[1+i] >= QUARTER_PI)
          fitness += h * (1 + 1/(abs[4+i] + 1));     //one point zone + speed bonus
        else if (abs[1+i] >= QUARTER_PI/2)
          fitness += h * (2 + 2/(abs[4+i] + 1) + 1/(abs[0] + 1));     //two point zone + double speed bonus + zero position bonus
        else if (abs[1+i] >= QUARTER_PI/4)
          fitness += h * (4 + 4/(abs[4+i] + 1) + 2/(abs[0] + 1) + 1/(abs[3] + 1));  //four point zone + quad speed bonus + double zero position bonus + cart velocity bonus
        else
          fitness += h * (8 + 8/(abs[4+i] + 1) + 4/(abs[0] + 1) + 2/(abs[3] + 1));  //eight point zone + octa speed bonus + quad zero position bonus + double cart velocity bonus
        }
      }
    }

    /* stopping on the edge. It is mainly a visual effect (not real physics effect), but also ensures that the pendulum remains on the screen */
    private boolean bound()
  {
    if (solver.x[0] > gantry/2) {
      solver.x[0] = gantry/2;
      solver.x[3] = 0;
      out_of_range = true;
    } else if (solver.x[0] < -gantry/2) {
      solver.x[0] = -gantry/2;
      solver.x[3] = 0;
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
    //In the absence of friction and control("u") we can find out a solver error (in this case energy should stay the same all the time)
    println( "Initial Energy:", (float)E[0], "Actual Energy:", (float)E[1] );
  }

  public void show()
  {
    float[] temp = new float[3];
    pushMatrix();
    translate(width/2, height/2);
    if (out_of_range) { 
      stroke(80, 80, 80, 127);
      fill(80, 80, 80, 127);
    } else {
      stroke(80, 80, 255);
      fill(80, 80, 255);
    }
    rect(temp[0]=(float)solver.x[0] * scale, 0, c_w, c_h);
    translate(temp[0], 0);
    if (out_of_range) stroke(127, 127, 127, 127);
    else stroke(127, 127, 255);
    line(0.0, 0.0, temp[1]=(float)(L1*Math.sin(PI - solver.x[1]) * scale), temp[2]=(float)(L1*Math.cos(PI - solver.x[1])) * scale);
    translate(temp[1], temp[2]);
    if (out_of_range) stroke(200, 200, 200, 127);
    else stroke(200, 200, 255);
    line(0.0, 0.0, (float)(L2*Math.sin(PI - solver.x[2]) * scale), (float)(L2*Math.cos(PI - solver.x[2])) * scale);
    popMatrix();
  }
}
