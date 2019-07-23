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
    solver.execute(h, 0, 0, 0, 0, false);
  }
  public void calc(double h, double u)
  {
    bound();
    solver.execute(h, u, 0, 0, 0, false);
  }

  public void calc_neural(double h)
  {
    if (Math.abs(solver.x[1]) >= QUARTER_PI/4 || Math.abs(solver.x[2]) >= QUARTER_PI/4) {
      u = out_of_range ? 0 : control_constant * brain.think(solver.x)[0]; //not very elegant solution, but it allows to maintain consistency in the neuralnetwork class
      bound();
      solver.execute(h, u, 0, 0, 0, false); // object without stabilisation
    } else {
      bound();
      solver.execute(h, 0, 0, 0, 0, true); // object with upward stabilisation
    }
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
          fitness += h * (8 + 8/(4*abs[4+i] + 1) + 4/(abs[0] + 1) + 2/(4*abs[3] + 1));  //eight point zone + octa speed bonus + quad zero position bonus + double cart velocity bonus
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

  public void reset()
  {
    solver.x[0]=x1;
    solver.x[1]=x2;
    solver.x[2]=x3;
    solver.x[3]=x4;
    solver.x[4]=x5;
    solver.x[5]=x6;
    fitness = 0;
    out_of_range = false;
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
