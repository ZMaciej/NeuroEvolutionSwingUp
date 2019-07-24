import com.hamoid.*; //library for video recording

VideoExport VideoOut;
boolean recording=false;
boolean startRecording = false;
boolean stopRecording = false;

final int stroke_weight = 5;
final int dip_count = 70; //how many pendulums in one generation
double generation_time = 0;
final double[] generation_times = {2, 3, 5};
final int[] generation_times_change = {15, 30};
/* time of generation existence [s] the time is described as the simulation world time not the real world time 
 generation_times parameter is time
 generation_times_change parameter is the number of generations to which the corresponding time applies
 
 example:
 generation_times = {5, 3, 2};
 generation_times_change = {10, 15};
 time 5s is applied to 10th generation
 time 3s is applied to 15th generation
 time 2s is applied to above 15th generation
 */
double actual_generation_time = 0;
int generation_counter = 1;
dip[] pendulums = new dip[dip_count];
dip[] next_generation_pendulums = new dip[dip_count];
dip best;
double best_fit = 0;
double u=0;
boolean all_out_of_range;
boolean force_next_generation = false;
boolean show_best = false;
boolean show_info = false;
boolean add_noise = false;

/* neural network constants */
final int input_count = 6; //6 states of pendulum
final int hidden_count = 10;  //some arbitrary value
final int output_count = 1; //one control force applied to cart

PFont font;
PrintWriter logger;

void setup()
{
  size(600, 400); //x width = dip.scale*gantry+100  dip.scale=300
  VideoOut = new VideoExport(this); //startMovie, saveFrame, endMovie
  VideoOut.setFrameRate(50);
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
  best = new dip(x1, x2, x3, x4, x5, x6);
}

int k = 30; //how many iterations by one presentation
double h = (double)1/(k*100); //time step for 50 frame per second and 2 times slow motion

void draw()
{
  background(0);
  textSize(20);
  textFont(font);
  fill(255);
  text(String.format("GENERATION: %d", generation_counter), 40, 20);
  text(String.format("TIME: %.3f", (float)actual_generation_time), 40, 45);

  if (!show_best) {
    /* updating the pendulums */
    for (int j =0; j<dip_count; j++)
    {
      for (int i = 0; i<k; i++) 
      {    //k - how many iterations by one presentation
        pendulums[j].calc_neural(h); //solving next position
        
      }
      pendulums[j].show();
    }
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

    generation_time = generation_times[generation_times.length - 1];
    for (int i = 0; i<generation_times_change.length; i++)
    {
      if (generation_times_change[i] >= generation_counter)
      {
        generation_time = generation_times[i];
        break;
      }
    }

    if (actual_generation_time >= generation_time || all_out_of_range || force_next_generation)
    {
      next_generation();
      actual_generation_time = 0;
      generation_counter++;
      force_next_generation = false;
    }
  }
  /* showing best */
  else
  {
    for (int i = 0; i<k; i++) 
    {    //k - how many iterations by one presentation
      best.calc_neural(h); //solving next position
      logger.println(
      str((float)best.solver.x[0]) + "," + str((float)best.solver.x[1]) + "," + str((float)best.solver.x[2]) + "," +
      str((float)best.solver.x[3]) + "," + str((float)best.solver.x[4]) + "," + str((float)best.solver.x[5]) + ",");
    }

    best.show();
    actual_generation_time += h * k;
    
    if(actual_generation_time > 1)
    {
      logger.flush();
      logger.close();
    }
  }

  /* drawing the edges */
  pushMatrix();
  translate(width/2, height/2);
  stroke(255);
  line((float)(pendulums[0].scale*gantry/2) + pendulums[0].c_w/2 + stroke_weight, 0, (float)(pendulums[0].scale*gantry/2) + pendulums[0].c_w/2 + stroke_weight, 10);
  line(-(float)(pendulums[0].scale*gantry/2) - pendulums[0].c_w/2 - stroke_weight, 0, -(float)(pendulums[0].scale*gantry/2) - pendulums[0].c_w/2 - stroke_weight, 10);
  popMatrix();

  /*recording control*/

  if (startRecording)
  {
    String name = String.valueOf(year())+"-"+String.valueOf(month())+"-"+String.valueOf(day())+"-"+String.valueOf(hour())+"-"+String.valueOf(minute())+"-"+String.valueOf(second())+".mp4";
    VideoOut.setMovieFileName(name);
    VideoOut.startMovie();
    recording = true;
    println("video started");
    startRecording = false;
  }
  if (recording)
  {
    VideoOut.saveFrame();
    fill(255, 0, 0);
    stroke(255, 150, 150);
    ellipse(20, 25, 20, 20);
  }
  if (stopRecording)
  {
    recording = false;
    VideoOut.endMovie();
    println("video ended");
    stopRecording = false;
  }

  /*info*/
  if (show_info)
  {
    textSize(20);
    textFont(font);
    fill(255);
    text(String.format("X - reset program"), 300, 20);
    text(String.format("B - show, hide best"), 300, 45);
    text(String.format("P - play again current generation"), 300, 70);
    text(String.format("N - force next generation"), 300, 95);
    text(String.format("R - record"), 300, 120);
    if (add_noise) {
      text(String.format("S - remove noise"), 300, 145);
    } else {
      text(String.format("S - add noise"), 300, 145);
    }
  }
}

void keyPressed() {
  if (key == 'r' || key == 'R') // start/stop recording
  {
    if (!recording)
      startRecording = true; 
    else
      stopRecording = true;
  }
  if (key == 'p' || key == 'P') // play again actual generation
  {
    if (show_best)
    {
      actual_generation_time = 0;
      best.reset();
    } else
    {
      actual_generation_time = 0;
      for (int i =0; i<dip_count; i++)
      {
        pendulums[i].reset();
      }
    }
  }
  if (key == 'n' || key == 'N') // next generation
  {
    if (actual_generation_time > 0.5 && !show_best) force_next_generation = true;
  }
  if (key == 'b' || key == 'B')
  {
    show_best = !show_best;
    if (show_best)
    {
      logger = createWriter("state" + str(best.objectCounter) + ".csv");
      actual_generation_time = 0;
      best.reset();
    } else
    {
      actual_generation_time = 0;
      for (int i =0; i<dip_count; i++)
      {
        pendulums[i].reset();
      }
    }
  }
  if (key == 'x' || key == 'X') // reset everything
  {
    if (!show_best)
    {
      actual_generation_time = 0;
      generation_counter = 1;
      for (int i = 0; i<dip_count; i++)
      {
        pendulums[i] = new dip(x1, x2, x3, x4, x5, x6);
      }
    }
  }
  if (key == 'i' || key == 'I') // info
  {
    show_info = true;
  }
  if (key == 's' || key == 'S') // noise
  {
    add_noise = !add_noise;
  }


  /* steering pendulum on arrows, use dip.calc(h,u); in draw function*/
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
  if (key == 'i' || key == 'I') // info
  {
    show_info = false;
  }
}
