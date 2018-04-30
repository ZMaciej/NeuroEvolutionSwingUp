void next_generation()
{
  int non_zero_count = normalize_fitness();
  double rand;

  for (int i = 0; i < dip_count; i++)
  {
    if (non_zero_count > 4) {
      rand = Math.random();
      if (rand >= 0.5)
        pick_and_cross(i); //50% chance of crossing

      else if (rand >= 0.1)
        pick_tweak(i);  //40% chance of tweaking

      else
        mutatant(i); //10% chance of new random pendulum
    } else if (non_zero_count > 0) {
      rand = Math.random();
      if (rand >= 0.5)
        pick_tweak(i);  //50% chance of tweaking
      else
        mutatant(i); //50% chance of new random pendulum
    } else {
      mutatant(i);
    }
  }

  for (int i = 0; i < dip_count; i++)
    pendulums[i] = next_generation_pendulums[i];
}

void pick_tweak(int i)
{
  dip parent = null;
  int x;
  boolean picked;

  picked = false;
  do {
    x = (int)(Math.random()*dip_count);
    if (Math.random() <= pendulums[x].fitness)
    {
      parent = pendulums[x];
      picked = true;
    }
  } while (!picked);
  next_generation_pendulums[i] = tweak(parent, max_weight * 0.1, max_bias * 0.1, max_force * 0.1);
}

dip tweak(dip parent, double weight_tweak_range, double bias_tweak_range, double control_constant_tweak_range)
{
  dip child;
  neuralnetwork brain = new neuralnetwork(input_count, hidden_count, output_count, false); // making a new brain (with no randomization) for a child
  double control_constant;

  /* tweaking weights */
  for (int i = 0; i < brain.weights_ih.length; i++)
    for (int j = 0; j < brain.weights_ih[0].length; j++)
    {
      brain.weights_ih[i][j] = parent.brain.weights_ih[i][j] + ((Math.random()*2)-1) * weight_tweak_range ;
    }

  for (int i = 0; i < brain.weights_ho.length; i++)
    for (int j = 0; j < brain.weights_ho[0].length; j++)
    {
      brain.weights_ho[i][j] = parent.brain.weights_ho[i][j] + ((Math.random()*2)-1) * weight_tweak_range;
    }

  /* tweaking biases */
  for (int i = 0; i < brain.biases_ih.length; i++)
  {
    brain.biases_ih[i] = parent.brain.biases_ih[i] + ((Math.random()*2)-1) * bias_tweak_range;
  }

  for (int i = 0; i < brain.biases_ho.length; i++)
  {
    brain.biases_ho[i] = parent.brain.biases_ho[i] + ((Math.random()*2)-1) * bias_tweak_range;
  }

  /* tweaking control constant */
  control_constant = parent.control_constant + ((Math.random()*2)-1) * control_constant_tweak_range;

  child = new dip(x1, x2, x3, x4, x5, x6, control_constant, brain);

  return child;
}

void mutatant(int i)
{
  next_generation_pendulums[i] = new dip(x1, x2, x3, x4, x5, x6);
}

void pick_and_cross(int i)
{
  dip parent_a = null;
  int a;
  dip parent_b = null;
  int b;
  boolean picked;

  picked = false;
  do {
    a = (int)(Math.random()*dip_count);
    if (Math.random() <= pendulums[a].fitness)
    {
      parent_a = pendulums[a];
      picked = true;
    }
  } while (!picked);
  picked = false;
  do {
    b = (int)(Math.random()*dip_count);
    if (a!=b && (Math.random() <= pendulums[b].fitness))
    {
      parent_b = pendulums[b];
      picked = true;
    }
  } while (!picked);
  next_generation_pendulums[i] = cross(parent_a, parent_b);
}

dip cross(dip a, dip b)
{
  dip child;
  neuralnetwork brain = new neuralnetwork(input_count, hidden_count, output_count, false); // making a new brain (with no randomization) for a child
  double rand;
  double control_constant;

  /* crossing weights */
  for (int i = 0; i < brain.weights_ih.length; i++)
    for (int j = 0; j < brain.weights_ih[0].length; j++)
    {
      rand = Math.random();
      brain.weights_ih[i][j] = rand * a.brain.weights_ih[i][j] + (1 - rand) * b.brain.weights_ih[i][j];
    }

  for (int i = 0; i < brain.weights_ho.length; i++)
    for (int j = 0; j < brain.weights_ho[0].length; j++)
    {
      rand = Math.random();
      brain.weights_ho[i][j] = rand * a.brain.weights_ho[i][j] + (1 - rand) * b.brain.weights_ho[i][j];
    }

  /* crossing biases */
  for (int i = 0; i < brain.biases_ih.length; i++)
  {
    rand = Math.random();
    brain.biases_ih[i] = rand * a.brain.biases_ih[i] + (1 - rand) * b.brain.biases_ih[i];
  }

  for (int i = 0; i < brain.biases_ho.length; i++)
  {
    rand = Math.random();
    brain.biases_ho[i] = rand * a.brain.biases_ho[i] + (1 - rand) * b.brain.biases_ho[i];
  }

  /* crossing control constant */
  rand = Math.random();
  control_constant = rand * a.control_constant + (1 - rand) * b.control_constant;

  child = new dip(x1, x2, x3, x4, x5, x6, control_constant, brain);

  return child;
}

int normalize_fitness()
{
  int non_zero = 0;
  double max_fit = 0;
  for (int i =0; i < dip_count; i++)
  {
    if (pendulums[i].fitness > 0)
      non_zero++;
    max_fit = pendulums[i].fitness > max_fit ? pendulums[i].fitness : max_fit;
  }
  for (int i =0; i < dip_count; i++)
  {
    pendulums[i].fitness /= max_fit;
  }
  return non_zero;
}
