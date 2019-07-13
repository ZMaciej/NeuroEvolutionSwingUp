/* class of neural network with only one hidden layer available
 activation function is ReLU on hidden and tanh on output */

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

  neuralnetwork(int input_count, int hidden_count, int output_count, boolean randomize)
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

    if (randomize)
      randomization(max_weight, max_bias); //this values are only for testing right now
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
          temp += (input[j] + (add_noise?random(-0.001,0.001):0.0)) * weights_ih[i][j];
        }
        hidden[i] = temp + biases_ih[i];
      }
      ReLU(hidden);

      // activate( weights_ho * (input + biases_ho) )
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
  void randomization( double weights_range, double biases_range)
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
