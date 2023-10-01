/*
 * File: loadmodel.cpp
 * Created Date: Wednesday August 16th 2023
 * Author: Steven Atkinson (steven@atkinson.mn)
 */

// Test loading a model

#include <stdlib.h>
#include "NAM/dsp.h"

int main(int argc, char* argv[])
{
  if (argc > 1)
  {
    char* modelPath = argv[1];

    fprintf(stderr, "Loading model [%s]\n", modelPath);

    auto model = get_dsp(modelPath);

    if (model != nullptr)
    {
      fprintf(stderr, "Model loaded successfully\n");
    }
    else
    {
      fprintf(stderr, "Failed to load model\n");

      exit(1);
    }
  }
  else
  {
    fprintf(stderr, "Usage: loadmodel <model_path>\n");
  }

  exit(0);
}
