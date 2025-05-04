// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdio>
#include <cstring>

#include <mujoco/mujoco.h>

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data

// main function
int main(int argc, const char** argv) {
  // Activate MuJoCo license (only needed for older versions or offline activation)
  // mj_activate("mjkey.txt");

  // Load model from file (adjust the path to your XML model)
  char error[1000] = "Could not load XML model";
  mjModel* m = mj_loadXML("../model/car/car.xml", NULL, error, 1000);
  if (!m) {
      printf("Load model error: %s\n", error);
      return 1;
  }

  // Make data structure
  mjData* d = mj_makeData(m);

  // Run a single forward step
  mj_step(m, d);

  // Print base x position as a sanity check
  printf("Base x position: %f\n", d->qpos[0]);

  // Free resources
  mj_deleteData(d);
  mj_deleteModel(m);

  // Deactivate license if needed
  // mj_deactivate();

  return 0;
}
