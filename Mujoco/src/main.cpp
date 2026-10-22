// Build-verification stub: loads a trivial MuJoCo model and steps the
// simulation headlessly. This will be replaced by the biped walking scene.
#include <mujoco/mujoco.h>

#include <cstdio>
#include <string>

int main() {
  char error[1000] = "";
  std::string model_path = std::string(MUJOCO_MODEL_DIR) + "/g1/g1.xml";

  mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Failed to load model '%s': %s\n", model_path.c_str(), error);
    return 1;
  }

  mjData* data = mj_makeData(model);

  for (int step = 0; step < 1000; ++step) {
    mj_step(model, data);
  }

  std::printf("MuJoCo %s OK — simulated %.3f s, hinge angle = %.4f rad\n",
              mj_versionString(), data->time, data->qpos[0]);

  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}
