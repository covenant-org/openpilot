#include "system/camerad/cameras/camera_common.h"
#include "system/camerad/cameras/remote_camera.h"

#include <cassert>

#include "common/params.h"
#include "common/util.h"
#include "common/swaglog.h"

int main(int argc, char *argv[]) {
  // doesn't need RT priority since we're using isolcpus
  int ret = util::set_core_affinity({6});
  assert(ret == 0 || Params().getBool("IsOffroad")); // failure ok while offroad due to offlining cores

  std::string use_remote_camera = util::getenv("REMOTE_CAMERA", "");
  if (use_remote_camera.length() > 0) {
    LOGW("Using remote camera");
    remote_camerad_thread(use_remote_camera);
    return 0;
  }
  camerad_thread();
  return 0;
}
