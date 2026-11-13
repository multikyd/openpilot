#include "system/camerad/cameras/camera_common.h"

#include <cassert>

#include "common/params.h"
#include "common/util.h"

int main(int argc, char *argv[]) {
  // card also runs on CPU6 at FIFO 53. Core isolation alone no longer keeps
  // camera delivery from waiting behind the CAN/control thread.
  int ret = util::set_core_affinity({6});
  assert(ret == 0 || Params().getBool("IsOffroad")); // failure ok while offroad due to offlining cores
#ifdef __COMMA_HARDWARE__
  // Frame handlers block on camera events; their short delivery work must run
  // before card's next batch. Keep both processes and their rates unchanged.
  ret = util::set_realtime_priority(54);
  assert(ret == 0);
#endif

  camerad_thread();
  return 0;
}
