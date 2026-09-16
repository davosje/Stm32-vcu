/*
 * Bench test: i3LIMClass from the firmware, unchanged, talking to a real BMW
 * i3 LIM through SocketCAN.
 *
 * opmode defaults to MOD_OFF and no charge is ever requested, so the LIM is
 * only ever asked to sit there and answer. Build in test/:
 *
 *   make lim_bench
 *
 * Run: ./lim_bench [seconds] [interface] [opmode] [udc]
 *
 * udc is the pack voltage we claim to have. The LIM checks it against what it
 * measures on the inlet, so it is the quickest way to see whether it is really
 * listening to us.
 */

#include "i3LIM.h"
#include "params.h"
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <map>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace std;

// Normally provided by stm32_vcu.cpp and the error list; not needed here.
const char *errorListString = "";
namespace Param {
void Change(PARAM_NUM) {}
} // namespace Param

class SocketCan : public CanHardware {
public:
  int fd = -1;
  map<uint32_t, int> sent;
  int errors = 0;

  bool Open(const char *name) {
    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0)
      return false;
    ifreq ifr;
    memset(&ifr, 0, sizeof ifr);
    strncpy(ifr.ifr_name, name, IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0)
      return false;
    sockaddr_can addr;
    memset(&addr, 0, sizeof addr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, (sockaddr *)&addr, sizeof addr) < 0)
      return false;
    return fcntl(fd, F_SETFL, O_NONBLOCK) == 0;
  }

  void SetBaudrate(enum baudrates) override {}

  void Send(uint32_t canId, uint32_t data[2], uint8_t len) override {
    can_frame f;
    memset(&f, 0, sizeof f);
    f.can_id = canId;
    f.can_dlc = len > 8 ? 8 : len;
    memcpy(f.data, data, f.can_dlc);
    if (write(fd, &f, sizeof f) == sizeof f)
      sent[canId]++;
    else
      errors++;
  }

private:
  void ConfigureFilters() override {}
};

static volatile sig_atomic_t stop = 0;
static void Stop(int) { stop = 1; }

int main(int argc, char **argv) {
  int seconds = argc > 1 ? atoi(argv[1]) : 600;
  const char *name = argc > 2 ? argv[2] : "can0";
  int opmode = argc > 3 ? atoi(argv[3]) : MOD_OFF;
  float udc = argc > 4 ? atof(argv[4]) : 0;
  static SocketCan bus;
  static i3LIMClass lim;

  if (!bus.Open(name)) {
    perror(name);
    return 1;
  }
  signal(SIGINT, Stop);
  signal(SIGTERM, Stop);

  Param::SetInt(Param::opmode, opmode);
  Param::SetFloat(Param::udc, udc);
  Param::SetFloat(Param::idc, 0);
  Param::SetFloat(Param::Voltspnt, 448.2f);
  Param::SetInt(Param::BattCap, 62000);
  Param::SetInt(Param::SOC, 50);
  Param::SetInt(Param::SOCFC, 80);
  Param::SetInt(Param::CCS_ILim, 125);
  Param::SetFloat(Param::BMS_ChargeLim, 0);
  lim.SetCanInterface(&bus);

  map<uint32_t, int> received;
  map<uint32_t, can_frame> heard;
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  setvbuf(stdout, NULL, _IOLBF, 0);
  printf("lim_bench op %s, %d s, opmode %d, udc %.0f V\n", name, seconds,
         opmode, udc);

  for (long tick = 0; tick < seconds * 100L && !stop; tick++) {
    can_frame f;
    while (read(bus.fd, &f, sizeof f) == sizeof f) {
      uint32_t id = f.can_id & CAN_SFF_MASK;
      uint32_t data[2] = {0, 0};
      memcpy(data, f.data, f.can_dlc > 8 ? 8 : f.can_dlc);
      received[id]++;
      heard[id] = f;
      lim.DecodeCAN(id, data);
    }

    lim.Task10Ms();
    if (tick % 10 == 9) {
      lim.DCFCRequest(false);
      lim.ACRequest(false);
      lim.Task100Ms();
    }
    if (tick % 20 == 19)
      lim.Task200Ms();

    if (tick % 100 == 99) {
      printf("%4lds lim:", (tick + 1) / 100);
      if (received.empty())
        printf(" -");
      for (auto &r : received)
        printf(" %03X x%d", r.first, r.second);
      printf(" | wij:");
      if (bus.sent.empty())
        printf(" -");
      for (auto &s : bus.sent)
        printf(" %03X x%d", s.first, s.second);
      if (heard.count(0x3B4)) {
        printf(" | 3B4=");
        for (int i = 0; i < heard[0x3B4].can_dlc; i++)
          printf("%02X", heard[0x3B4].data[i]);
      }
      printf(" | state=%d cond=%d contactor=%d deur=%d plug=%d pilot=%d/%d"
             " inlaat=%dV ccs=%dV/%dA",
             Param::GetInt(Param::CCS_State), Param::GetInt(Param::CCS_COND),
             Param::GetInt(Param::CCS_Contactor), Param::GetInt(Param::CP_DOOR),
             Param::GetInt(Param::PlugDet), Param::GetInt(Param::PilotTyp),
             Param::GetInt(Param::PilotLim), Param::GetInt(Param::CCS_V_Con),
             Param::GetInt(Param::CCS_V), Param::GetInt(Param::CCS_I_Avail));
      if (bus.errors)
        printf(" | zendfouten=%d", bus.errors);
      printf("\n");
      received.clear();
      bus.sent.clear();
    }

    next.tv_nsec += 10000000;
    if (next.tv_nsec >= 1000000000) {
      next.tv_nsec -= 1000000000;
      next.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
  }
  printf("klaar\n");
  return 0;
}
