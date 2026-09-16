/*
 * Wake test on the bench: MGgen2Charger from the firmware, unchanged, talking
 * to a real charger through SocketCAN.
 *
 * opmode stays MOD_OFF and ControlCharge is never asked to charge, so the
 * class can only go Asleep -> Waking -> Standby and back. Build in test/:
 *
 *   g++ -I../include -I../libopeninv/include -o mg_bench mg_bench.cpp \
 *       my_string.o params.o MGgen2Frames.o canhardware.o MGgen2Charger.o
 *
 * Run: ./mg_bench [seconds] [interface]
 */

#include "MGgen2Charger.h"
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
  map<uint32_t, can_frame> last;
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
    if (write(fd, &f, sizeof f) == sizeof f) {
      sent[canId]++;
      last[canId] = f;
    } else {
      errors++;
    }
  }

private:
  void ConfigureFilters() override {}
};

static volatile sig_atomic_t stop = 0;
static void Stop(int) { stop = 1; }

static void Hex(const can_frame &f) {
  for (int i = 0; i < f.can_dlc; i++)
    printf("%02X", f.data[i]);
}

int main(int argc, char **argv) {
  int seconds = argc > 1 ? atoi(argv[1]) : 600;
  const char *name = argc > 2 ? argv[2] : "can0";
  static SocketCan bus;
  static MGgen2Charger charger;

  if (!bus.Open(name)) {
    perror(name);
    return 1;
  }
  signal(SIGINT, Stop);
  signal(SIGTERM, Stop);

  Param::SetInt(Param::opmode, MOD_OFF);
  Param::SetFloat(Param::Voltspnt, 448.2f);
  Param::SetFloat(Param::udc, 0);
  Param::SetFloat(Param::ChgTemp, -99);
  charger.SetCanInterface(&bus);

  map<uint32_t, int> received;
  map<uint32_t, can_frame> heard;
  timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  setvbuf(stdout, NULL, _IOLBF, 0);
  printf("mg_bench op %s, %d s, opmode MOD_OFF\n", name, seconds);

  for (long tick = 0; tick < seconds * 100L && !stop; tick++) {
    can_frame f;
    while (read(bus.fd, &f, sizeof f) == sizeof f) {
      uint32_t id = f.can_id & CAN_SFF_MASK;
      uint32_t data[2] = {0, 0};
      memcpy(data, f.data, f.can_dlc > 8 ? 8 : f.can_dlc);
      received[id]++;
      heard[id] = f;
      charger.DecodeCAN(id, data);
    }

    charger.Task10Ms();
    if (tick % 10 == 9) {
      charger.ControlCharge(false, false);
      charger.Task100Ms();
    }

    if (tick % 100 == 99) {
      printf("%4lds lader:", (tick + 1) / 100);
      if (received.empty())
        printf(" -");
      for (auto &r : received)
        printf(" %03X x%d", r.first, r.second);
      printf(" | wij:");
      if (bus.sent.empty())
        printf(" -");
      for (auto &s : bus.sent)
        printf(" %03X x%d", s.first, s.second);
      if (bus.last.count(0x297)) {
        printf(" | 297=");
        Hex(bus.last[0x297]);
      }
      if (bus.last.count(0x33F)) {
        printf(" 33F=");
        Hex(bus.last[0x33F]);
      }
      if (heard.count(0x324)) {
        printf(" | 324=");
        Hex(heard[0x324]);
      }
      if (heard.count(0x33B)) {
        printf(" 33B=");
        Hex(heard[0x33B]);
      }
      printf(" | ChgTemp=%.0f", Param::GetFloat(Param::ChgTemp));
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
