#include "opendbc/safety/tests/libsafety/safety.c"
#include <string.h>
static unsigned tests = 0U;
static uint8_t counts[2048];
#define CHECK(x) do { tests++; if (!(x)) { printf("FAIL line %d: %s\n", __LINE__, #x); return 1; } } while (0)
static CANPacket_t pkt(unsigned addr, unsigned bus, unsigned dlc) {
  CANPacket_t p = {0}; p.addr = addr; p.bus = bus; p.data_len_code = dlc; p.fd = 1U;
  p.data[2] = counts[addr]++; return p;
}
static void crc(CANPacket_t *p) {
  unsigned c = hyundai_common_canfd_compute_checksum(p); p->data[0] = c & 255U; p->data[1] = c >> 8U;
}
static CANPacket_t command(unsigned bus, unsigned value) {
  CANPacket_t p = pkt(0x16a, bus, 13); p.data[16] = value << 5U; crc(&p); return p;
}
static void reset(unsigned flags) {
  memset(counts, 0, sizeof(counts)); set_safety_hooks(SAFETY_HYUNDAI_CANFD, flags | 1U);
  init_tests(); set_timer(1000000U); set_controls_allowed(false);
}
static void feed(unsigned bus, unsigned speed, bool brake, bool gas, unsigned mode) {
  CANPacket_t wheel = pkt(0xa0, bus, 12);
  for (int i=8; i<16; i+=2) { wheel.data[i]=speed & 255U; wheel.data[i+1]=speed >> 8U; }
  crc(&wheel); safety_rx_hook(&wheel);
  CANPacket_t tcs = pkt(0x175, bus, 12); tcs.data[10] = brake ? 2U : 0U; crc(&tcs); safety_rx_hook(&tcs);
  CANPacket_t accel = pkt(0x35, bus, 13); accel.data[5] = gas ? 5U : 0U; crc(&accel); safety_rx_hook(&accel);
  CANPacket_t spas = pkt(0x165, bus, 12); spas.data[11] = mode << 2U; crc(&spas); safety_rx_hook(&spas);
}
int main(void) {
  const unsigned variants[] = {0U, 8U, 16U, 24U, 16U|128U, 24U|128U, 4U, 4U|8U, 4U|16U, 4U|24U};
  for (unsigned v=0; v<sizeof(variants)/sizeof(variants[0]); v++) {
    unsigned f=variants[v], bus=(f&16U) && !(f&8U) ? 1U : 0U;
    reset(f); int original_rx = current_safety_config.rx_checks_len;
    for (unsigned action=0; action<8; action++) {
      CANPacket_t p=command(bus, action); CHECK(!safety_tx_hook(&p));
    }
    reset(f|1024U);
    if ((f&16U) && (f&8U) && !(f&4U)) {
      CHECK(current_safety_config.rx_checks_len == original_rx);
      feed(bus,0,true,false,3); set_controls_allowed(false);
      CANPacket_t rejected=command(bus,3); CHECK(!safety_tx_hook(&rejected)); continue;
    }
    CHECK(current_safety_config.rx_checks_len == original_rx);
    CANPacket_t p=command(bus,3); CHECK(!safety_tx_hook(&p)); // no sensors yet
    feed(bus,0,true,false,3); set_controls_allowed(false);
    p=command(bus,3); CHECK(safety_tx_hook(&p));
    p=command(bus,4); CHECK(!safety_tx_hook(&p)); // rate limit
    set_timer(1020000U); feed(bus,0,true,false,3); set_controls_allowed(false);
    p=command(bus,4); CHECK(safety_tx_hook(&p));
    for (unsigned wrong=0; wrong<3; wrong++) if (wrong!=bus) {
      set_timer(timer_cnt+20000U); feed(bus,0,true,false,3); set_controls_allowed(false);
      p=command(wrong,3); CHECK(!safety_tx_hook(&p));
    }
    for (unsigned action=1; action<8; action++) if (action!=3 && action!=4) {
      p=command(bus,action); CHECK(!safety_tx_hook(&p));
    }
    p=command(bus,0); CHECK(safety_tx_hook(&p));
    CANPacket_t steering=pkt(0x165,bus,12); crc(&steering); CHECK(!safety_tx_hook(&steering));
    for (int i=3; i<32; i++) { // every reserved byte/bit must stay zero
      p=command(bus,0); p.data[i] |= 1U; crc(&p); CHECK(!safety_tx_hook(&p));
    }
    p=command(bus,3); p.data[0]^=1U; CHECK(!safety_tx_hook(&p));
    p=command(bus,3); p.data_len_code=12; CHECK(!safety_tx_hook(&p));
    p=command(bus,3); p.data_len_code=14; CHECK(!safety_tx_hook(&p));
    reset(f|1024U); feed(bus,2,true,false,3); set_controls_allowed(false);
    p=command(bus,3); CHECK(!safety_tx_hook(&p)); // even tiny wheel motion
    reset(f|1024U); feed(bus,0,false,false,3); p=command(bus,3); CHECK(!safety_tx_hook(&p));
    reset(f|1024U); feed(bus,0,true,true,3); set_controls_allowed(false); p=command(bus,3); CHECK(!safety_tx_hook(&p));
    reset(f|1024U); feed(bus,0,true,false,3); set_controls_allowed(true); p=command(bus,3); CHECK(!safety_tx_hook(&p));
    reset(f|1024U); feed(bus,0,true,false,3); set_controls_allowed(false); set_timer(timer_cnt+100000U);
    p=command(bus,3); CHECK(!safety_tx_hook(&p)); // stale sensors
    p=command(bus,0); CHECK(safety_tx_hook(&p)); // release always possible
    reset(f|1024U); feed(bus,0,true,false,3); set_controls_allowed(false);
    p=command(bus,3); CHECK(safety_tx_hook(&p));
    set_timer(4200000U); feed(bus,0,true,false,3); set_controls_allowed(false);
    p=command(bus,3); CHECK(!safety_tx_hook(&p)); // hard duration cutoff
    p=command(bus,0); CHECK(safety_tx_hook(&p)); // idle > 1 sec resets test
    p=command(bus,3); CHECK(safety_tx_hook(&p));
    reset(f|1024U); set_timer(0xfffffff0U); feed(bus,0,true,false,3); set_controls_allowed(false);
    set_timer(0x100U); p=command(bus,3); CHECK(safety_tx_hook(&p)); // timer wrap
    set_relay_malfunction(true); p=command(bus,0); CHECK(!safety_tx_hook(&p));
  }
  printf("PASS %u native safety checks\n",tests); return 0;
}
