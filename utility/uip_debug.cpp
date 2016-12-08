#include <Arduino.h>
#include <HardwareSerial.h>
#include <inttypes.h>
#include <utility/uip_debug.h>
#include <utility/logging.h>
extern "C" {
  #include "utility/uip.h"
}

struct uip_conn con[UIP_CONNS];

void
UIPDebug::uip_debug_printconns()
{
  for(uint8_t i=0;i<UIP_CONNS;i++)
    {
      if (uip_debug_printcon(&con[i],&uip_conns[i]))
        {
          LogObject.print(F("connection["));
          LogObject.print(i);
          LogObject.println(F("] changed."));
        }
    }
}

bool
UIPDebug::uip_debug_printcon(struct uip_conn *lhs,struct uip_conn *rhs)
{
  bool changed = false;
  if (!uip_ipaddr_cmp(lhs->ripaddr,rhs->ripaddr))
    {
      LogObject.print(F(" ripaddr: "));
      uip_debug_printbytes((const uint8_t *)lhs->ripaddr,4);
      LogObject.print(F(" -> "));
      uip_debug_printbytes((const uint8_t *)rhs->ripaddr,4);
      LogObject.println();
      uip_ipaddr_copy(lhs->ripaddr,rhs->ripaddr);
      changed = true;
    }
  if (lhs->lport != rhs->lport)
    {
      LogObject.print(F(" lport: "));
      LogObject.print(htons(lhs->lport));
      LogObject.print(F(" -> "));
      LogObject.println(htons(rhs->lport));
      lhs->lport = rhs->lport;
      changed = true;
    }
  if (lhs->rport != rhs->rport)
    {
      LogObject.print(F(" rport: "));
      LogObject.print(htons(lhs->rport));
      LogObject.print(F(" -> "));
      LogObject.println(htons(rhs->rport));
      lhs->rport = rhs->rport;
      changed = true;
    }
  if ((uint32_t)lhs->rcv_nxt[0] != (uint32_t)rhs->rcv_nxt[0])
    {
      LogObject.print(F(" rcv_nxt: "));
      uip_debug_printbytes(lhs->rcv_nxt,4);
      LogObject.print(F(" -> "));
      uip_debug_printbytes(rhs->rcv_nxt,4);
      *((uint32_t *)&lhs->rcv_nxt[0]) = (uint32_t)rhs->rcv_nxt[0];
      LogObject.println();
      changed = true;
    }
  if ((uint32_t)lhs->snd_nxt[0] != (uint32_t)rhs->snd_nxt[0])
    {
      LogObject.print(F(" snd_nxt: "));
      uip_debug_printbytes(lhs->snd_nxt,4);
      LogObject.print(F(" -> "));
      uip_debug_printbytes(rhs->snd_nxt,4);
      *((uint32_t *)&lhs->snd_nxt[0]) = (uint32_t)rhs->snd_nxt[0];
      LogObject.println();
      changed = true;
    }
  if (lhs->len != rhs->len)
    {
      LogObject.print(F(" len: "));
      LogObject.print(lhs->len);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->len);
      lhs->len = rhs->len;
      changed = true;
    }
  if (lhs->mss != rhs->mss)
    {
      LogObject.print(F(" mss: "));
      LogObject.print(lhs->mss);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->mss);
      lhs->mss = rhs->mss;
      changed = true;
    }
  if (lhs->initialmss != rhs->initialmss)
    {
      LogObject.print(F(" initialmss: "));
      LogObject.print(lhs->initialmss);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->initialmss);
      lhs->initialmss = rhs->initialmss;
      changed = true;
    }
  if (lhs->sa != rhs->sa)
    {
      LogObject.print(F(" sa: "));
      LogObject.print(lhs->sa);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->sa);
      lhs->sa = rhs->sa;
      changed = true;
    }
  if (lhs->sv != rhs->sv)
    {
      LogObject.print(F(" sv: "));
      LogObject.print(lhs->sv);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->sv);
      lhs->sv = rhs->sv;
      changed = true;
    }
  if (lhs->rto != rhs->rto)
    {
      LogObject.print(F(" rto: "));
      LogObject.print(lhs->rto);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->rto);
      lhs->rto = rhs->rto;
      changed = true;
    }
  if (lhs->tcpstateflags != rhs->tcpstateflags)
    {
      LogObject.print(F(" tcpstateflags: "));
      LogObject.print(lhs->tcpstateflags);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->tcpstateflags);
      lhs->tcpstateflags = rhs->tcpstateflags;
      changed = true;
    }
  if (lhs->timer != rhs->timer)
    {
      LogObject.print(F(" timer: "));
      LogObject.print(lhs->timer);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->timer);
      lhs->timer = rhs->timer;
      changed = true;
    }
  if (lhs->nrtx != rhs->nrtx)
    {
      LogObject.print(F(" nrtx: "));
      LogObject.print(lhs->nrtx);
      LogObject.print(F(" -> "));
      LogObject.println(rhs->nrtx);
      lhs->nrtx = rhs->nrtx;
      changed = true;
    }
  return changed;
}

void
UIPDebug::uip_debug_printbytes(const uint8_t *data, uint8_t len)
{
  for(uint8_t i=0;i<len;i++)
    {
      LogObject.print(data[i]);
      if (i<len-1)
        LogObject.print(F(","));
    }
}
