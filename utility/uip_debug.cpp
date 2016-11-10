#include <Arduino.h>
#include <HardwareSerial.h>
#include <inttypes.h>
#include <utility/uip_debug.h>
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
          Serial.print(F("connection["));
          Serial.print(i);
          Serial.println(F("] changed."));
        }
    }
}

bool
UIPDebug::uip_debug_printcon(struct uip_conn *lhs,struct uip_conn *rhs)
{
  bool changed = false;
  if (!uip_ipaddr_cmp(lhs->ripaddr,rhs->ripaddr))
    {
      Serial.print(F(" ripaddr: "));
      uip_debug_printbytes((const uint8_t *)lhs->ripaddr,4);
      Serial.print(F(" -> "));
      uip_debug_printbytes((const uint8_t *)rhs->ripaddr,4);
      Serial.println();
      uip_ipaddr_copy(lhs->ripaddr,rhs->ripaddr);
      changed = true;
    }
  if (lhs->lport != rhs->lport)
    {
      Serial.print(F(" lport: "));
      Serial.print(htons(lhs->lport));
      Serial.print(F(" -> "));
      Serial.println(htons(rhs->lport));
      lhs->lport = rhs->lport;
      changed = true;
    }
  if (lhs->rport != rhs->rport)
    {
      Serial.print(F(" rport: "));
      Serial.print(htons(lhs->rport));
      Serial.print(F(" -> "));
      Serial.println(htons(rhs->rport));
      lhs->rport = rhs->rport;
      changed = true;
    }
  if ((uint32_t)lhs->rcv_nxt[0] != (uint32_t)rhs->rcv_nxt[0])
    {
      Serial.print(F(" rcv_nxt: "));
      uip_debug_printbytes(lhs->rcv_nxt,4);
      Serial.print(F(" -> "));
      uip_debug_printbytes(rhs->rcv_nxt,4);
      *((uint32_t *)&lhs->rcv_nxt[0]) = (uint32_t)rhs->rcv_nxt[0];
      Serial.println();
      changed = true;
    }
  if ((uint32_t)lhs->snd_nxt[0] != (uint32_t)rhs->snd_nxt[0])
    {
      Serial.print(F(" snd_nxt: "));
      uip_debug_printbytes(lhs->snd_nxt,4);
      Serial.print(F(" -> "));
      uip_debug_printbytes(rhs->snd_nxt,4);
      *((uint32_t *)&lhs->snd_nxt[0]) = (uint32_t)rhs->snd_nxt[0];
      Serial.println();
      changed = true;
    }
  if (lhs->len != rhs->len)
    {
      Serial.print(F(" len: "));
      Serial.print(lhs->len);
      Serial.print(F(" -> "));
      Serial.println(rhs->len);
      lhs->len = rhs->len;
      changed = true;
    }
  if (lhs->mss != rhs->mss)
    {
      Serial.print(F(" mss: "));
      Serial.print(lhs->mss);
      Serial.print(F(" -> "));
      Serial.println(rhs->mss);
      lhs->mss = rhs->mss;
      changed = true;
    }
  if (lhs->initialmss != rhs->initialmss)
    {
      Serial.print(F(" initialmss: "));
      Serial.print(lhs->initialmss);
      Serial.print(F(" -> "));
      Serial.println(rhs->initialmss);
      lhs->initialmss = rhs->initialmss;
      changed = true;
    }
  if (lhs->sa != rhs->sa)
    {
      Serial.print(F(" sa: "));
      Serial.print(lhs->sa);
      Serial.print(F(" -> "));
      Serial.println(rhs->sa);
      lhs->sa = rhs->sa;
      changed = true;
    }
  if (lhs->sv != rhs->sv)
    {
      Serial.print(F(" sv: "));
      Serial.print(lhs->sv);
      Serial.print(F(" -> "));
      Serial.println(rhs->sv);
      lhs->sv = rhs->sv;
      changed = true;
    }
  if (lhs->rto != rhs->rto)
    {
      Serial.print(F(" rto: "));
      Serial.print(lhs->rto);
      Serial.print(F(" -> "));
      Serial.println(rhs->rto);
      lhs->rto = rhs->rto;
      changed = true;
    }
  if (lhs->tcpstateflags != rhs->tcpstateflags)
    {
      Serial.print(F(" tcpstateflags: "));
      Serial.print(lhs->tcpstateflags);
      Serial.print(F(" -> "));
      Serial.println(rhs->tcpstateflags);
      lhs->tcpstateflags = rhs->tcpstateflags;
      changed = true;
    }
  if (lhs->timer != rhs->timer)
    {
      Serial.print(F(" timer: "));
      Serial.print(lhs->timer);
      Serial.print(F(" -> "));
      Serial.println(rhs->timer);
      lhs->timer = rhs->timer;
      changed = true;
    }
  if (lhs->nrtx != rhs->nrtx)
    {
      Serial.print(F(" nrtx: "));
      Serial.print(lhs->nrtx);
      Serial.print(F(" -> "));
      Serial.println(rhs->nrtx);
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
      Serial.print(data[i]);
      if (i<len-1)
        Serial.print(F(","));
    }
}
