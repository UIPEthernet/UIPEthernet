#if defined(ARDUINO)
  #include <Arduino.h>
#endif
#if defined(__MBED__)
  #include <mbed.h>
#endif
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
	#if ACTLOGLEVEL>LOG_NONE
          LogObject.uart_send_str(F("connection["));
          LogObject.uart_send_dec(i);
          LogObject.uart_send_strln(F("] changed."));
	#endif
        }
    }
}

bool
UIPDebug::uip_debug_printcon(struct uip_conn *lhs,struct uip_conn *rhs)
{
  bool changed = false;
  if (!uip_ipaddr_cmp(lhs->ripaddr,rhs->ripaddr))
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" ripaddr: "));
      uip_debug_printbytes((const uint8_t *)lhs->ripaddr,4);
      LogObject.uart_send_str(F(" -> "));
      uip_debug_printbytes((const uint8_t *)rhs->ripaddr,4);
      LogObject.uart_send_strln(F(""));
    #endif
    uip_ipaddr_copy(lhs->ripaddr,rhs->ripaddr);
    changed = true;
    }
  if (lhs->lport != rhs->lport)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" lport: "));
      LogObject.uart_send_dec(htons(lhs->lport));
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(htons(rhs->lport));
    #endif
    lhs->lport = rhs->lport;
    changed = true;
    }
  if (lhs->rport != rhs->rport)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" rport: "));
      LogObject.uart_send_dec(htons(lhs->rport));
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(htons(rhs->rport));
    #endif
    lhs->rport = rhs->rport;
    changed = true;
    }
  if ((uint32_t)lhs->rcv_nxt[0] != (uint32_t)rhs->rcv_nxt[0])
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" rcv_nxt: "));
      uip_debug_printbytes(lhs->rcv_nxt,4);
      LogObject.uart_send_str(F(" -> "));
      uip_debug_printbytes(rhs->rcv_nxt,4);
    #endif
    *((uint32_t *)&lhs->rcv_nxt[0]) = (uint32_t)rhs->rcv_nxt[0];
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_strln(F(""));
    #endif
    changed = true;
    }
  if ((uint32_t)lhs->snd_nxt[0] != (uint32_t)rhs->snd_nxt[0])
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" snd_nxt: "));
      uip_debug_printbytes(lhs->snd_nxt,4);
      LogObject.uart_send_str(F(" -> "));
      uip_debug_printbytes(rhs->snd_nxt,4);
    #endif
    *((uint32_t *)&lhs->snd_nxt[0]) = (uint32_t)rhs->snd_nxt[0];
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_strln(F(""));
    #endif
    changed = true;
    }
  if (lhs->len != rhs->len)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" len: "));
      LogObject.uart_send_dec(lhs->len);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->len);
    #endif
    lhs->len = rhs->len;
    changed = true;
    }
  if (lhs->mss != rhs->mss)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" mss: "));
      LogObject.uart_send_dec(lhs->mss);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->mss);
    #endif
    lhs->mss = rhs->mss;
    changed = true;
    }
  if (lhs->initialmss != rhs->initialmss)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" initialmss: "));
      LogObject.uart_send_dec(lhs->initialmss);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->initialmss);
    #endif
    lhs->initialmss = rhs->initialmss;
    changed = true;
    }
  if (lhs->sa != rhs->sa)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" sa: "));
      LogObject.uart_send_dec(lhs->sa);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->sa);
    #endif
    lhs->sa = rhs->sa;
    changed = true;
    }
  if (lhs->sv != rhs->sv)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" sv: "));
      LogObject.uart_send_dec(lhs->sv);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->sv);
    #endif
    lhs->sv = rhs->sv;
    changed = true;
    }
  if (lhs->rto != rhs->rto)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" rto: "));
      LogObject.uart_send_dec(lhs->rto);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->rto);
    #endif
    lhs->rto = rhs->rto;
    changed = true;
    }
  if (lhs->tcpstateflags != rhs->tcpstateflags)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" tcpstateflags: "));
      LogObject.uart_send_dec(lhs->tcpstateflags);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->tcpstateflags);
    #endif
    lhs->tcpstateflags = rhs->tcpstateflags;
    changed = true;
    }
  if (lhs->timer != rhs->timer)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" timer: "));
      LogObject.uart_send_dec(lhs->timer);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->timer);
    #endif
    lhs->timer = rhs->timer;
    changed = true;
    }
  if (lhs->nrtx != rhs->nrtx)
    {
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_str(F(" nrtx: "));
      LogObject.uart_send_dec(lhs->nrtx);
      LogObject.uart_send_str(F(" -> "));
      LogObject.uart_send_decln(rhs->nrtx);
    #endif
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
    #if ACTLOGLEVEL>LOG_NONE
      LogObject.uart_send_dec(data[i]);
      if (i<len-1)
        LogObject.uart_send_str(F(","));
    #endif
    }
}
