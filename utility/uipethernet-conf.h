#ifndef UIPETHERNET_CONF_H
#define UIPETHERNET_CONF_H

/* for TCP */
#ifndef UIP_SOCKET_NUMPACKETS
#define UIP_SOCKET_NUMPACKETS    5
#endif
#ifndef UIP_CONF_MAX_CONNECTIONS
#define UIP_CONF_MAX_CONNECTIONS 4
#endif

/* for UDP
 * set UIP_CONF_UDP to 0 to disable UDP (saves aprox. 5kb flash) */
#ifndef UIP_CONF_UDP
#define UIP_CONF_UDP             1
#endif
#ifndef UIP_CONF_BROADCAST
#define UIP_CONF_BROADCAST       1
#endif
#ifndef UIP_CONF_UDP_CONNS
#define UIP_CONF_UDP_CONNS       4
#endif

/* timeout in ms for attempts to get a free memory block to write
 * before returning number of bytes sent so far
 * set to 0 to block until connection is closed by timeout */
#ifndef UIP_WRITE_TIMEOUT
#define UIP_WRITE_TIMEOUT        0
#endif

/* timeout after which UIPClient::connect gives up. The timeout is specified in seconds.
 * if set to a number <= 0 connect will timeout when uIP does (which might be longer than you expect...) */
#ifndef UIP_CONNECT_TIMEOUT
#define UIP_CONNECT_TIMEOUT      -1
#endif

/* periodic timer for uip (in ms) */
#ifndef UIP_PERIODIC_TIMER
#define UIP_PERIODIC_TIMER       250
#endif

/* timer to poll client for data after last write (in ms)
 * set to -1 to disable fast polling and rely on periodic only (saves 100 bytes flash) */
#ifndef UIP_CLIENT_TIMER
#define UIP_CLIENT_TIMER         10
#endif

#endif
