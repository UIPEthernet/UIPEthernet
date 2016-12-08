#ifndef __LOGGING_H__
#define __LOGGING_H__

#define	LOG_NONE		-1	/* Logging nothing */
#define	LOG_EMERG		0	/* system is unusable */
#define	LOG_ALERT		1	/* action must be taken immediately */
#define	LOG_CRIT		2	/* critical conditions */
#define	LOG_ERR			3	/* error conditions */
#define	LOG_WARNING		4	/* warning conditions */
#define	LOG_NOTICE		5	/* normal but significant condition */
#define	LOG_INFO		6	/* informational */
#define	LOG_DEBUG		7	/* debug-level messages */
#define	LOG_DEBUG_V1		8	/* debug-verbose-level (v) messages */
#define	LOG_DEBUG_V2		9	/* debug-verbose-level (vv) messages */
#define	LOG_DEBUG_V3		10	/* debug-verbose-level (vvv) messages */

#pragma message "You can configure LogObject and ACTLOGLEVEL in 'utility/logging.h'. More verbosity more memory usage."
//#define ACTLOGLEVEL     LOG_NONE
#define ACTLOGLEVEL LOG_INFO

#if ACTLOGLEVEL>LOG_NONE 
   #include "HardwareSerial.h"
   #if defined(__STM32F1__) || defined(__STM32F3__) || defined(__STM32F4__)
      #define LogObject Serial1
   #else
      #define LogObject Serial
   #endif
#endif

#endif
