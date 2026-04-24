/**
 *  @file       cpcu_log.c
 *  @brief      Global log state definitions
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    2.1
 */

#include "cpcu_log.h"
#include <stdbool.h>    /* defensive — already pulled in by cpcu_log.h v2.1,
                           but explicit here so a stale v2.0 header still
                           compiles past the bool declaration below.      */

/*============= CORE STATE =========================================================*/

LogLevel        g_log_level =   LOG_INFO;
const char     *g_log_proc  =   "????";
int             g_log_color =   0;
struct timespec g_log_boot  =   {0, 0};

/*============= FILE-SINK STATE ====================================================*/

bool            g_log_to_file       =   false;
char            g_log_dir[128]      =   LOG_DIR_DEFAULT;
LogFileSink     g_log_sinks[LOG_MAX_MODULES] = {{{0}, NULL}};
int             g_log_sink_count    =   0;
