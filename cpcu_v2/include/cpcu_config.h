/**
 *  @file       cpcu_config.h
 *  @brief      Runtime config loader — JSON to IPC_RuntimeConfig (v2.3.3).
 *  @author     bugrASl
 *  @date       April 2026
 *  @version    1.0
 *
 *  Parses cpcu_v2/config/runtime.json (or whatever path is passed) into
 *  an IPC_RuntimeConfig struct. cpcu_kernel calls this on startup and
 *  on SIGHUP, then publishes the result to shared memory via
 *  IPC_WriteRuntimeConfig.
 *
 *  Design rules:
 *      - Refuse to start on schema mismatch. No silent defaults.
 *      - Validate every numeric field against its sane range.
 *      - On any parse failure, return non-zero and DO NOT touch the
 *        output struct beyond zeroing it.
 *      - No third-party JSON dependency — small hand-rolled parser
 *        sufficient for the flat, well-known schema.
 *
 *  See cpcu_v2/docs/RUNTIME_CONFIG.md.
 */

#ifndef CPCU_CONFIG_H
#define CPCU_CONFIG_H

#include <stddef.h>

#include "cpcu_ipc.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CPCU_CONFIG_SCHEMA_VERSION  1

/*  Return codes for CFG_LoadFromFile. */
typedef enum
{
    CFG_OK = 0,
    CFG_ERR_OPEN,           /* fopen failed, file missing/unreadable */
    CFG_ERR_PARSE,          /* JSON syntax error */
    CFG_ERR_SCHEMA,         /* schema_version mismatch */
    CFG_ERR_RANGE,          /* a value was out of its sane range */
    CFG_ERR_MISSING,        /* a required field was absent */
    CFG_ERR_INTERNAL        /* something went wrong inside the loader */
} CFG_Status;

/*  Parse JSON file into runtime config struct.
 *      path        absolute path to runtime.json
 *      out         destination — zeroed before use
 *      err_msg     buffer for human-readable error message, may be NULL
 *      err_msg_sz  size of err_msg buffer
 *  Returns CFG_OK on success. On any error the caller should LOG_E and
 *  refuse to start. */
CFG_Status  CFG_LoadFromFile(const char *path, IPC_RuntimeConfig *out,
                             char *err_msg, size_t err_msg_sz);

/*  Populate out with compile-time defaults (factory configuration).
 *  Used by tests and by the configure.sh "reset" path; cpcu_kernel
 *  itself never falls back to this on production startup. */
void        CFG_Defaults(IPC_RuntimeConfig *out);

/*  Convert a CFG_Status to a static string (for logging). */
const char *CFG_StatusStr(CFG_Status s);

#ifdef __cplusplus
}
#endif

#endif  /* CPCU_CONFIG_H */
