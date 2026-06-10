/* ---------------------------------------------------------------------------
 * ODYX :: RTKLIB receiver-decoder stubs.
 *
 * We compile rtklib/src/rcvraw.c only for decode_frame() (GPS LNAV ephemeris
 * decoding). rcvraw.c's input_raw()/init_raw()/free_raw() dispatch to the
 * per-receiver decoders in rtklib/src/rcv/*.c, which ODYX does not use (Android
 * GnssMeasurement is already decoded by the framework). Rather than compile all
 * those receiver formats (and their RTCM dependencies), we provide stub
 * definitions so the static archive links. These are never called by ODYX.
 * --------------------------------------------------------------------------- */
#include "rtklib.h"

#define IN_STUB(name)  int name(raw_t *raw, uint8_t data){ (void)raw;(void)data; return -1; }
#define INF_STUB(name) int name(raw_t *raw, FILE *fp){ (void)raw;(void)fp; return -2; }

IN_STUB(input_oem4)  IN_STUB(input_oem3)  IN_STUB(input_ubx)   IN_STUB(input_ss2)
IN_STUB(input_cres)  IN_STUB(input_stq)   IN_STUB(input_javad) IN_STUB(input_nvs)
IN_STUB(input_bnx)   IN_STUB(input_rt17)  IN_STUB(input_sbf)

INF_STUB(input_oem4f)  INF_STUB(input_oem3f)  INF_STUB(input_ubxf)   INF_STUB(input_ss2f)
INF_STUB(input_cresf)  INF_STUB(input_stqf)   INF_STUB(input_javadf) INF_STUB(input_nvsf)
INF_STUB(input_bnxf)   INF_STUB(input_rt17f)  INF_STUB(input_sbff)

int  init_rt17(raw_t *raw){ (void)raw; return 0; }
int  init_cmr (raw_t *raw){ (void)raw; return 0; }
void free_rt17(raw_t *raw){ (void)raw; }
void free_cmr (raw_t *raw){ (void)raw; }
int  update_cmr(raw_t *raw, rtksvr_t *svr, obs_t *obs){ (void)raw;(void)svr;(void)obs; return 0; }
