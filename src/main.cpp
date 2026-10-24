#include "node_common.h"

#if defined(STEP1)
void step1_main();
#elif defined(STEP2)
void step2_main();
#elif defined(STEP3)
void step3_main();
#elif defined(STEP4)
void step4_main();
#endif

extern "C" void app_main(void) {
#if defined(STEP1)
    step1_main();   // minimal LED + UART bring-up
#elif defined(STEP2)
    step2_main();   // LED + leader election + telemetry
#elif defined(STEP3)
    step3_main();   // + Wi-Fi SoftAP + web dashboard (no CSI)
#elif defined(STEP4)
    step4_main();   // full pipeline: + CSI capture + DSP + fusion
#else
    app_run();
#endif
}
