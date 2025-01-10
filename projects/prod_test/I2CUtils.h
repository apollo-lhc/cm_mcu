// includes for types
#include <stdbool.h>

// local includes
#include "common/smbus.h"

/**
 * @brief helper function to verify I2C/SMBUS transactions sucecssful
 *
 * @param [in] r  tSMBUSStatus returned by SMBUS call to check
 * @param [in] timeout  time until timeout in units of 10ms, <0 disables check
 * @param [in] no_message  if true, disables printing message
 * @param [in] g_sMaster  tSMBus associated with transactoin
 * @param [in] eStatus  tSMBusStatus for target tSMBus
 * @param [out] m  output string
 * @return 0 if no errors encountered, otherwise, size of m buffer used by
 *         error message (no_message false) or 1 (no_message true)
 */
int check_i2c_transaction(tSMBusStatus r, int timeout, bool no_message,
                          tSMBus *g_sMaster, tSMBusStatus eStatus, char *m);
