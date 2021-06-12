// void sdInit();
// void sdMount();
// void sdDone();
#define SD_CARD_PRESENT()              true // SD card always present on raspberry pi.
#define UNEXPECTED_SHUTDOWN()          false // Tinycore OS is readonly, so shutdown is never unexpected.