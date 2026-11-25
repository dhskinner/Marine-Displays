#pragma once
#include "Arduino.h"
#include <cstring>
#include "FS.h"
#include "SD.h"
#include "vfs_api.h"
#include "SPI.h"

#include "TCA9554PWR.h"

// Waveshare ESP32-S3-Touch-LCD-7: the TF slot is wired for SPI, with chip
// select on the CH422G expander (EXIO4), not a GPIO. CS is held low (card
// selected) while mounted; the card is the only device on this SPI bus.
#define SD_MOSI_PIN  11
#define SD_SCK_PIN   12
#define SD_MISO_PIN  13
#define SD_SPI_HZ    20000000

// The firmware uses the SD_MMC (SDMMC host) API throughout. This class
// gives the same calls on top of the SPI SD driver, mounted at the same
// "/sdcard" path so the POSIX fopen()/stat() code works unchanged.
class SdSpiCard : public fs::SDFS {
public:
  SdSpiCard() : fs::SDFS(fs::FSImplPtr(new VFSImpl())) {}
  // Same signature as SDMMCFS::begin; mode1bit/format/freq are ignored.
  bool begin(const char *mountpoint = "/sdcard", bool mode1bit = true,
             bool format_if_mount_failed = false, int sdmmc_frequency = 0,
             uint8_t maxOpenFiles = 5);
  void setPins(int, int, int) {}
};
extern SdSpiCard SD_SPI_CARD;
#define SD_MMC SD_SPI_CARD

extern uint16_t SDCard_Size;
extern uint16_t Flash_Size;

esp_err_t SD_Init();

// Check SD health and attempt a conservative recovery (end()/begin()) if reads fail.
void SD_RecoveryCheck(void);
void Flash_test();

// SD chip select (EXIO4, active low): EN selects the card, Dis releases it.
void SD_D3_Dis();
void SD_D3_EN();

// Runtime SD control helpers
bool SD_IsMounted();

bool File_Search(const char* directory, const char* fileName);
uint16_t Folder_retrieval(const char* directory, const char* fileExtension, char File_Name[][100],uint16_t maxFiles);
