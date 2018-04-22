#include <stdio.h>
#include "nvs_flash.h"
#include "nvs.h"


uint8_t init_nv(void);

uint8_t open_nv(nvs_handle *my_handle);

uint8_t read_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name);

uint8_t write_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name);

uint8_t close_nv(nvs_handle my_handle);

