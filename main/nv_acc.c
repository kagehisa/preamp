#include "nv_acc.h"

uint8_t init_nv(void)
{
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );
 
    return ((err != ESP_OK) ? 0 : 1 );
}
    

uint8_t open_nv(nvs_handle *my_handle)
{
    // Open

    uint8_t err;
    printf("Opening Non-Volatile Storage (NVS) handle...\n ");
 
    err = nvs_open("storage", NVS_READWRITE, my_handle);
    if (err != ESP_OK) {
        printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	return 0; //error
    } else {
        printf("Done\n");
	return 1; //success
    }
}
        

uint8_t read_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name)
{
   uint8_t err;
   // Read
   printf("Reading %s from NVS ...\n ", blob_name);
    
   err = nvs_get_blob(my_handle, blob_name, blob, (size_t *)&blob_size);
        switch (err) {
            case ESP_OK:
                printf("Done\n");
		return 1;
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
	return 0; //err
}


uint8_t write_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name)
{  
   uint8_t err;
   // Write
   printf("Updating %s in NVS ...\n ", blob_name);
    
   err = nvs_set_blob(my_handle, blob_name, blob, (size_t) blob_size);
   
   // Commit written value.
   // After setting any values, nvs_commit() must be called to ensure changes are written
   // to flash storage. Implementations may write to storage at other times,
   // but this is not guaranteed.
   printf("Committing updates in NVS ...\n ");
   err = nvs_commit(my_handle);
   return ((err != ESP_OK) ? 0 : 1 );
}


uint8_t close_nv(nvs_handle my_handle)
{
   
   // Close
   nvs_close(my_handle);
   return 1;
}

