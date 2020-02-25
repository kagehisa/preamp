/*  NVS access routines

   This code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "msg_stuff.h"
#include "nv_acc.h"

/* Initialises the nvs 
 * returns 0 in case of error, 1 for success
 */
esp_err_t init_nv(void)
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
 
    return err;
}
    

/* Opens the nvs section for a certain handle 
 * returns 0 in case of error, 1 for success
 */
esp_err_t open_nv(nvs_handle *my_handle)
{
    // Open

    esp_err_t err;
    MSG("Opening Non-Volatile Storage (NVS) handle...\n ");
 
    err = nvs_open("storage", NVS_READWRITE, my_handle);
    if (err != ESP_OK) {
        MSG("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	return 0; //error
    } else {
        MSG("Done\n");
	return 1; //success
    }
}




/* Read a uint8_t value from the nvs
 * requires a valid handle, uint8_t pointer, key name for the nvs section 
 * returns ESP_ERR in case of error, ESP_OK for success
 */
esp_err_t read_u8_nv(nvs_handle my_handle, uint8_t *val, const char *val_name)
{
   esp_err_t err;
   // Read
   MSG("Reading %s from NVS ...\n ", val_name);
    
   err = nvs_get_u8(my_handle, val_name, val);
        switch (err) {
            case ESP_OK:
                MSG("Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                MSG("The value is not initialized yet!\n");
                break;
            default :
                MSG("Error (%s) reading!\n", esp_err_to_name(err));
        }
   return err; //err
}

/* Write a uint8_t to the nvs
 * requires a valid handle, uint8_t pointer, key name for the nvs section 
 * returns ESP_ERR in case of error, ESP_OK for success
 */
esp_err_t write_u8_nv(nvs_handle my_handle, uint8_t val, const char *val_name)
{  
   esp_err_t err;
   // Write
   MSG("Updating %s in NVS ...\n ", val_name);
    
   err = nvs_set_u8(my_handle, val_name, val);
   
   // Commit written value.
   // After setting any values, nvs_commit() must be called to ensure changes are written
   // to flash storage. Implementations may write to storage at other times,
   // but this is not guaranteed.
   MSG("Committing updates in NVS ...\n ");
   err = nvs_commit(my_handle);
   return err;
}


/* Read a uint8_t array from the nvs
 * requires a valid handle, uint8_t array pointer, array size, key name for the nvs section 
 * returns ESP_ERR in case of error, ESP_OK for success
 */
esp_err_t read_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name)
{
   esp_err_t err;
   // Read
   MSG("Reading %s from NVS ...\n ", blob_name);
    
   err = nvs_get_blob(my_handle, blob_name, blob, (size_t *)&blob_size);
        switch (err) {
            case ESP_OK:
                MSG("Done\n");
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                MSG("The value is not initialized yet!\n");
                break;
            default :
                MSG("Error (%s) reading!\n", esp_err_to_name(err));
        }
   return err; //err
}


/* Write a uint8_t array to the nvs
 * requires a valid handle, uint8_t array pointer, array size, key name for the nvs section 
 * returns ESP_ERR in case of error, ESP_OK for success
 */
esp_err_t write_blob_nv(nvs_handle my_handle, uint8_t *blob, uint8_t blob_size, const char *blob_name)
{  
   esp_err_t err;
   // Write
   MSG("Updating %s in NVS ...\n ", blob_name);
    
   err = nvs_set_blob(my_handle, blob_name, blob, (size_t) blob_size);
   
   // Commit written value.
   // After setting any values, nvs_commit() must be called to ensure changes are written
   // to flash storage. Implementations may write to storage at other times,
   // but this is not guaranteed.
   MSG("Committing updates in NVS ...\n ");
   err = nvs_commit(my_handle);
   return err;
}


/* Closes the handle for the nvs page
 * requires a valid handle 
 * returns 1 for success
 */
esp_err_t close_nv(nvs_handle my_handle)
{
   
   // Close
   nvs_close(my_handle);
   return ESP_OK;
}

