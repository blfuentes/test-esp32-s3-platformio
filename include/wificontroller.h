#ifndef __WIFIDEFINITION_H__
#define __WIFIDEFINITION_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <string.h>

#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

class WifiDefinition {
public:
    static EventGroupHandle_t wifi_event_group;
    static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data);
    WifiDefinition();
    WifiDefinition(const char* ssid, const char* password);
    void setSSID(const char* ssid);
    void setPassword(const char* password);
    const char* getSSID();
    const char* getPassword();
    void connect(bool *status);
    void setStatus(bool status);
    bool getStatus();
private:
    const char* ssid;
    const char* password;
    bool status;
    void wifi_init_sta();
};

#endif // __WIFIDEFINITION_H__