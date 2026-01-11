idf_component_register(
    SRCS 
        "main.cpp"
        "config_defs.c"
        "dos/cpu8086.c"
        "dos/memory.c"
        "dos/disk.c"
        "dos/fat16_image.c"
        "dos/hdd_image.c"
        "dos/bios.c"
        "dos/video.c"
        "dos/ports.c"
        "dos/interrupts.c"
        "dos/xms.c"
        "dos/speaker.cpp"
        "net/wifi_sta.c"
        "net/wifi_ap.c"
        "net/disk_swap_server.c"
        "settings.c"
        "display/epd_driver.c"
        "display/epd_update.c"
        "display/font8x16.c"
        "sdcard/sdcard.c"
        "emulator/embedded_8086tiny_bios.c"
        "bt/esp_hid_gap.c"
        "bt/bt_keyboard.c"
    INCLUDE_DIRS 
        "."
        "dos"
        "display"
        "sdcard"
        "emulator"
        "bt"
        "net"
    EMBED_TXTFILES
        "net/webui/index.html"
    REQUIRES 
        driver
        fatfs
        esp_http_server
        esp_netif
        esp_wifi
        esp_timer
        spi_flash
        sdmmc
        m5unified
        nvs_flash
        esp_hid
        bt
        esp_event
)

# Generate `wifi_secrets.h` from `${PROJECT_ROOT}/secrets.txt` (kept out of git).
set(SECRETS_TXT "${CMAKE_SOURCE_DIR}/secrets.txt")
set(GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${GEN_DIR}")

set(WIFI_SSID_RAW "")
set(WIFI_PASS_RAW "")
if(EXISTS "${SECRETS_TXT}")
    # Re-run CMake configure when secrets change (otherwise values can stay stale).
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${SECRETS_TXT}")
    file(READ "${SECRETS_TXT}" SECRETS_CONTENT)
    # Accept either WIFI_SSID/WIFI_PASS or SSID/PASSWORD for convenience.
    string(REGEX MATCH "(^|[\r\n])[ \t]*WIFI_SSID[ \t]*=[ \t]*([^\r\n]+)" _ssid_match "${SECRETS_CONTENT}")
    set(WIFI_SSID_RAW "${CMAKE_MATCH_2}")
    if(WIFI_SSID_RAW STREQUAL "")
        string(REGEX MATCH "(^|[\r\n])[ \t]*SSID[ \t]*=[ \t]*([^\r\n]+)" _ssid2_match "${SECRETS_CONTENT}")
        set(WIFI_SSID_RAW "${CMAKE_MATCH_2}")
    endif()

    string(REGEX MATCH "(^|[\r\n])[ \t]*WIFI_PASS[ \t]*=[ \t]*([^\r\n]+)" _pass_match "${SECRETS_CONTENT}")
    set(WIFI_PASS_RAW "${CMAKE_MATCH_2}")
    if(WIFI_PASS_RAW STREQUAL "")
        string(REGEX MATCH "(^|[\r\n])[ \t]*PASSWORD[ \t]*=[ \t]*([^\r\n]+)" _pass2_match "${SECRETS_CONTENT}")
        set(WIFI_PASS_RAW "${CMAKE_MATCH_2}")
    endif()

    string(STRIP "${WIFI_SSID_RAW}" WIFI_SSID_RAW)
    string(STRIP "${WIFI_PASS_RAW}" WIFI_PASS_RAW)

    if(WIFI_SSID_RAW MATCHES "^\"(.*)\"$")
        set(WIFI_SSID_RAW "${CMAKE_MATCH_1}")
    endif()
    if(WIFI_PASS_RAW MATCHES "^\"(.*)\"$")
        set(WIFI_PASS_RAW "${CMAKE_MATCH_1}")
    endif()
endif()

set(WIFI_SSID_ESC "${WIFI_SSID_RAW}")
set(WIFI_PASS_ESC "${WIFI_PASS_RAW}")
string(REPLACE "\\" "\\\\" WIFI_SSID_ESC "${WIFI_SSID_ESC}")
string(REPLACE "\"" "\\\"" WIFI_SSID_ESC "${WIFI_SSID_ESC}")
string(REPLACE "\\" "\\\\" WIFI_PASS_ESC "${WIFI_PASS_ESC}")
string(REPLACE "\"" "\\\"" WIFI_PASS_ESC "${WIFI_PASS_ESC}")

configure_file("${CMAKE_CURRENT_LIST_DIR}/wifi_secrets.h.in" "${GEN_DIR}/wifi_secrets.h" @ONLY)
target_include_directories(${COMPONENT_LIB} PRIVATE "${GEN_DIR}")
