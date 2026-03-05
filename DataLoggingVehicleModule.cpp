#include "SPI.h"
#include "CAN.h"
#include <cstring>
#include <thread>
#include <cstdio>

// 1. Configure the SPI controller
// --- 1.1 SPI clock 1MHz, CS1
// 2. Configure the CAN controller
// --- 2.1 Baud 100K, 11 bit format
// 3. Add the CAN RX ISR
// 4. Every 500ms send the CAN RTR frames to the CAN Bus
// --- 4.5 CAN_AVG_TEMPERATURE_11_SENSOR_ID
// --- 4.6 CAN_CURRENT_TEMP_11_SENSOR_ID
// --- 4.7 CAN_TIME_11_SENSOR_ID
// 5. Save the temperature results to the SPI Flash (See SPI Flash section below)

// SPI Flash Brief
// * The SPI Flash is on CS1, not CS0
// * The SPI Flash has a Page size of of 64 bytes defined as SPI_FLASH_PAGE_SIZE in SPI.h
// * The SPI Flash has a total size of 4096 bytes defined as SPI_FLASH_SZ in SPI.h
// * A SPI Flash must be written to in pages, not bytes
// * A SPI Flash will return an error if you attemp to write to a page that is not erased
// * The SPI Flash expects the MOSI data to look like the following
//               | Flash Command | Page Number | Data 1 | Data 2 | ... | Data 64 |
// * The available flash commands area (SPI.h):
// ** SPI_FLASH_CMD_WRITE
// ** SPI_FLASH_CMD_READ
// ** SPI_FLASH_CMD_ERASE



#define SPI_FLASH_DATA_PTS_PER_PAGE  (SPI_FLASH_PAGE_SIZE / sizeof(SPI_FLASH_data_pt_t))
#define SPI_FLASH_PAGES_TOTAL        (SPI_FLASH_SZ / SPI_FLASH_PAGE_SIZE)

static uint8_t g_current_val = 0;
static uint8_t g_average_val = 0;
static uint64_t g_timestamp_current_val = 0;

SPI_FLASH_data_pt_t g_buffer[SPI_FLASH_DATA_PTS_PER_PAGE];
static uint8_t g_buffer_counter_val = 0;
static uint32_t g_page_counter_val = 0;

void can_packet_isr(uint32_t id, CAN_FRAME_TYPES type, uint8_t *data, uint8_t len) {
    // we can not save the page in the isr, too slow
    // so you need to accomodate for this in the main loop

    // Do something with the CAN packet
    if(id == CAN_AVG_TEMPERATURE_11_SENSOR_ID && type == CAN_DATA_FRAME) {

        printf("Average temperature data: %d\n", data[0]);
        g_buffer[g_buffer_counter_val].avg_temp = data[0];

    } else if(id == CAN_CURRENT_TEMP_11_SENSOR_ID && type == CAN_DATA_FRAME) {

        printf("Current temperature data: %d\n", data[0]);
        g_buffer[g_buffer_counter_val].current_temp = data[0];

    } else if(id == CAN_TIME_11_SENSOR_ID && type == CAN_DATA_FRAME) {

        printf("Time stamp data: %d, %d, %d, %d\n", data[3],data[2],data[1],data[0]);
        g_buffer[g_buffer_counter_val].time = (data[3] << 24) | (data[2] << 16) | (data[1] << 8)| (data[0] << 0);

    }
    else {
        printf("Unknown CAN packet received\n");
    }

    // printf("CAN packet received\n");
    // Clear the can interrupt before exit isr:
    can_clear_rx_packet_interrupt();
}


#define SLEEP_NOW_MS(ms)   std::this_thread::sleep_for(std::chrono::milliseconds(ms))

int main(int argc, char **argv) {
    // Configure the SPI and CAN controllers;
    can_write_config(CAN_HARDWARE_REGISTER, CAN_BAUD_RATE_100K | CAN_FORMAT_11BIT);
    spi_write_config(SPI_HARDWARE_REGISTER, SPI_CLK_1MHZ | SPI_CS_1);
    printf("CAN & SPI config set\n");

    //can_add_filter(0,0x00,0x00);
    can_add_filter(0,0x7FF,0x14F);
    can_add_filter(1,0x7FF,0x15F);
    can_add_filter(2,0x7FF,0x18F);
    printf("CAN filters set\n");

    // Add the CAN RX ISR
    can_add_rx_packet_interrupt(can_packet_isr);

    while(true) {
        // Send the CAN RTR frames to the BatteryTemperatureVehicleModule every 500ms
        // Once a full SPI Flash page size worth of data is received, save it to the SPI Flash
        can_send_new_packet(CAN_AVG_TEMPERATURE_11_SENSOR_ID, CAN_RTR_FRAME, nullptr, 0);
        can_send_new_packet(CAN_CURRENT_TEMP_11_SENSOR_ID, CAN_RTR_FRAME, nullptr, 0);
        can_send_new_packet(CAN_TIME_11_SENSOR_ID, CAN_RTR_FRAME, nullptr, 0);

        //printf("CAN RTR frames sent\n");

        // Save data to SPI flash when size of buffer is equal to page size 
        if (g_buffer_counter_val >= (SPI_FLASH_DATA_PTS_PER_PAGE - 1)) {

            SPI_xmit_t erase_page;
            erase_page.cmd = SPI_FLASH_CMD_ERASE;
            erase_page.data = (uint8_t*)&g_page_counter_val;

            printf("Erase Flash Page Number: %d\n", spi_write_data(&erase_page,SPI_CS_1));

            uint8_t write_data[(sizeof(g_page_counter_val) + sizeof(g_buffer))];
            memcpy(write_data,&g_page_counter_val,sizeof(g_page_counter_val));
            memcpy(write_data + sizeof(g_page_counter_val),&g_buffer,sizeof(g_buffer));

            SPI_xmit_t write_page;
            write_page.cmd = SPI_FLASH_CMD_WRITE;
            write_page.len = sizeof(g_buffer);
            write_page.data = (uint8_t*)&write_data;

            printf("Write Flash Page Number: %d\n", spi_write_data(&write_page,SPI_CS_1));

            // Reset buffer counter to 0 after flash page is written
            g_buffer_counter_val = 0;
            
            // Optional block to read flash page memory to verify the data
            /* 
            uint8_t buffer_read[(sizeof(g_page_counter_val) + sizeof(g_buffer))];

            memcpy(buffer_read,&g_page_counter_val,sizeof(g_page_counter_val));

            SPI_xmit_t read_page;
            read_page.cmd = SPI_FLASH_CMD_READ;
            read_page.len = sizeof(g_buffer);
            read_page.data = (uint8_t*)&buffer_read;

            
            printf("Read Flash Page Number: %d\n", spi_read_data(&read_page,SPI_CS_1));
            printf("Page content:\n");
            for(int i = 0; i <= 63; i++) {
                printf("%d,", buffer_read[i + sizeof(g_page_counter_val)]);
            }
            printf("\n");
            */

            if (g_page_counter_val >= (SPI_FLASH_SZ / SPI_FLASH_PAGE_SIZE - 1)) {
                // Reset flash page counter to 0 after flash is full
                g_page_counter_val = 0;
            }
            else {
                // Increment flash page counter after writing data to flash page
                g_page_counter_val += 1;
            }
        }
        else {
            // Increment buffer counter after CAN RTR frames have been send
            g_buffer_counter_val += 1;
        } 


        SLEEP_NOW_MS(500);


    }
    return 0;
}
