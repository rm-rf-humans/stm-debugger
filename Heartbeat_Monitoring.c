#include "stm32f4xx.h"  
#include "can.h"
#include "tim.h"

#define NUM_DEVICES 4  
#define HEARTBEAT_INTERVAL 1000   
#define HEARTBEAT_TIMEOUT 1500    
#define MAX_JITTER 50             

#define FREQ_CHECK_INTERVAL 1000
#define BUS_WARNING_THRESHOLD 1000

typedef enum {
    DEVICE_OK = 0,
    DEVICE_WARNING = 1,
    DEVICE_ERROR = 2,
    DEVICE_RESTARTED = 3
} DeviceStatus;

typedef struct {
    uint8_t id;
    uint16_t last_heartbeat;  
    uint16_t last_interval;  
    uint8_t sequence;
    uint8_t status : 2;  
} Device;

typedef struct {
    uint32_t message_count;
    uint32_t last_check_time;
    uint32_t current_frequency;
    uint32_t peak_frequency;
} BusMonitor;

Device devices[NUM_DEVICES] = {
    {0x01, 0, 0, 0, DEVICE_ERROR},  
    {0x02, 0, 0, 0, DEVICE_ERROR},  
    {0x03, 0, 0, 0, DEVICE_ERROR},  
    {0x10, 0, 0, 0, DEVICE_ERROR}   
};

volatile uint32_t current_time_ms = 0;
BusMonitor bus_monitor = {0, 0, 0, 0};

void log_error(const char* message, uint8_t device_id) {
    printf("[ERROR] Device 0x%X: %s\n", device_id, message);
}

void log_warning(const char* message, uint8_t device_id) {
    printf("[WARNING] Device 0x%X: %s\n", device_id, message);
}

void log_info(const char* message) {
    printf("[INFO] %s\n", message);
}

void update_bus_frequency(void) {
    bus_monitor.message_count++;
    
    if (current_time_ms - bus_monitor.last_check_time >= FREQ_CHECK_INTERVAL) {
        bus_monitor.current_frequency = bus_monitor.message_count;
        
        if (bus_monitor.current_frequency > bus_monitor.peak_frequency) {
            bus_monitor.peak_frequency = bus_monitor.current_frequency;
        }
        
        if (bus_monitor.current_frequency > BUS_WARNING_THRESHOLD) {
            char msg[50];
            sprintf(msg, "High bus usage: %lu msg/s", bus_monitor.current_frequency);
            log_warning(msg, 0xFF);
        }
        
        bus_monitor.message_count = 0;
        bus_monitor.last_check_time = current_time_ms;
    }
}


void print_bus_stats(void) {
    printf("\n=== Bus Statistics ===\n");
    printf("Current frequency: %lu messages/second\n", bus_monitor.current_frequency);
    printf("Peak frequency: %lu messages/second\n", bus_monitor.peak_frequency);
}

void process_heartbeat(uint8_t sender_id, uint8_t sequence_num) {
    for (int i = 0; i < NUM_DEVICES; i++) {
        if (devices[i].id == sender_id) {
            uint16_t interval = current_time_ms - devices[i].last_heartbeat;

            if (sequence_num == 0 || (sequence_num < devices[i].sequence && devices[i].sequence - sequence_num > 10)) {
                log_warning("Device may have restarted!", sender_id);
                devices[i].status = DEVICE_RESTARTED;
            }

            if (devices[i].last_interval > 0 && abs((int32_t)(interval - devices[i].last_interval)) > MAX_JITTER) {
                log_warning("High jitter detected!", sender_id);
            }

            if (sequence_num == devices[i].sequence) {
                log_warning("Duplicate heartbeat received!", sender_id);
                return;
            }

            if (sequence_num < devices[i].sequence && sequence_num != 0) {
                log_warning("Out-of-order heartbeat received!", sender_id);
            }

            devices[i].last_heartbeat = current_time_ms;
            devices[i].last_interval = interval;
            devices[i].sequence = sequence_num;
            devices[i].status = DEVICE_OK;
            return;
        }
    }
}

void check_heartbeats() {
    for (int i = 0; i < NUM_DEVICES; i++) {
        uint16_t time_since_last = current_time_ms - devices[i].last_heartbeat;

        if (time_since_last > HEARTBEAT_TIMEOUT) {
            devices[i].status = DEVICE_ERROR;
            log_error("Lost heartbeat!", devices[i].id);
        } else if (time_since_last > HEARTBEAT_INTERVAL) {
            devices[i].status = DEVICE_WARNING;
            log_warning("Delayed heartbeat detected!", devices[i].id);
        }
    }
}

void CAN_Receive_Callback(uint8_t sender_id, uint8_t sequence_num) {
    update_bus_frequency(); 
    process_heartbeat(sender_id, sequence_num);
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {  
        TIM2->SR &= ~TIM_SR_UIF;
        check_heartbeats();
        
        if (current_time_ms % 5000 == 0) {
            print_bus_stats();
        }
    }
}

void SysTick_Handler(void) {
    current_time_ms++;
}

void setup() {
    SysTick_Config(SystemCoreClock / 1000);  
    TIM2->CR1 |= TIM_CR1_CEN;
    bus_monitor.last_check_time = current_time_ms;
}

void main_loop() {
    while (1) {
    }
}
