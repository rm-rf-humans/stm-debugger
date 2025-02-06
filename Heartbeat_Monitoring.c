#include "stm32f4xx.h" 
#include "stm32f4xx_hal_can.h"
#include "stm32f4xx_hal_tim.h"
#include "stdbool.h"
#include "stdint.h"

#define NUM_DEVICES 4  
#define HEARTBEAT_INTERVAL 1000   
#define HEARTBEAT_TIMEOUT 1500    
#define MAX_JITTER 50
#define QUEUE_SIZE 255             

#define FREQ_CHECK_INTERVAL 1000
#define BUS_WARNING_THRESHOLD 1000

typedef enum {
    DEVICE_OK = 0,
    DEVICE_WARNING = 1,
    DEVICE_ERROR = 2,
    DEVICE_RESTARTED = 3
} DeviceStatus;

typedef struct {
    bool arr[QUEUE_SIZE];
    uint8_t last;
    uint8_t first;
} Queue;

typedef struct {
    uint8_t id;
    uint16_t last_heartbeat;  
    uint16_t last_interval;  
    uint8_t sequence;
    uint8_t status : 2;
    uint8_t delivered;
    uint8_t dropped;
    Queue history;
} Device;

// typedef struct {
//     uint32_t message_count;
//     uint32_t last_check_time;
//     uint32_t current_frequency;
//     uint32_t peak_frequency;
// } BusMonitor;

Device devices[NUM_DEVICES] = {
    {0x01, 0, 0, 0, DEVICE_ERROR, 0, 0, {{0}, 0, 0}},  
    {0x02, 0, 0, 0, DEVICE_ERROR, 0, 0, {{0}, 0, 0}},
    {0x03, 0, 0, 0, DEVICE_ERROR, 0, 0, {{0}, 0, 0}},
    {0x04, 0, 0, 0, DEVICE_ERROR, 0, 0, {{0}, 0, 0}} 
};

volatile uint32_t current_time_ms = 0;
// BusMonitor bus_monitor = {0, 0, 0, 0};

void log_error(const char* message, uint8_t device_id) {
    printf("[ERROR] Device 0x%X: %s\n", device_id, message);
}

void log_warning(const char* message, uint8_t device_id) {
    printf("[WARNING] Device 0x%X: %s\n", device_id, message);
}

void log_info(const char* message) {
    printf("[INFO] %s\n", message);
}

void push (Queue q, bool state){
    q.arr[q.last] = state;
    q.last = (q.last + 1) % QUEUE_SIZE; 
}

bool pop (Queue q){
    bool out = q.arr[q.first];
    q.first = (q.first + 1) % QUEUE_SIZE;
}

void handle_history(Device* d, bool state){
    if ((d->history.last + 1) % QUEUE_SIZE == d->history.first){
        bool omitted = pop(d->history);
        push(d->history, state);
        if (omitted){
            d->delivered--;
        }else{
            d->dropped--;
        }
        if(state){
            d->delivered++;
        }else{
            d->dropped--;
        }
    }
}

// void update_bus_frequency(void) {
//     bus_monitor.message_count++;
    
//     if (current_time_ms - bus_monitor.last_check_time >= FREQ_CHECK_INTERVAL) {
//         bus_monitor.current_frequency = bus_monitor.message_count;
        
//         if (bus_monitor.current_frequency > bus_monitor.peak_frequency) {
//             bus_monitor.peak_frequency = bus_monitor.current_frequency;
//         }
        
//         if (bus_monitor.current_frequency > BUS_WARNING_THRESHOLD) {
//             char msg[50];
//             sprintf(msg, "High bus usage: %lu msg/s", bus_monitor.current_frequency);
//             log_warning(msg, 0xFF);
//         }
        
//         bus_monitor.message_count = 0;
//         bus_monitor.last_check_time = current_time_ms;
//     }
// }


// void print_bus_stats(void) {
//     printf("\n=== Bus Statistics ===\n");
//     printf("Current frequency: %lu messages/second\n", bus_monitor.current_frequency);
//     printf("Peak frequency: %lu messages/second\n", bus_monitor.peak_frequency);
// }

void process_heartbeat(uint8_t sender_id, uint8_t sequence_num) {
    Device* sender;
    switch (sender_id)
    {
    case 0x01:
        sender = &devices[0];
        break;
    case 0x02:
        sender = &devices[1];
        break;
    case 0x03:
        sender = &devices[2];
        break;
    case 0x04:
        sender = &devices[3];
        break;
    default:
        break;
    }
    int32_t interval = current_time_ms - sender->last_heartbeat;
    if (sequence_num == 0 && sender->sequence != 0){
        sender->status = DEVICE_RESTARTED;
    }
    else if (sender->last_interval !=0 && interval - sender->last_interval > MAX_JITTER)
    {
        log_warning("High jitter detected!", sender_id);
        sender->status = DEVICE_WARNING;
    }
    else if (sequence_num = sender->sequence)
    {
        log_warning("Duplicate heartbeat received!", sender_id);
        sender->status = DEVICE_WARNING;
    } 
    else if (sequence_num < sender->sequence && sequence_num != 0) 
    {
        log_warning("Out-of-order heartbeat received!", sender_id);
        sender->status = DEVICE_WARNING;
    }
    else{
        sender->status = DEVICE_OK;
    }
    handle_history(&sender, true);
    sender->last_heartbeat = current_time_ms;
    sender->last_interval = interval;
    sender->sequence = sequence_num;
    return;    
}

void check_heartbeats() {
    for (int i = 0; i < NUM_DEVICES; i++) {
        uint16_t time_since_last = current_time_ms - devices[i].last_heartbeat;
        if (time_since_last > HEARTBEAT_TIMEOUT) {
            devices[i].status = DEVICE_ERROR;
            log_error("Lost heartbeat!", devices[i].id);
            handle_history(&devices[i], false);   
        }
    }
}

void CAN_Receive_Callback(uint8_t sender_id, uint8_t sequence_num) {
    // update_bus_frequency(); 
    switch (sender_id)
    {
    case 0x01 || 0x02|| 0x03 || 0x04:
        process_heartbeat(sender_id, sequence_num);
        break;
    default:
        break;
    }
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
    // bus_monitor.last_check_time = current_time_ms;
}

void main_loop() {
    while (1) {
    }
}

int main(){
    HAL_Init();
}
