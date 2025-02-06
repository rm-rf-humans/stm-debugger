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

#define HEARTBEAT_1_HEX         0x01
#define HEARTBEAT_2_HEX         0x02
#define HEARTBEAT_3_HEX         0x03
#define HEARTBEAT_4_HEX         0x04


#define IMU_HEX                 0x05

#define THRUSTER_CURRENT_1_HEX  0x06
#define THRUSTER_CURRENT_2_HEX  0x07
#define THRUSTER_CURRENT_3_HEX  0x08
#define THRUSTER_CURRENT_4_HEX  0x09
#define THRUSTER_CURRENT_5_HEX  0x0A
#define THRUSTER_CURRENT_6_HEX  0x0B

#define THRUSTER_PWM_1_HEX      0x0C
#define THRUSTER_PWM_2_HEX      0x0D
#define THRUSTER_PWM_3_HEX      0x0E
#define THRUSTER_PWM_4_HEX      0x0F
#define THRUSTER_PWM_5_HEX      0x10
#define THRUSTER_PWM_6_HEX      0x11

#define INDICATOR_1_HEX         0x12
#define INDICATOR_2_HEX         0x13
#define INDICATOR_3_HEX         0x14
#define INDICATOR_4_HEX         0x15
#define INDICATOR_5_HEX         0x16
#define INDICATOR_6_HEX         0x17


#define ARM_1_HEX               0x1C
#define ARM_2_HEX               0x1D
#define ARM_3_HEX               0x1E
#define ARM_4_HEX               0x1F

#define ROV_DEPTH_HEX           0x20


typedef enum {
    DEVICE_OK = 0,
    DEVICE_WARNING = 1,
    DEVICE_ERROR = 2,
    DEVICE_RESTARTED = 3
} DeviceStatus;

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float imu_roll;
    float imu_pitch;
    float imu_yaw;
} Imu;

typedef struct {
    int id; // Store the ID to know which data is valid
    bool valid; // Flag to indicate if the data is valid
    Imu imu;
    float thruster_current_1;
    float thruster_current_2;
    float thruster_current_3;
    float thruster_current_4;
    float thruster_current_5;
    float thruster_current_6;
    float thruster_pwm_1;
    float thruster_pwm_2;
    float thruster_pwm_3;
    float thruster_pwm_4;
    float thruster_pwm_5;
    float thruster_pwm_6;
    float indicator_1;
    float indicator_2;
    float indicator_3;
    float indicator_4;
    float indicator_5;
    float indicator_6;
    bool heartbeat_1;
    bool heartbeat_2;
    bool heartbeat_3;
    bool heartbeat_4;
    float connection_percentage_1;
    float connection_percentage_2;
    float connection_percentage_3;
    float connection_percentage_4;
    float arm_1;
    float arm_2;
    float arm_3;
    float arm_4;
    float rov_depth;
} DecodedData;

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

void process_thruster_current(uint8_t id, float value, DecodedData * current_msg){
    switch (id)
    {
    case 1:
        current_msg->thruster_current_1 = value;
        break;
    case 2:
        current_msg->thruster_current_2 = value;
        break;
    case 3:
        current_msg->thruster_current_3 = value;
        break;
    case 4:
        current_msg->thruster_current_4 = value;
        break;
    case 5:
        current_msg->thruster_current_5 = value;
        break;
    case 6:
        current_msg->thruster_current_6 = value;
        break;
    default:
        break;
    }   
}

void process_thruster_pwm(uint8_t id, float value, DecodedData * current_msg){
    switch (id)
    {
    case 1:
        current_msg->thruster_pwm_1 = value;
        break;
    case 2:
        current_msg->thruster_pwm_2 = value;
        break;
    case 3:
        current_msg->thruster_pwm_3 = value;
        break;
    case 4:
        current_msg->thruster_pwm_4 = value;
        break;
    case 5:
        current_msg->thruster_pwm_5 = value;
        break;
    case 6:
        current_msg->thruster_pwm_6 = value;
        break;
    default:
        break;
    }   
}

void process_indicator(uint8_t id, float value, DecodedData * current_msg){
    switch (id)
    {
    case 1:
        current_msg->indicator_1 = value;
        break;
    case 2:
        current_msg->indicator_2 = value;
        break;
    case 3:
        current_msg->indicator_3 = value;
        break;
    case 4:
        current_msg->indicator_4 = value;
        break;
    case 5:
        current_msg->indicator_5 = value;
        break;
    case 6:
        current_msg->indicator_6 = value;
        break;
    default:
        break;
    }   
}

void process_arm(uint8_t id, float value, DecodedData * current_msg){
    switch (id)
    {
    case 1:
        current_msg->arm_1 = value;
        break;
    case 2:
        current_msg->arm_2 = value;
        break;
    case 3:
        current_msg->arm_3 = value;
        break;
    case 4:
        current_msg->arm_4 = value;
        break;
    default:
        break;
    }   
}

void process_depth(float value, DecodedData* current_msg){
    current_msg->rov_depth = value;
}
void process_imu(Imu value, DecodedData* current_msg){
    current_msg->imu = value;
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

void CAN_Receive_Callback(uint8_t sender_id, uint8_t* sequence, DecodedData* current_msg) {
    // update_bus_frequency(); 
    switch (sender_id)
    {
    case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
            process_heartbeat(sender_id, sequence[0]);
            break;

        // Group: IMU (0x05)
        case 0x05:
            Imu imu_data;
            memcpy(&imu_data, sequence, sizeof(Imu));
            process_imu(imu_data, current_msg);
            break;

        // Group: Thruster Currents (0x06 - 0x0B)
        case 0x06:
        case 0x07:
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B:
            float thruster_current;
            memcpy(&thruster_current, sequence, sizeof(float));
            process_thruster_current(sender_id - 5, thruster_current, current_msg);
            break;

        // Group: Thruster PWM (0x0C - 0x11)
        case 0x0C:
        case 0x0D:
        case 0x0E:
        case 0x0F:
        case 0x10:
        case 0x11:
            float thruster_pwm;
            memcpy(&thruster_pwm, sequence, sizeof(float));
            process_thruster_pwm(sender_id - 0x0B, thruster_pwm, current_msg);
            break;

        // Group: Indicators (0x12 - 0x17)
        case 0x12:
        case 0x13:
        case 0x14:
        case 0x15:
        case 0x16:
        case 0x17:
            float indicator;
            memcpy(&indicator, sequence, sizeof(float));
            process_indicator(sender_id - 0x11, indicator, current_msg);
            break;

        // Group: Arms (0x1C - 0x1F)
        case 0x1C:
        case 0x1D:
        case 0x1E:
        case 0x1F:
            float arm;
            memcpy(&arm, sequence, sizeof(float));
            process_arm(sender_id - 0x1B, indicator, current_msg);
            break;


        // Group: ROV Depth (0x20)
        case 0x20:
            float depth;
            memcpy(&depth, sequence, sizeof(float));
            process_depth(depth, current_msg);
            break;

        default:
            log_warning("Unkown device!",0x00)
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
