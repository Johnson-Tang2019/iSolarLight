#define BLINKER_WIFI

#include <Blinker.h>
#include <Arduino.h>
#include<IRrecv.h>
#include<IRutils.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <esp_log.h>

// 硬件配置
#define PWM_PIN          23     // PWM输出引脚
#define PWM_CHANNEL      LEDC_CHANNEL_0  // PWM通道
#define PWM_TIMER        LEDC_TIMER_0    // PWM定时器
#define PWM_MODE         LEDC_LOW_SPEED_MODE
#define PWM_RESOLUTION   LEDC_TIMER_13_BIT // 13位分辨率 (8192级)
#define PWM_FREQ         5000            // 0.5kHz频率

char auth[] = "ad6375f89672";
char ssid[] = "Sindyhome01";
char pswd[] = "Tjs34333";

const int8_t powerPin = 32;
const int8_t batteryPin = 33;

int power_read = 0;
int battery_read = 0;

int mode = 0;//0 off 1 full 2 auto 3 on

IRrecv irrecv(21);
decode_results results;

// 全局变量
static const char *TAG = "PWM_Dimmer";
static uint32_t current_duty = 10;        // 当前占空比值

// 新建组件对象
BlinkerNumber Number1("num-power");
BlinkerNumber Number2("num-battery");


void dataRead(const String & data)
{
    BLINKER_LOG("Blinker readString: ", data);

    Blinker.vibrate();
    
    uint32_t BlinkerTime = millis();
    
    Blinker.print("millis", BlinkerTime);
}

void heartbeat()
{
    String str_power = String(power_read);
    String str_battery = String(battery_read);
    Number1.print(power_read);
    Number2.print(battery_read);
}

// PWM初始化函数
void pwm_init() {
    // 1. 配置PWM定时器
    ledc_timer_config_t timer_cfg = {
        .speed_mode = PWM_MODE,
        .duty_resolution = PWM_RESOLUTION,
        .timer_num = PWM_TIMER,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));
    
    // 2. 配置PWM通道
    ledc_channel_config_t channel_cfg = {
        .gpio_num = PWM_PIN,
        .speed_mode = PWM_MODE,
        .channel = PWM_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,      // 初始占空比为0
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    
    ESP_LOGI(TAG, "PWM initialized at %dHz with %d-bit resolution", PWM_FREQ, PWM_RESOLUTION);
}

// 设置亮度 (0-100%)
void set_brightness(uint8_t percent) {
    // 计算占空比值 (0-8191)
    uint32_t duty = (8191 * percent) / 100;
    
    // 更新占空比
    ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, duty));
    ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    
    current_duty = duty;
    ESP_LOGI(TAG, "Brightness set to %d%% (duty: %lu)", percent, duty);
}

// 渐变效果函数
void fade_brightness(uint8_t target_percent, uint16_t duration_ms) {
    uint32_t target_duty = (8191 * target_percent) / 100;
    uint32_t steps = duration_ms / 20;  // 每20ms一步
    int32_t duty_step = (target_duty - current_duty) / steps;
    
    ESP_LOGI(TAG, "Fading from %lu to %lu in %d steps", current_duty, target_duty, steps);
    
    for(uint32_t i = 0; i < steps; i++) {
        current_duty += duty_step;
        
        // 边界检查
        if((duty_step > 0 && current_duty > target_duty) || 
           (duty_step < 0 && current_duty < target_duty)) {
            current_duty = target_duty;
        }
        
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, current_duty));
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
        
        vTaskDelay(20 / portTICK_PERIOD_MS); // 20ms延迟
    }
    
    // 确保达到目标值
    if(current_duty != target_duty) {
        current_duty = target_duty;
        ESP_ERROR_CHECK(ledc_set_duty(PWM_MODE, PWM_CHANNEL, current_duty));
        ledc_update_duty(PWM_MODE, PWM_CHANNEL);
    }
    
    ESP_LOGI(TAG, "Fade complete. Current duty: %lu", current_duty);
}



void setup() {
  pwm_init();
  pinMode(22, INPUT);
  Serial.begin(115200);
  BLINKER_DEBUG.stream(Serial);
  fade_brightness(0, 3000);

  // 初始化blinker
  Blinker.begin(auth, ssid, pswd);
  Blinker.attachData(dataRead);
  Blinker.attachHeartbeat(heartbeat);

  irrecv.enableIRIn();
  Serial.println("红外已启动");

  int64_t power_read = 0;
  int64_t battery_read = 0;
}


void loop(){

    power_read = analogRead(powerPin);//0-4095
    battery_read = analogRead(batteryPin);//0-4095
    Blinker.run();

    if(irrecv.decode(&results)){
        if(results.value == 0x1FE807F){
            mode > 0 ? mode = 0 : mode = 1 ;
            Serial.println("switch1");
        } else if(results.value == 0x1FE48B7){
            mode = 2;
            Serial.println("switch2");
        } else if(results.value == 0x1FE7887){
            mode = 3;
            Serial.println("switch3");
        }
    }

    
    if(mode == 0){
        fade_brightness(0, 1500);
        Serial.println("off");
    }else if(mode == 3){
        fade_brightness(100,1500);
        Serial.println("full");
    }else if(mode == 2 && power_read < 2){
        Serial.println("auto:");
        if(digitalRead(22) == HIGH)
        {
            fade_brightness(50, 1500);
            Serial.println("somebody");
            delay(10000);
    
        }else {
            fade_brightness(10,1500);
            Serial.println("nobody");
        }
    }else{
        if(power_read < 2){
            fade_brightness(50,1500);
        }else {
            fade_brightness(0,1500);
        }
    }
    delay(200);
}
