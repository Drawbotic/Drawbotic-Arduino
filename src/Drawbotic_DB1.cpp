#include "Drawbotic_DB1.h"

//------- DB1 HAL Functions  -------//
void db1_hal_pinmode(uint8_t pin, db1_hal_pinmode_t mode)
{
    switch (mode)
    {
    case DB1_HAL_PIN_IN:
        pinMode(pin, INPUT);
        break;
    case DB1_HAL_PIN_IN_PULLUP:
        pinMode(pin, INPUT_PULLUP);
        break;
    case DB1_HAL_PIN_IN_ANALOG:
        pinMode(pin, INPUT);
        break;
    case DB1_HAL_PIN_OUT:
        pinMode(pin, OUTPUT);
        break;
    case DB1_HAL_PIN_OUT_PULLUP:
        pinMode(pin, OUTPUT);
        break;
    case DB1_HAL_PIN_OUT_PWM:
        pinMode(pin, OUTPUT);
        break;
    case DB1_HAL_PIN_OUT_SERVO:
        //Nothing
        break;
    }
}

uint8_t db1_hal_digital_read(uint8_t pin)
{
    return digitalRead(pin);
}

void db1_hal_set_pin(uint8_t pin, uint8_t val)
{
    digitalWrite(pin, val > 0 ? HIGH : LOW);
}

uint16_t db1_hal_analog_read(uint8_t pin)
{
    return analogRead(pin);
}

void db1_hal_analog_write(uint8_t pin, uint16_t duty)
{
    analogWrite(pin, duty);
}

void db1_hal_analog_resolution(uint8_t res)
{
    analogWriteResolution(res);
}

void db1_hal_attach_interrupt(uint8_t pin, db1_hal_interrupt_type_t type, db1_hal_int_callback_t handler)
{
    switch (type)
    {
    case DB1_HAL_INT_RISING:
        attachInterrupt(pin, handler, RISING);
        break;
    case DB1_HAL_INT_FALLING:
        attachInterrupt(pin, handler, FALLING);
        break;
    case DB1_HAL_INT_CHANGE:
        attachInterrupt(pin, handler, CHANGE);
        break;
    }
}

void db1_hal_detach_interrupt(uint8_t pin)
{
    detachInterrupt(pin);
}

bool db1_hal_i2c_read(uint8_t addr, uint8_t *data, uint8_t len)
{
    Wire.requestFrom(addr, len);
    for(int i = 0; i < len; i++)
        data[i] = Wire.read();
        
    return true;
}

bool db1_hal_i2c_write(uint8_t addr, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(addr);
    for(int i = 0; i < len; i++)
        Wire.write(data[i]);
        
    return Wire.endTransmission() == 0;
}

bool db1_hal_i2c_reg_read(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission();

    Wire.requestFrom(addr, len);
    for(int i = 0; i < len; i++)
        data[i] = Wire.read();
    
    return true;
}

bool db1_hal_i2c_reg_write(uint8_t addr, uint8_t reg, uint8_t *data, uint8_t len)
{
    Wire.beginTransmission(addr);
    Wire.write(reg);
    
    for(int i = 0; i < len; i++)
        Wire.write(data[i]);

    return Wire.endTransmission() == 0;
}

uint16_t db1_hal_i2c_reg_read16(uint8_t addr, uint8_t reg)
{
    uint16_t data = 0;

    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(addr, 2);
    while(Wire.available()) {
        data = Wire.read();
        data |= Wire.read() << 8;
    }
    return data;
}

uint32_t db1_hal_i2c_buffer_size()
{
    return 32;
}

void db1_hal_servo_write(uint8_t pin, uint8_t val)
{
    DB1.writeServo(pin, val);
}

void db1_hal_rgb_init(uint8_t pin, uint8_t count)
{
    //Nothing
}

void db1_hal_rgb_set(uint8_t pin, uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    DB1.setPixel(pin, index, r, g, b);
}

void db1_hal_rgb_fill(uint8_t pin, uint8_t r, uint8_t g, uint8_t b)
{
    DB1.fillPixels(pin, r, g, b);
}

void db1_hal_delay_ms(uint32_t ms)
{
    delay(ms);
}

uint32_t db1_hal_millis()
{
    return millis();
}

uint32_t db1_hal_micros()
{
    return micros();
}

//------- Singleton Specific -------//
Drawbotic_DB1 &Drawbotic_DB1::getInstance()
{
    static Drawbotic_DB1 instance;
    return instance;
}
Drawbotic_DB1 &DB1 = DB1.getInstance();

//------- Class Implementation -------//
Drawbotic_DB1::Drawbotic_DB1() : m_top_light(1, STAT_DOUT, NEO_GRB + NEO_KHZ800),
                                 m_lights   (DB1_LIGHT_COUNT, RGB_DOUT,  NEO_GRB + NEO_KHZ800)
{
    m_hal.pins.m1_dir_a     = M1_DIR_A;
    m_hal.pins.m1_dir_b     = M1_DIR_B;
    m_hal.pins.m2_dir_a     = M2_DIR_A;
    m_hal.pins.m2_dir_b     = M2_DIR_B;
    m_hal.pins.stat_dout    = STAT_DOUT;
    m_hal.pins.rgb_dout     = RGB_DOUT;
    m_hal.pins.m1_e_a       = M1_E_A;
    m_hal.pins.m1_e_b       = M1_E_B;
    m_hal.pins.m2_e_a       = M2_E_A;
    m_hal.pins.m2_e_b       = M2_E_B;
    m_hal.pins.led_en       = LED_EN;
    m_hal.pins.tof1_int     = TOF1_INT;
    m_hal.pins.tof2_int     = TOF2_INT;
    m_hal.pins.tof3_int     = TOF3_INT;
    m_hal.pins.tof1_en      = TOF1_EN;
    m_hal.pins.tof2_en      = TOF2_EN;
    m_hal.pins.tof3_en      = TOF3_EN;
    m_hal.pins.batt_lvl1    = BATT_LVL1;
    m_hal.pins.batt_lvl2    = BATT_LVL2;
    m_hal.pins.batt_lvl3    = BATT_LVL3;
    m_hal.pins.batt_lvl4    = BATT_LVL4;
    m_hal.pins.host_int_1   = HOST_INT_1;
    m_hal.pins.host_int_2   = HOST_INT_2;
    m_hal.pins.imu_int      = IMU_INT;
    m_hal.pins.imu_reset    = IMU_RESET;
    m_hal.pins.m1_pwm       = M1_PWM;
    m_hal.pins.m2_pwm       = M2_PWM;
    m_hal.pins.buzzer       = BUZZER;
    m_hal.pins.servo_pwm    = SERVO_PWM;
    m_hal.pins.ir_centre    = LINE1;
    m_hal.pins.ir_right     = LINE2;
    m_hal.pins.ir_left      = LINE3;
    m_hal.pins.ir_far_right = LINE4;
    m_hal.pins.ir_far_left  = LINE5;
    m_hal.pins.v_div_batt   = V_DIV_BATT;
    m_hal.pins.mic          = MIC;

    m_hal.set_pinmode       = db1_hal_pinmode;
    m_hal.digital_read      = db1_hal_digital_read;
    m_hal.digital_write     = db1_hal_set_pin;
    m_hal.analog_read       = db1_hal_analog_read;
    m_hal.analog_write      = db1_hal_analog_write;
    m_hal.analog_resolution = db1_hal_analog_resolution;
    m_hal.attach_interrupt  = db1_hal_attach_interrupt;
    m_hal.detach_interrupt  = db1_hal_detach_interrupt;
    m_hal.i2c_read          = db1_hal_i2c_read;
    m_hal.i2c_write         = db1_hal_i2c_write;
    m_hal.i2c_reg_read      = db1_hal_i2c_reg_read;
    m_hal.i2c_reg_write     = db1_hal_i2c_reg_write;
    m_hal.i2c_reg_read_16   = db1_hal_i2c_reg_read16;
    m_hal.i2c_buffer_size   = db1_hal_i2c_buffer_size;
    m_hal.servo_write       = db1_hal_servo_write;
    m_hal.rgb_init          = db1_hal_rgb_init;
    m_hal.rgb_set           = db1_hal_rgb_set;
    m_hal.rgb_fill          = db1_hal_rgb_fill;
    m_hal.delay_ms          = db1_hal_delay_ms;
    m_hal.millis            = db1_hal_millis;
    m_hal.micros            = db1_hal_micros;
}

int Drawbotic_DB1::init()
{
    db1_settings_t defaults = DB1_DEFAULT_SETTINGS;
    return init(defaults);
}

int Drawbotic_DB1::init(db1_settings_t settings)
{
    Wire.begin();
    
    m_pen_servo.attach(SERVO_PWM);
    
    m_top_light.begin();
    m_top_light.clear();
    m_top_light.show();
    
    m_lights.begin();
    m_lights.clear();
    m_lights.show();

    return db1_init(&m_hal, settings);
}

void Drawbotic_DB1::writeServo(uint32_t pin, uint8_t val)
{
    switch(pin)
    {
    case SERVO_PWM:
        m_pen_servo.write(val);
        break;
    }
}

void Drawbotic_DB1::setPixel(uint32_t pin, uint16_t index, uint8_t r, uint8_t g, uint8_t b)
{
    switch(pin)
    {
    case RGB_DOUT:
        m_lights.setPixelColor(index, r, g, b);
        m_lights.show();
        break;
    case STAT_DOUT:
        m_top_light.setPixelColor(index, r, g, b);
        m_top_light.show();
        break;
    }
}

void Drawbotic_DB1::fillPixels(uint32_t pin, uint8_t r, uint8_t g, uint8_t b)
{
    switch(pin)
    {
    case RGB_DOUT:
        m_lights.fill(Adafruit_NeoPixel::Color(r, g, b));
        m_lights.show();
        break;
    case STAT_DOUT:
        m_top_light.fill(Adafruit_NeoPixel::Color(r, g, b));
        m_top_light.show();
        break;
    }
}
