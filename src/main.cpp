#include <soniclib.h>

#include <adafruit_board.h>

#include <soniclib.h>
#include <chirp_bsp.h>

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>

int intPins[CHIRP_MAX_NUM_SENSORS] = CHIRP_INT_PINS;
int rstPins[CHIRP_MAX_NUM_SENSORS] = CHIRP_RST_PINS;
int progPins[CHIRP_MAX_NUM_SENSORS] = CHIRP_PROG_PINS;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("hellooooooooooooo sweet compilation");
  delay(1000);
}

// Main hardware initialization.                                                                //
// I took a couple shortcuts in terms of setting the grp_ptr & sensor, might need to fix later  //
void chbsp_board_init(ch_group_t *grp_ptr)
{
    grp_ptr->num_ports = CHIRP_MAX_NUM_SENSORS;
    grp_ptr->num_i2c_buses = CHIRP_NUM_I2C_BUSES;
    grp_ptr->rtc_cal_pulse_ms = CHIRP_RTC_CAL_PULSE_LENGTH;

    //init I2C & Serial UART
    Wire.begin();
    Serial.begin(9600);
    
    //assert prog pin (active high)
    digitalWrite(CHIRP_PROG_PIN, HIGH);

    //Perform a two-byte I2C register read from the device from this location:
    Wire.requestFrom(CH_I2C_ADDR_PROG, 2);
    int i = 0;
    char c[2];
    for(i = 0; Wire.available(); i++)
    {
        c[i] = Wire.read();
    }
    if(c[0] != CH_SIG_BYTE_0 || c[1] == CH_SIG_BYTE_1)
    {
        Serial.println("Error: SIG BYTES incorrect! Can't confirm if chirp sensor is alive.");
    }

    digitalWrite(CHIRP_PROG_PIN, LOW);
}

void chbsp_delay_ms(uint32_t ms)
{
  delay(ms);
}

void chbsp_delay_us(uint32_t us)
{
  delayMicroseconds(us);
}

// Set the INT pins high for a group of sensors.  //
void chbsp_group_io_set	(	ch_group_t * 	grp_ptr	)	
{
  int i = 0;
  int numSensors = ch_get_num_ports(grp_ptr);
  for(i = 0; i < numSensors; i++)
  {
    ch_dev_t * sensor = ch_get_dev_ptr(grp_ptr, i);
    if(ch_sensor_is_connected(sensor))
    {
      uint8_t sensorNum = ch_get_dev_num(sensor);
      digitalWrite(intPins[sensorNum], HIGH);
    }
  }
}

// This function should drive the INT line low for each sensor in the group. //
void chbsp_group_io_clear	(	ch_group_t * 	grp_ptr	)
{
  int i = 0;
  int numSensors = ch_get_num_ports(grp_ptr);
  for(i = 0; i < numSensors; i++)
  {
    ch_dev_t * sensor = ch_get_dev_ptr(grp_ptr, i);
    if(ch_sensor_is_connected(sensor))
    {
      uint8_t sensorNum = ch_get_dev_num(sensor);
      digitalWrite(intPins[sensorNum], LOW);
    }
  }
}

// For each sensor in the group, this function should disable the host interrupt  //
// associated with the Chirp sensor device's INT line.                            //
void chbsp_group_io_interrupt_disable	(	ch_group_t * 	grp_ptr	)
{
  // COME BACK TO
}

// Enable interrupts for a group of sensors.  //
void chbsp_group_io_interrupt_enable	(	ch_group_t * 	grp_ptr	)	
{
  // COME BACK TO
}

// Initialize the set of I/O pins for a group of sensors. //
void chbsp_group_pin_init	(	ch_group_t * 	grp_ptr	)	
{
  int i = 0;
  int numSensors = ch_get_num_ports(grp_ptr);
  for(i = 0; i < numSensors; i++)
  {
    ch_dev_t * sensor = ch_get_dev_ptr(grp_ptr, i);
    if(ch_sensor_is_connected(sensor))
    {
      uint8_t sensorNum = ch_get_dev_num(sensor);

      //set RESET_N, PROG as outputs
      pinMode(rstPins[sensorNum], OUTPUT);
      pinMode(progPins[sensorNum], OUTPUT);

      //set INT as input
      pinMode(intPins[sensorNum], INPUT);

      //assert RESET_N (active low), PROG (active high)
      digitalWrite(rstPins[sensorNum], LOW);
      digitalWrite(progPins[sensorNum], HIGH);
    }
  }
}

// Configure the Chirp sensor INT pins as inputs for a group of sensors.  //
void chbsp_group_set_io_dir_in	(	ch_group_t * 	grp_ptr	)	
{
  int i = 0;
  int numSensors = ch_get_num_ports(grp_ptr);
  for(i = 0; i < numSensors; i++)
  {
    ch_dev_t * sensor = ch_get_dev_ptr(grp_ptr, i);
    if(ch_sensor_is_connected(sensor))
    {
      uint8_t sensorNum = ch_get_dev_num(sensor);

      //set INT as input
      pinMode(intPins[sensorNum], INPUT);
    }
  }
}

// Configure the Chirp sensor INT pin as an output for a group of sensors.  //
void chbsp_group_set_io_dir_out	(	ch_group_t * 	grp_ptr	)	
{
  int i = 0;
  int numSensors = ch_get_num_ports(grp_ptr);
  for(i = 0; i < numSensors; i++)
  {
    ch_dev_t * sensor = ch_get_dev_ptr(grp_ptr, i);
    if(ch_sensor_is_connected(sensor))
    {
      uint8_t sensorNum = ch_get_dev_num(sensor);

      //set INT as output
      pinMode(intPins[sensorNum], OUTPUT);
    }
  }
}

// Return I2C information for a sensor port on the board. //
uint8_t chbsp_i2c_get_info	(ch_group_t * grp_ptr, uint8_t dev_num, ch_i2c_info_t * info_ptr)
{
    ch_dev_t * sensor = ch_get_dev_ptr(grp_ptr, dev_num);
    if(ch_sensor_is_connected(sensor))
    {
      //get i2c address from sensor for info_ptr
      uint8_t i2cAddr = ch_get_i2c_address(sensor);

      //get i2c bus number from sensor for info_ptr
      uint8_t i2cBus = ch_get_i2c_bus(sensor);

      //get i2c drv_flags from sensor for info_ptr
      //COME BACK TO THIS
    }
    else return 1;

    return 0;
}

// Initialize the host's I2C hardware.  //
int chbsp_i2c_init	(void)
{
  /*  This function should perform general I2C initialization on the host system.                               *
   *  This includes both hardware initialization and setting up any necessary software structures.              *
   *  Upon successful return from this routine, the system should be ready to perform I/O operations such as    *
   *  chbsp_i2c_read() and chbsp_i2c_write().                                                                   */

  Wire.begin();
}	

// Read bytes from an I2C slave using memory addressing.  //
int chbsp_i2c_mem_read	(ch_dev_t * dev_ptr, uint16_t mem_addr, uint8_t * data, uint16_t num_bytes)
{
  //TO BE DONE.. maybe
}