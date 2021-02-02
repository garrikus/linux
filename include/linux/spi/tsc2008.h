#ifndef __LINUX_SPI_TSC2008_H
#define __LINUX_SPI_TSC2008_H

#define TS_POLL_DELAY  1 /* ms delay between samples */
#define TS_POLL_PERIOD 1 /* ms delay between samples */

#define MAX_12BIT     (1 << 12)
#define MAX_8BIT      (1 << 8)

#define TSC2008_START_CMD       (0x1 << 7)

#define TSC2008_MEASURE_TEMP1   (0x0 << 4)
#define TSC2008_MEASURE_Y       (0x1 << 4)
#define TSC2008_SETUP           (0x2 << 4)
#define TSC2008_MEASURE_Z1      (0x3 << 4)
#define TSC2008_MEASURE_Z2      (0x4 << 4)
#define TSC2008_MEASURE_X       (0x5 << 4)
#define TSC2008_MEASURE_AUX     (0x6 << 4)
#define TSC2008_MEASURE_TEMP2   (0x7 << 4)

#define TSC2008_USE_50K         (0x0 << 3)
#define TSC2008_USE_90K         (0x1 << 3)

#define TSC2008_ENABLE_MAV      (0x0 << 2)
#define TSC2008_DISABLE_MAV     (0x1 << 2)

#define TSC2008_SOFTWARE_RESET    (0x1 << 0)

#define TSC2008_POWER_OFF_IRQ_EN  (0x0 << 0)
#define TSC2008_ADC_ON_IRQ_DIS0   (0x1 << 0)
#define TSC2008_ADC_OFF_IRQ_EN    (0x2 << 0)
#define TSC2008_ADC_ON_IRQ_DIS1   (0x3 << 0)

#define TSC2008_12BIT (0x0 << 1)
#define TSC2008_8BIT  (0x1 << 1)

#define ADC_ON_12BIT  (TSC2008_START_CMD | TSC2008_12BIT | TSC2008_POWER_OFF_IRQ_EN)
#define ADC_ON_8BIT (TSC2008_START_CMD | TSC2008_8BIT | TSC2008_POWER_OFF_IRQ_EN)

/* usec, SCLK=25MHz w/o MAV */
#define TX_DELAY  (40)
#define RX_DELAY  (40)

struct ts_event {
  uint16_t  x;
  uint16_t  y;
  uint16_t  z1, z2;
};

struct tsc2008 {
  struct device     *dev;          
  struct input_dev  *input;        
  struct delayed_work work;        
  char   phys[32];                 

  struct spi_device *spi;         
  struct spi_message  msg;        
  struct spi_transfer xfer[8];     
  uint8_t tx_buf[4];              

  uint16_t model;                  
  uint16_t x_plate_ohms;            
  uint16_t max_bits;
  uint16_t min_x;
  uint16_t max_x;
  uint16_t min_y;
  uint16_t max_y;
  uint16_t min_z;
  uint16_t max_z;
  uint16_t fuzz_x;
  uint16_t fuzz_y;
  uint16_t fuzz_z;

  uint16_t bit_rate;
  uint32_t gpio;               
  bool pendown;                     
  int  irq;                        
  int  (*get_pendown_state)(uint32_t gpio);   
};
#endif
