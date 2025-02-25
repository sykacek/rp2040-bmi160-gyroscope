#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "pico/flash.h"

#include "hardware/i2c.h"
#include "hardware/sync.h"
#include "hardware/flash.h"

#include "driver/bmi160.h"
#include "driver/bmi160_def.h"

#define I2C_ADDRESS		(0x68)

#define LED			(25)
#define HALT_PIN		(10)

#define NVS_SIZE		(4096)
#define NVS_SECTORS		(446)
#define PAGE_SIZE		(256)
#define PAGE_SECTORS		(NVS_SECTORS * NVS_SIZE / PAGE_SIZE)

#define FLASH_WRITE_START	(0x42000)

#define FLASH_SECTORS		(256)
#define NVS_HEADER_IDENTIFIER	"DATA AB DATA ABC"

#define GYRO_RANGE_125_STR		"gyrorange: 125"
#define GYRO_RANGE_250_STR		"gyrorange: 250"
#define GYRO_RANGE_500_STR		"gyrorange: 500"
#define GYRO_RANGE_1000_STR		"gyrorange: 1000"
#define GYRO_RANGE_2000_STR		"gyrorange: 2000"

#define ACCEL_RANGE_2G_STR		"accelrange: 2G"
#define ACCEL_RANGE_4G_STR		"accelrange: 4G"
#define ACCEL_RANGE_8G_STR		"accelrange: 8G"
#define ACCEL_RANGE_16G_STR		"accelrange: 16G"


typedef struct tim_arg {
	int counter;
	uint16_t buffer_offset;
	uint32_t offset;
	unsigned char buffer[256];
	struct bmi160_sensor_data bmi160_accel;
	struct bmi160_sensor_data bmi160_gyro;
} tim_arg_t;

static queue_t call_queue;

static tim_arg_t tim_arg = {
	.counter = 0,
	.offset = FLASH_WRITE_START,
	.buffer = NVS_HEADER_IDENTIFIER,
	.buffer_offset = 16,
};

static int8_t driver_i2c_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
	int ret;
	ret = i2c_write_blocking(i2c_default, dev_addr, &reg_addr, 1, true);
	if(ret == PICO_ERROR_GENERIC){
		printf("failed to write to adress 0x%x\n", dev_addr);
		return ret;
	}

	ret = i2c_read_blocking(i2c_default, dev_addr, data, len, false);
	if(ret == PICO_ERROR_GENERIC){
		printf("failed to read from adress 0x%x\n", dev_addr);
		return ret;
	}

	return 0;
}

static int8_t driver_i2c_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
	int ret = 0;
	uint8_t pnt[len + 1];
	pnt[0] = reg_addr;

	for(uint16_t i = 0; i < len; ++i)
		pnt[i + 1] = data[i];

	ret = i2c_write_blocking(i2c_default, dev_addr, pnt, len + 1, false);
	if(ret == PICO_ERROR_GENERIC){
		printf("failed to write to adress 0x%x\n", dev_addr);
		return ret;
	}

	return 0;
}

static void driver_i2c_delay(uint32_t millis){
	sleep_ms(millis);
}

static  struct bmi160_dev mychip_dev = {
    .read = driver_i2c_read_reg,
    .delay_ms = driver_i2c_delay,
    .write = driver_i2c_write_reg,
    .intf = BMI160_I2C_INTF,
    .id = 0x68,
};

void __not_in_flash_func(call_write_flash)(void *param){
	tim_arg_t *arg = (tim_arg_t *)param;

	flash_range_program(arg->offset, arg->buffer, 256);
	arg->offset += 256;

	printf("writing to flash offset %lx\n", arg->offset);
}

void __not_in_flash_func(call_erase_flash)(void *param){
	tim_arg_t *arg = (tim_arg_t *)param;
	flash_range_erase(arg->offset, NVS_SIZE * NVS_SECTORS);
}

void core1_entry(void) {
	flash_safe_execute_core_init();
	flash_safe_execute(call_erase_flash, &tim_arg, 200);

	int queue_res;

	while(1){	
		queue_remove_blocking(&call_queue, &queue_res);
		gpio_put(LED, queue_res % 2);
	
		if(queue_res % 20 == 0){
			printf("writing to flash returned %d\n", flash_safe_execute(call_write_flash, &tim_arg, 200));
			tim_arg.buffer_offset = 16;
			memset(tim_arg.buffer + 16, 0, 240);
		}
	}
}

static bool tim_callback(repeating_timer_t *rt){
	(void)(rt);

	tim_arg_t *arg = (tim_arg_t *)(rt->user_data);
	int val = arg->counter;
	queue_add_blocking(&call_queue, &val);
	
	val++;
	arg->counter = val;

	bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &(arg->bmi160_accel), &(arg->bmi160_gyro), &mychip_dev);

	memcpy(arg->buffer + arg->buffer_offset, &(arg->bmi160_accel), 6);
	arg->buffer_offset += 6;

	memcpy(arg->buffer + arg->buffer_offset, &(arg->bmi160_gyro), 6);
	arg->buffer_offset += 6;

	return true;
}

int main() {
	while(!stdio_init_all()){
		tight_loop_contents();
	}
	gpio_init(LED);
	gpio_set_dir(LED, GPIO_OUT);

	gpio_init(HALT_PIN);
	gpio_set_dir(HALT_PIN, GPIO_IN);

	if(gpio_get(HALT_PIN) == 1){
		int state = 1;
		while(1){
			puts("please use picotool to extract the data");
			gpio_put(LED, state);
			state = !state;

			sleep_ms(1000);
		}
	}
	i2c_init(i2c_default, 100 * 1000);
	gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
	gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
	gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
	gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

	int ret = bmi160_init(&mychip_dev);
	if(ret)
		printf("error: bmi160_init returned %d\n", ret);

        mychip_dev.accel_cfg.odr = BMI160_ACCEL_ODR_200HZ;
        mychip_dev.accel_cfg.range = BMI160_ACCEL_RANGE_16G;
        mychip_dev.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
        mychip_dev.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

        mychip_dev.gyro_cfg.odr = BMI160_GYRO_ODR_200HZ;
        mychip_dev.gyro_cfg.range = BMI160_GYRO_RANGE_500_DPS;
        mychip_dev.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE;
        mychip_dev.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;


	char *pnt = (char *)tim_arg.buffer + 128;
	switch(mychip_dev.gyro_cfg.range){
	case(BMI160_GYRO_RANGE_125_DPS):{
		memcpy(pnt, GYRO_RANGE_125_STR, 15);
		break;
	}
	case(BMI160_GYRO_RANGE_250_DPS):{
		memcpy(pnt, GYRO_RANGE_250_STR, 15);
		break;
	}
	case(BMI160_GYRO_RANGE_500_DPS):{
		memcpy(pnt, GYRO_RANGE_500_STR, 15);
		break;
	}
	case(BMI160_GYRO_RANGE_1000_DPS):{
		memcpy(pnt, GYRO_RANGE_1000_STR, 16);
		break;
	}
	case(BMI160_GYRO_RANGE_2000_DPS):{
		memcpy(pnt, GYRO_RANGE_2000_STR, 16);
		break;
	}
	}

	pnt += 32;
	switch(mychip_dev.accel_cfg.range){
	case(BMI160_ACCEL_RANGE_2G):{
		memcpy(pnt, ACCEL_RANGE_2G_STR, 15);
		break;
	}
	case(BMI160_ACCEL_RANGE_4G):{
		memcpy(pnt, ACCEL_RANGE_4G_STR, 15);
		break;
	}
	case(BMI160_ACCEL_RANGE_8G):{
		memcpy(pnt, ACCEL_RANGE_8G_STR, 15);
		break;
	}
	case(BMI160_ACCEL_RANGE_16G):{
		memcpy(pnt, ACCEL_RANGE_16G_STR, 16);
		break;
	}
	}

        ret = bmi160_set_sens_conf(&mychip_dev);
	if(ret)
		printf("error: bmi160_set_sens_conf returned %d\n", ret);

	queue_init(&call_queue, sizeof(int), 1);

	flash_safe_execute_core_init();
	multicore_launch_core1(core1_entry);

	repeating_timer_t tim;
	add_repeating_timer_ms(100, tim_callback, (void *)&tim_arg, &tim);

	while(1){
		;
	}

	return 0;
}
