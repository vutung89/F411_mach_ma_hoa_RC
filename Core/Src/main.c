/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 * chuyen che do auto: dong thoi switch phai gai xuong duoi + nhan button A sang
 * chuyen che do ngat SBUS_OUT: nhan button B sang
 * auto_control_frame [10]= 0xAA + 0x55 + scaled_rc_channel[4](R-P-T-Y) + UAV_mode + RC_mode + crc16[2] (low + high)
 * request_channel_frame [40] = 0xAA + 0xAF + rc_channel_raw[18*2] + crc16 (low + high)
 *
 * UART config: set wordleght = 9bit (include parity bit), 2 stop bit
 * TX_SBUS_timer = 10 ms (dont use TX_SBUS_DMA)
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "crc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define UART_DEBUG		huart1

#define DISABLE_SBUS_OUT_ENABLE    	0x00
#define AUTONOMUOS_RC_ENABLE      	0x01

#define SBUS_FRAME_LENGTH    	25
#define SBUS_HEADER           	0x0F
#define SBUS_FOOTER           	0x00

#define SBUS_SIGNAL_OK          0x00
#define SBUS_SIGNAL_LOST        0x01
#define SBUS_SIGNAL_FAILSAFE    0x03


#define CHANNEL_COUNT			18
// #define CHANNEL_MIN_VALUE		272
// #define CHANNEL_MID_VALUE		992
// #define CHANNEL_MAX_VALUE		1712

#define CHANNEL_MIN_VALUE		0
#define CHANNEL_MID_VALUE		992
#define CHANNEL_MAX_VALUE		2048

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
uint16_t failsafe_status;
uint16_t SBUS_footer;
uint8_t buf[SBUS_FRAME_LENGTH];
uint16_t CH[CHANNEL_COUNT];

uint8_t rx_usb_buf[10];
bool rx_usb_data_ok = false;
uint16_t rx_usb_cmp = 0;
uint16_t received_crc, calculated_crc;  // CRC nhận được và CRC tính toán
uint8_t scaled_roll; // scaled_roll in range (0-255)
uint8_t scaled_throttle;
uint8_t scaled_pitch;
uint8_t scaled_yaw;
uint8_t UAV_Mode;
uint8_t RC_Mode;
// autonomous control: 4 channels: roll, pitch, throttle, yaw
uint16_t CH_auto[4] = {CHANNEL_MID_VALUE, CHANNEL_MID_VALUE, CHANNEL_MID_VALUE, CHANNEL_MID_VALUE}; //init mid value -> UAV dung yen
uint8_t pre_catch_buf[1] = {0x00}; // buffer to catch head byte of SBUS
int pre_init_counter = 0;
bool start_counting = false;
bool catch_signal = false;

bool tx_sbus_flag = false;
uint16_t tx_sbus_flag_count = 0;
uint16_t sbus_tx_cmp = 0;
bool encode_sbus_tx_done = false;
uint8_t sbus_tx_buffer[SBUS_FRAME_LENGTH];

bool autonomous_status = false; // Biến để kiểm tra xem có phải chế độ điều khiển tự động hay không
bool disable_sbus_out_status = false; // Biến để kiểm tra xem có cần tắt SBUS_OUT hay không
bool valid_sbusframe_status = false; // Biến để kiểm tra tính hợp lệ của khung SBUS nhận được
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/**
 * @brief Function to redirect printf to UART for Debugging
 * @param file File descriptor (not used)
 * @param pData Pointer to data to be sent
 * @param len Length of data to be sent
 * @return Number of bytes written on success, -1 on failure
 */
#ifdef UART_DEBUG
int _write(int file, char *pData, int len)
{
	// Gửi dữ liệu qua UART1 với timeout 100ms (blocking)
	HAL_StatusTypeDef status = HAL_UART_Transmit(&UART_DEBUG, (uint8_t*)pData, len, 100);

	// Trả về len nếu thành công, -1 nếu thất bại
	if (status == HAL_OK) {
		return len;
	} else {
		return -1;  // Gửi thất bại (timeout, error, v.v.)
	}
}
#endif
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Function to Update Failsafe Status from SBUS Buffer
 * @param sbus_buf Pointer to the SBUS buffer
 * @param failsafe_status Pointer to store the failsafe status
 * @return void
 */
void UpdateFailsafeStatus(uint8_t sbus_buf[SBUS_FRAME_LENGTH], uint16_t *failsafe_status)
{
	*failsafe_status = SBUS_SIGNAL_OK;
	if (sbus_buf[23] & (1 << 3))
		*failsafe_status = SBUS_SIGNAL_FAILSAFE;
	else if (sbus_buf[23] & (1 << 2))
		*failsafe_status = SBUS_SIGNAL_LOST;

	else
		*failsafe_status = SBUS_SIGNAL_OK;
}

/**
 * @brief Kiểm tra xem có phải chế độ điều khiển tự động hay không
 * @param CH Mảng chứa giá trị của 18 kênh
 * @return Trả về true nếu là chế độ điều khiển tự động, false nếu không
 */
bool IsAutonomousControl(uint16_t CH[CHANNEL_COUNT]){
	// Kiem tra channel 6 (Switch phai) va channel 8 (Button A)
	if (0 < CH[5] && CH[5] < 600 && 1200 < CH[7] && CH[7] < 2000) {
		// Neu channel 6 trong khoang (0, 600) va channel 8 trong khoang (1200, 2000), la che do tu dong
		return true;
	}
	return false;
}

bool IsDisableSbusOut(uint16_t CH[CHANNEL_COUNT]){

	// Kiem tra channel 9 (button B)
	if ( 1200 < CH[8] && CH[8] < 2000) {
		// Nếu channel 9 trong khoang (1200, 2000) (tương đương Button B bật), la che do ngắt sbus out
		return true;
	}
	return false;
}

bool IsValidSbusFrame(const uint8_t sbus_buf[SBUS_FRAME_LENGTH]) {
	// Kiểm tra header và footer
	if (sbus_buf[0] != SBUS_HEADER || sbus_buf[24] != SBUS_FOOTER) {
		printf("Invalid SBUS frame: Header %02X, Footer %02X\n", sbus_buf[0], sbus_buf[24]);
		return false; // Frame không hợp lệ
	}
	// Check digital channels
	// Use the passed-in buffer (sbus_buf) instead of global 'buf'
	if ((sbus_buf[23] & 0xF0) != 0x00) {
		printf("Invalid SBUS frame: Digital channels byte %02X\n", sbus_buf[23]);
		return false;
	}
	return true; // Frame hợp lệ
}

uint16_t cal_CH_auto_value(uint8_t scaled_value){
	uint32_t temp;
	temp = (scaled_value*(CHANNEL_MAX_VALUE- CHANNEL_MIN_VALUE))/255 + CHANNEL_MIN_VALUE;
	uint16_t CH_value;
	CH_value  = (uint16_t) temp;
	return CH_value;
}

uint8_t cal_scaled_value(uint16_t CH){
	uint32_t temp;
	temp = (255*(CH - CHANNEL_MIN_VALUE)) / (CHANNEL_MAX_VALUE - CHANNEL_MIN_VALUE);
	uint8_t scaled_value;
	scaled_value = (uint8_t) temp;
	return scaled_value;
}

/**
 * @brief Chuyển đổi giá trị kênh sang định dạng SBUS
 * @param CH Mảng chứa giá trị của 18 kênh
 * @return Trả về mảng đã mã hóa SBUS
 */
void ChannelToSbus(const uint16_t CH[CHANNEL_COUNT], uint8_t sbus[SBUS_FRAME_LENGTH]) {

	// Check if the channel values are within the valid range
	// for (int i = 0; i < 18; i++) {
	// 	if (CH[i] < CHANNEL_MIN_VALUE || CH[i] > CHANNEL_MAX_VALUE) {
	// 		return 1; // Invalid channel value
	// 	}
	// }

	// Header
	sbus[0] = SBUS_HEADER;

	sbus[1]  = (uint8_t)(CH[0] & 0xFF);
	sbus[2]  = (uint8_t)(((CH[0] >> 8) & 0x07) | ((CH[1] << 3) & 0xF8));
	sbus[3]  = (uint8_t)(((CH[1] >> 5) & 0x3F) | ((CH[2] << 6) & 0xC0));
	sbus[4]  = (uint8_t)((CH[2] >> 2) & 0xFF);
	sbus[5]  = (uint8_t)(((CH[2] >> 10) & 0x01) | ((CH[3] << 1) & 0xFE));
	sbus[6]  = (uint8_t)(((CH[3] >> 7) & 0x0F) | ((CH[4] << 4) & 0xF0));
	sbus[7]  = (uint8_t)(((CH[4] >> 4) & 0x7F) | ((CH[5] << 7) & 0x80));
	sbus[8]  = (uint8_t)((CH[5] >> 1) & 0xFF);
	sbus[9]  = (uint8_t)(((CH[5] >> 9) & 0x03) | ((CH[6] << 2) & 0xFC));
	sbus[10] = (uint8_t)(((CH[6] >> 6) & 0x1F) | ((CH[7] << 5) & 0xE0));
	sbus[11] = (uint8_t)((CH[7] >> 3) & 0xFF);
	sbus[12] = (uint8_t)(CH[8] & 0xFF);
	sbus[13] = (uint8_t)(((CH[8] >> 8) & 0x07) | ((CH[9] << 3) & 0xF8));
	sbus[14] = (uint8_t)(((CH[9] >> 5) & 0x3F) | ((CH[10] << 6) & 0xC0));
	sbus[15] = (uint8_t)((CH[10] >> 2) & 0xFF);
	sbus[16] = (uint8_t)(((CH[10] >> 10) & 0x01) | ((CH[11] << 1) & 0xFE));
	sbus[17] = (uint8_t)(((CH[11] >> 7) & 0x0F) | ((CH[12] << 4) & 0xF0));
	sbus[18] = (uint8_t)(((CH[12] >> 4) & 0x7F) | ((CH[13] << 7) & 0x80));
	sbus[19] = (uint8_t)((CH[13] >> 1) & 0xFF);
	sbus[20] = (uint8_t)(((CH[13] >> 9) & 0x03) | ((CH[14] << 2) & 0xFC));
	sbus[21] = (uint8_t)(((CH[14] >> 6) & 0x1F) | ((CH[15] << 5) & 0xE0));
	sbus[22] = (uint8_t)((CH[15] >> 3) & 0xFF);

	// Digital channels and failsafe flags
	sbus[23] = 0x00;

	// Thiết lập các kênh digital
	if (CH[16] == 1) {
		sbus[23] |= (1 << 0);  // CH17 (Digital)
	}

	if (CH[17] == 1) {
		sbus[23] |= (1 << 1);  // CH18 (Digital)
	}
	// Footer
	sbus[24] = SBUS_FOOTER;
}

/**
 * @brief Chuyển đổi dữ liệu SBUS sang mảng kênh
 * @param sbus Mảng chứa dữ liệu SBUS
 * @param CH Mảng để lưu giá trị của 18 kênh
 */
void SbusToChannel(const uint8_t sbus[SBUS_FRAME_LENGTH], uint16_t CH[CHANNEL_COUNT])
{
	// Kiểm tra header
	if (sbus[0] != SBUS_HEADER || sbus[24] != SBUS_FOOTER) {
		// Có thể reset mảng CH về giá trị mặc định nếu cần
		// memset(CH, 0, sizeof(uint16_t) * 18);
		// return;
		printf("Invalid SBUS header: %02X\n", sbus[0]);
	}

	CH[0]  = (sbus[1]      	| (sbus[2] << 8)) & 0x07FF;
	CH[1]  = ((sbus[2] >> 3) | (sbus[3] << 5)) & 0x07FF;
	CH[2]  = ((sbus[3] >> 6) | (sbus[4] << 2) | (sbus[5] << 10)) & 0x07FF;
	CH[3]  = ((sbus[5] >> 1) | (sbus[6] << 7)) & 0x07FF;
	CH[4]  = ((sbus[6] >> 4) | (sbus[7] << 4)) & 0x07FF;
	CH[5]  = ((sbus[7] >> 7) | (sbus[8] << 1) | ((sbus[9] & 0x03) << 9)) & 0x07FF;
	CH[6]  = ((sbus[9] >> 2) | (sbus[10] << 6)) & 0x07FF;
	CH[7]  = ((sbus[10] >> 5) | (sbus[11] << 3)) & 0x07FF;
	CH[8]  = (sbus[12] | (sbus[13] << 8)) & 0x07FF;
	CH[9]  = ((sbus[13] >> 3) | (sbus[14] << 5)) & 0x07FF;
	CH[10] = ((sbus[14] >> 6) | (sbus[15] << 2) | (sbus[16] << 10)) & 0x07FF;
	CH[11] = ((sbus[16] >> 1) | (sbus[17] << 7)) & 0x07FF;
	CH[12] = ((sbus[17] >> 4) | (sbus[18] << 4)) & 0x07FF;
	CH[13] = ((sbus[18] >> 7) | (sbus[19] << 1) | (sbus[20] << 9)) & 0x07FF;
	CH[14] = ((sbus[20] >> 2) | (sbus[21] << 6)) & 0x07FF;
	CH[15] = ((sbus[21] >> 5) | (sbus[22] << 3)) & 0x07FF;

	/**
	 * Lấy bit số 0 (bit thấp nhất) của byte thứ 24 trong mảng SBUS (sbus[23]).
	 * Nếu bit này bằng 1, thì CH[16] = 1; nếu bit này bằng 0, thì CH[16] = 0.
	 * Tương tự cho CH[17].
	 */
	CH[16] = (sbus[23] & (1 << 0)) ? 1 : 0;
	CH[17] = (sbus[23] & (1 << 1)) ? 1 : 0;
}

/**
 * @brief Kiểm tra mã hóa và giải mã Channel
 * @note Hàm này sẽ kiểm tra tính đúng đắn của mã hóa và giải mã Channel
 * @return void
 */
void test_channel_codec(void)
{
	// 1. Tạo mảng Channel mẫu
	uint16_t channel_test[CHANNEL_COUNT] = {
			272, 500, 800, 1000,
			1200, 1400, 1600, 1712,
			900, 1100, 1300, 1500,
			1700, 272, 1712, 992, 1, 0
	};
	uint8_t sbus_buf[SBUS_FRAME_LENGTH] = {0};
	uint16_t channel_result[CHANNEL_COUNT] = {0};

	// 2. Channel -> SBUS
	ChannelToSbus(channel_test, sbus_buf);

	// 3. SBUS -> Channel
	SbusToChannel(sbus_buf, channel_result);

	// 4. So sánh kết quả
	int match = 1;
	int channel_count = sizeof(channel_result) / sizeof(channel_result[0]);
	for (int i = 0; i < channel_count; i++) {
		if (channel_test[i] != channel_result[i]) {
			match = 0;
			break;
		}
	}

	if (match) {
		printf("Test PASSED: Channel encode/decode OK!\r\n");
	} else {
		printf("Test FAILED: Channel encode/decode mismatch!\r\n");
		printf("Original: ");
		for (int i = 0; i < channel_count; i++) printf("%4d ", channel_test[i]);
		printf("\r\nResult:   ");
		for (int i = 0; i < channel_count; i++) printf("%4d ", channel_result[i]);
		printf("\r\n");
	}
}

/**
 * @brief Kiểm tra mã hóa và giải mã SBUS
 * @note Hàm này sẽ kiểm tra tính đúng đắn của mã hóa và giải mã SBUS
 * @return void
 */
void test_sbus_codec()
{
	// 1. Mẫu thử SBUS
	uint8_t sbus_test[SBUS_FRAME_LENGTH] = {
			0x0F,0xE0,0x03,0x1F,0xF8,
			0xC0,0x07,0x3E,0x88,0x80,
			0x0F,0xD6,0x10,0x81,0x08,
			0x44,0xC0,0x07,0x3E,0x80,
			0xC3,0x1A,0x22,0x00,0x00
	};
	uint16_t CH_test[CHANNEL_COUNT] = {0};
	uint8_t sbus_result[SBUS_FRAME_LENGTH] = {0};

	// 2. Giải mã SBUS -> Channel
	SbusToChannel(sbus_test, CH_test);

	// 3. Mã hóa lại Channel -> SBUS
	ChannelToSbus(CH_test, sbus_result);

	// 4. So sánh kết quả
	int match = 1;
	int sbus_size = sizeof(sbus_result) / sizeof(sbus_result[0]);
	for (int i = 0; i < sbus_size; i++) {
		if (sbus_test[i] != sbus_result[i]) {
			match = 0;
			break;
		}
	}

	if (match) {
		printf("Test PASSED: SBUS encode/decode OK!\r\n");
	} else {
		printf("Test FAILED: SBUS encode/decode mismatch!\r\n");
		printf("Original: ");
		for (int i = 0; i < sbus_size; i++) printf("%02X ", sbus_test[i]);
		printf("\r\nResult:   ");
		for (int i = 0; i < sbus_size; i++) printf("%02X ", sbus_result[i]);
		printf("\r\n");
	}
}

/**
 * @brief Kiểm tra mã hóa và giải mã CRC
 * @note Hàm này sẽ kiểm tra tính đúng đắn của mã hóa và giải mã CRC-16
 * @return void
 */
void test_crc_data_codec(void)
{
	// 1. Mảng dữ liệu mẫu (8 byte dữ liệu + 2 byte CRC mẫu)
	uint8_t crc_data[10] = {0xAA, 0x55, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x02, 0x58, 0xF1};
	uint16_t crc_value, crc_rx;

	// 2. Tính CRC cho 8 byte đầu
	crc_value = calculate_crc16(crc_data, 8);

	// 3. Lấy CRC mẫu từ 2 byte cuối
	crc_rx = (crc_data[8]) | (crc_data[9] << 8);

	// 4. So sánh và in kết quả
	if (crc_value == crc_rx) {
		printf("Test PASSED: CRC encode/decode OK! (0x%04X)\r\n", crc_value);
	} else {
		printf("Test FAILED: CRC mismatch!\r\n");
		printf("Calculated CRC: 0x%04X, Received CRC: 0x%04X\r\n", crc_value, crc_rx);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance == htim3.Instance){
		tx_sbus_flag = true;
		tx_sbus_flag_count++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart6.Instance){
		// kiem tra head_byte
		if (pre_catch_buf[0] == 0x0F) start_counting = true;
		// bo qua 24 byte con lai
		if (pre_init_counter < 24 && start_counting == true)
		{
			pre_init_counter++;
			return;
		}
		else if (catch_signal == false)
		{
			catch_signal = true;
			memset(buf, 0, SBUS_FRAME_LENGTH);  // Clear buffer trước khi nhận frame mới
			HAL_UART_AbortReceive(&huart6); // thoat khoi ham receive dang chay
			HAL_UART_Receive_DMA(&huart6, buf, SBUS_FRAME_LENGTH); // bat dau nhan 25 byte SBUS
			return;

		}

		if (IsValidSbusFrame(buf))
		{
			SbusToChannel(buf, CH); // Chuyển đổi SBUS sang kênh
			if(AUTONOMUOS_RC_ENABLE){
				autonomous_status = IsAutonomousControl(CH); // Kiểm tra chế độ điều khiển tự động
			}
			if (DISABLE_SBUS_OUT_ENABLE){
				disable_sbus_out_status = IsDisableSbusOut(CH); // Kiểm tra chế độ tắt SBUS_OUT
			}
			UpdateFailsafeStatus(buf, &failsafe_status); // Cập nhật trạng thái failsafe từ SBUS
		}
		else
		{
			catch_signal = false; // Reset catch_signal để có thể nhận lại tín hiệu SBUS mới
			start_counting = false;
			pre_init_counter = 0;
			memset(buf, 0, SBUS_FRAME_LENGTH);  // Clear buffer để tránh dữ liệu cũ bị hỏng
			HAL_UART_AbortReceive(&huart6); // Hủy quá trình nhận hiện tại
			HAL_UART_Receive_DMA(&huart6, pre_catch_buf, 1);  
			/**
			 * Bắt đầu lại quá trình nhận 1 byte để kiểm tra head_byte
			 * Nếu nhận được head_byte đúng (0x0F), sẽ tiếp tục nhận 24 byte SBUS
			 * Nếu không nhận được head_byte đúng, sẽ tiếp tục nhận 1 byte để kiểm tra
			 */
			// return;
		}
	}
	if (huart->Instance == huart2.Instance){
		// Echo lại dữ liệu nhận được từ USB UART
		if (rx_usb_buf[0] == 0xAA && rx_usb_buf[1] == 0x55){
			HAL_UART_Transmit(&huart2, rx_usb_buf, 10, 10);
			// kiem tra crc
			calculated_crc = calculate_crc16(rx_usb_buf, 8); // CRC của dữ liệu (không bao gồm 2 byte CRC cuối)
			received_crc = (rx_usb_buf[8] | (rx_usb_buf[9] << 8)); // Lấy CRC nhận từ 2 byte cuối của dữ liệu
			if(calculated_crc == received_crc){
				scaled_roll = rx_usb_buf[2]; //scale_roll 0-255
				scaled_pitch = rx_usb_buf[3];
				scaled_throttle = rx_usb_buf[4];
				scaled_yaw = rx_usb_buf[5];
				UAV_Mode = rx_usb_buf[6]; // 0x7F
				RC_Mode = rx_usb_buf[7]; // mode = 1 -> rc_control; mode = 2 ->autonomous_control;

				rx_usb_data_ok = true;
			} else rx_usb_data_ok = false;
		}
		else if (rx_usb_buf[0] == 0xAA && rx_usb_buf[1] == 0xAF) {
			// kiem tra crc
			calculated_crc = calculate_crc16(rx_usb_buf, 8); // CRC của dữ liệu (không bao gồm 2 byte CRC cuối)
			received_crc = (rx_usb_buf[8] | (rx_usb_buf[9] << 8)); // Lấy CRC nhận từ 2 byte cuối của dữ liệu
			if(calculated_crc == received_crc){
				uint8_t ch_buf[36];
				for (int i = 0; i < 18; i++) {
					ch_buf[2*i] = (uint8_t)(CH[i] & 0xFF);
					ch_buf[2*i+1] = (uint8_t)((CH[i] >> 8) & 0xFF);
					/**
					 *  channels_raw = await asyncio.wait_for(self.reader.readexactly(36), timeout=0.2)
              channels = [channels_raw[i] | (channels_raw[i+1] << 8) for i in range(0, 36, 2)]
					 */
				}
				// HAL_UART_Transmit(&huart2, ch_buf, 36, 20);
				uint16_t crc_value = calculate_crc16(ch_buf, 36);
				uint8_t output_buf[2 + 36 +2];
				output_buf[0] = 0xAA;
				output_buf[1] = 0xAF;
				memcpy(&output_buf[2], ch_buf, 36);
				output_buf[38] = (uint8_t)(crc_value & 0xFF); // CRC low byte
				output_buf[39] = (uint8_t)((crc_value >> 8) & 0xFF); // CRC high byte
				HAL_UART_Transmit(&huart2, output_buf, 40, 20);
			}
		}

		HAL_UART_Receive_IT(&huart2, rx_usb_buf, 10);

	}
}

/**
 * @brief UART error callback
 * Called by HAL when an UART error occurs (framing, noise, overrun, etc.).
 * We use this to restart reception on huart6 so that reconnects/resynchronization work.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == huart6.Instance) {
		// Attempt to abort any ongoing transfers and restart reception of the head-byte
		HAL_UART_AbortReceive(&huart6);
		// Clear any DMA/USART error flags handled by HAL implicitly; add debug print
		printf("HAL_UART_ErrorCallback: huart6 error, restarting DMA receive\r\n");
		if (HAL_UART_Receive_DMA(&huart6, pre_catch_buf, 1) != HAL_OK) {
			// If restart failed, try a small delay and try again
			printf("HAL_UART_ErrorCallback: failed to restart huart6 DMA receive\r\n");
		}
	}
	if (huart->Instance == huart2.Instance) {
		// Restart receive
		HAL_UART_AbortReceive(&huart2);
		if (HAL_UART_Receive_IT(&huart2, rx_usb_buf, 10) != HAL_OK) {
			printf("[USB ERROR] Failed to restart after error\r\n");
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart2, rx_usb_buf, 10); // Start receiving data from USB UART
	// Start receiving 1 byte head check from SBUS (AirUnit) via DMA so we can detect frame head (0x0F)
	// This ensures huart6 RX is active from boot and will be restarted after disconnects.
	if (HAL_UART_Receive_DMA(&huart6, pre_catch_buf, 1) != HAL_OK) {
		printf("[WARN]: Failed to start huart6 DMA reception at init\r\n");
	}
	HAL_TIM_Base_Start_IT(&htim3); // Start timer for SBUS transmission
	printf("[INFO]: Setup All\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if (tx_sbus_flag == true){
			HAL_UART_Receive_DMA(&huart6, pre_catch_buf, 1);
			// UpdateFailsafeStatus(buf, &failsafe_status);
			memcpy(sbus_tx_buffer, buf, SBUS_FRAME_LENGTH);

			if (autonomous_status == true){
//				if (rx_usb_data_ok == true){
//					// ghi đè dữ liệu 4 kênh roll, pitch, throttle, yaw
//					CH_auto[0] = cal_CH_auto_value(scaled_roll);
//					CH_auto[1] = cal_CH_auto_value(scaled_pitch);
//					CH_auto[2] = cal_CH_auto_value(scaled_throttle);
//					CH_auto[3] = cal_CH_auto_value(scaled_yaw);
//
//					rx_usb_cmp++;
//					rx_usb_data_ok = false;
//				}
				CH_auto[0] = cal_CH_auto_value(scaled_roll);
				CH_auto[1] = cal_CH_auto_value(scaled_pitch);
				CH_auto[2] = cal_CH_auto_value(scaled_throttle);
				CH_auto[3] = cal_CH_auto_value(scaled_yaw);

				// Chuyển đổi kênh sang SBUS_OUT_AUTO
				sbus_tx_buffer[1] = (uint8_t) (CH_auto[0] & 0xFF);
				sbus_tx_buffer[2] = (uint8_t) ((CH_auto[0] >> 8) & 0x07) | ((CH_auto[1] << 3) & 0xF8);
				sbus_tx_buffer[3] = (uint8_t) ((CH_auto[1] >> 5) & 0x3F) | ((CH_auto[2] << 9) & 0xC0);
				sbus_tx_buffer[4] = (uint8_t) ((CH_auto[2] >> 2) & 0xFF);
				sbus_tx_buffer[5] = (uint8_t) ((CH_auto[2] >> 10) & 0x01) | ((CH_auto[3] << 1) & 0xFE);
				sbus_tx_buffer[6] = (uint8_t) ((CH_auto[3] >> 7) & 0x0F) | ((CH[4] << 4) & 0xF0);
				
			}
			sbus_tx_buffer[23] &= ~((1 << 2) | (1 << 3)); // Xóa cả 2 bit failsafe và lost
			// Gửi dữ liệu SBUS qua UART, tan so gui 100Hz (T_sbus = 10ms)
			// Kiểm tra xem có cần tắt SBUS_OUT hay không
			if (disable_sbus_out_status == false) {
				// Nếu cho phép SBUS_OUT, mã hóa dữ liệu SBUS và gửi
				HAL_UART_Transmit(&huart6, sbus_tx_buffer, SBUS_FRAME_LENGTH, 10);
			} else {
				// Nếu chặn SBUS_OUT, có thể gửi một mảng SBUS rỗng hoặc không gửi gì cả
				//        memset(sbus_tx_buffer, 0, sizeof(sbus_tx_buffer)); // Gửi mảng rỗng
				//        HAL_UART_Transmit(&huart6, sbus_tx_buffer, 25, 10);
			}

			sbus_tx_cmp++;
			tx_sbus_flag = false;
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15); // Toggle LED to indicate SBUS transmission
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 100000;
  huart6.Init.WordLength = UART_WORDLENGTH_9B;
  huart6.Init.StopBits = UART_STOPBITS_2;
  huart6.Init.Parity = UART_PARITY_EVEN;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
