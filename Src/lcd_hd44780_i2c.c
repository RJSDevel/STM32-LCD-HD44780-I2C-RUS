/**
 * Copyright Nikita Bulaev 2017
 * Russia language support Yagupov Ruslan
 *
 * STM32 HAL libriary for LCD display based on HITACHI HD44780U chip.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include "stdlib.h"
#include "string.h"
#include "lcd_hd44780_i2c.h"


uint8_t lcdCommandBuffer[6] = {0x00};

static LCDParams lcdParams;

static bool lcdWriteByte(uint8_t rsRwBits, uint8_t * data);

static uint8_t currentCurson = 0;


#define RUSSIAN_LOCALIZATION_COUNT 48

uint8_t russian_alphabet[RUSSIAN_LOCALIZATION_COUNT][9] = {
		{ 'á', 0x03, 0x0C, 0x10, 0x1E, 0x11, 0x11, 0x0E },
		{ 'â', 0x00, 0x00, 0x1E, 0x11, 0x1E, 0x11, 0x1E },
		{ 'ã', 0x00, 0x00, 0x1E, 0x10, 0x10, 0x10, 0x10 },
		{ 'ä', 0x00, 0x00, 0x06, 0x0A, 0x0A, 0xFF, 0x11 },
		{ '¸', 0x0A, 0x00, 0x0E, 0x11, 0xFF, 0x10, 0x0F },
		{ 'æ', 0x00, 0x00, 0x15, 0x15, 0x0E, 0x15, 0x15 },
		{ 'ç', 0x00, 0x00, 0x0E, 0x11, 0x06, 0x11, 0x0E },
		{ 'è', 0x00, 0x00, 0x11, 0x13, 0x15, 0x19, 0x11 },
		{ 'é', 0x0A, 0x04, 0x11, 0x13, 0x15, 0x19, 0x11 },
		{ 'ê', 0x00, 0x00, 0x12, 0x14, 0x18, 0x14, 0x12 },
		{ 'ë', 0x00, 0x00, 0x07, 0x09, 0x09, 0x09, 0x11 },
		{ 'ì', 0x00, 0x00, 0x11, 0x1b, 0x15, 0x11, 0x11 },
		{ 'í', 0x00, 0x00, 0x11, 0x11, 0x1f, 0x11, 0x11 },
		{ 'ï', 0x00, 0x00, 0x1f, 0x11, 0x11, 0x11, 0x11 },
		{ 'ò', 0x00, 0x00, 0x1f, 0x04, 0x04, 0x04, 0x04 },
		{ 'ó', 0x00, 0x00, 0x11, 0x11, 0x0f, 0x01, 0x0e },
		{ 'ô', 0x00, 0x00, 0x1f, 0x15, 0x15, 0x1f, 0x04 },
		{ 'ö', 0x00, 0x00, 0x12, 0x12, 0x12, 0x1f, 0x01 },
		{ '÷', 0x00, 0x00, 0x11, 0x11, 0x0f, 0x01, 0x01 },
		{ 'ø', 0x00, 0x00, 0x15, 0x15, 0x15, 0x15, 0x1f },
		{ 'ù', 0x00, 0x00, 0x15, 0x15, 0x15, 0x1f, 0x01 },
		{ 'ú', 0x00, 0x00, 0x18, 0x0e, 0x09, 0x09, 0x0e },
		{ 'û', 0x00, 0x00, 0x11, 0x11, 0x1d, 0x15, 0x1d },
		{ 'ü', 0x00, 0x00, 0x10, 0x1e, 0x11, 0x11, 0x1e },
		{ 'ý', 0x00, 0x00, 0x1e, 0x01, 0x0f, 0x01, 0x1e },
		{ 'þ', 0x00, 0x00, 0x17, 0x15, 0x1d, 0x15, 0x17 },
		{ 'ÿ', 0x00, 0x00, 0x0d, 0x13, 0x0f, 0x05, 0x09 },

		{ 'Á', 0xFF, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x1E },
		{ 'Ã', 0xFF, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10 },
		{ 'Ä', 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 },
		{ 'Æ', 0x11, 0x15, 0x15, 0x0E, 0x15, 0x15, 0x11 },
		{ 'Ç', 0x0e, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0e },
		{ 'È', 0x11, 0x11, 0x11, 0x13, 0x15, 0x19, 0x11 },
		{ 'É', 0x0e, 0x00, 0x11, 0x13, 0x15, 0x19, 0x11 },
		{ 'Ë', 0x0F, 0x09, 0x09, 0x09, 0x09, 0x11, 0x11 },
		{ 'Ï', 0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 },
		{ 'Ó', 0x11, 0x11, 0x11, 0x0f, 0x01, 0x11, 0x0e },
		{ 'Ô', 0x0e, 0x15, 0x15, 0x15, 0x0e, 0x04, 0x04 },
		{ 'Ö', 0x12, 0x12, 0x12, 0x12, 0x12, 0x1f, 0x01 },
		{ '×', 0x11, 0x11, 0x11, 0x0f, 0x01, 0x01, 0x01 },
		{ 'Ø', 0x11, 0x15, 0x15, 0x15, 0x15, 0x15, 0x1f },
		{ 'Ù', 0x10, 0x15, 0x15, 0x15, 0x15, 0x1F, 0x01 },
		{ 'Ú', 0x18, 0x08, 0x08, 0x0e, 0x09, 0x09, 0x0e },
		{ 'Û', 0x11, 0x11, 0x19, 0x15, 0x15, 0x15, 0x19 },
		{ 'Ü', 0x10, 0x10, 0x1e, 0x11, 0x11, 0x11, 0x1e },
		{ 'Ý', 0x1e, 0x01, 0x01, 0x0f, 0x01, 0x01, 0x1e },
		{ 'Þ', 0x12, 0x15, 0x15, 0x1D, 0x15, 0x15, 0x12 },
		{ 'ß', 0x0F, 0x11, 0x11, 0x0F, 0x05, 0x09, 0x11 }
};

#define RUSSIAN_ENG_SAME_COUNT 17

uint8_t russian_eng_same[][2] = {
		{'à', 'a'},
		{'å', 'e'},
		{'î', 'o'},
		{'ð', 'p'},
		{'ñ', 'c'},
		{'õ', 'x'},

		{'À', 'A'},
		{'Â', 'B'},
		{'Å', 'E'},
		{'Ê', 'K'},
		{'Ì', 'M'},
		{'Í', 'H'},
		{'Î', 'O'},
		{'Ð', 'P'},
		{'Ñ', 'C'},
		{'Ò', 'T'},
		{'Õ', 'X'}
};


#define MAX_CGRAM_LOCATION 8

uint8_t current_cgram_location = 0;
uint8_t alphabet_cgram_locations[MAX_CGRAM_LOCATION];



/**
 * @brief  Turn display on and init it params
 * @note   We gonna make init steps according to datasheep page 46.
 *         There are 4 steps to turn 4-bits mode on,
 *         then we send initial params.
 * @param  hi2c    I2C struct to which display is connected
 * @param  address Display I2C 7-bit address
 * @param  lines   Number of lines of display
 * @param  columns Number of colums
 * @return         true if success
 */
bool lcdInit(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t lines, uint8_t columns) {

    uint8_t lcdData = LCD_BIT_5x8DOTS;

    lcdParams.hi2c      = hi2c;
    lcdParams.address   = address << 1;
    lcdParams.lines     = lines;
    lcdParams.columns   = columns;
    lcdParams.backlight = LCD_BIT_BACKIGHT_ON;

    lcdCommandBuffer[0] = LCD_BIT_E | (0x03 << 4);
    lcdCommandBuffer[1] = lcdCommandBuffer[0];
    lcdCommandBuffer[2] = (0x03 << 4);

    /* First 3 steps of init cycles. They are the same. */
    for (uint8_t i = 0; i < 3; ++i) {
        if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, (uint8_t*)lcdCommandBuffer, 3) != HAL_OK) {
            return false;
        }

        while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
            HAL_Delay(1);
        }

        if (i == 2) {
            // For the last cycle delay is less then 1 ms (100us by datasheet)
        	HAL_Delay(2);
        } else {
            // For first 2 cycles delay is less then 5ms (4100us by datasheet)
        	HAL_Delay(5);
        }
    }

    /* Lets turn to 4-bit at least */
    lcdCommandBuffer[0] = LCD_BIT_BACKIGHT_ON | LCD_BIT_E | (LCD_MODE_4BITS << 4);
    lcdCommandBuffer[1] = lcdCommandBuffer[0];
    lcdCommandBuffer[2] = LCD_BIT_BACKIGHT_ON | (LCD_MODE_4BITS << 4);

    if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, (uint8_t*)lcdCommandBuffer, 3) != HAL_OK) {
        return false;
    }

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
    	HAL_Delay(1);
    }

    /* Lets set display params */
    /* First of all lets set display size */
    lcdData |= LCD_MODE_4BITS;

    if (lcdParams.lines > 1) {
        lcdData |= LCD_BIT_2LINE;
    }

    lcdWriteByte((uint8_t) 0x00, &lcdData);  // TODO: Make 5x10 dots font usable for some 1-line display

    /* Now lets set display, cursor and blink all on */
    lcdDisplayOn();

    /* Set cursor moving to the right */
    lcdCursorDirToRight();

    /* Clear display and Set cursor at Home */
    lcdDisplayClear();
    lcdCursorHome();

    return true;
}

/**
 * @brief  Send command to display
 * @param  command  One of listed in LCDCommands enum
 * @param  action   LCD_PARAM_SET or LCD_PARAM_UNSET
 * @return          true if success
 */
bool lcdCommand(LCDCommands command, LCDParamsActions action) {
    uint8_t lcdData = 0x00;

    /* First of all lest store the command */
    switch (action) {
        case LCD_PARAM_SET:
            switch (command) {
                case LCD_DISPLAY:
                    lcdParams.modeBits |=  LCD_BIT_DISPLAY_ON;
                    break;

                case LCD_CURSOR:
                    lcdParams.modeBits |= LCD_BIT_CURSOR_ON;
                    break;

                case LCD_CURSOR_BLINK:
                    lcdParams.modeBits |= LCD_BIT_BLINK_ON;
                    break;

                case LCD_CLEAR:
                    lcdData = LCD_BIT_DISP_CLEAR;

                    if (lcdWriteByte((uint8_t)0x00, &lcdData) == false) {
                        return false;
                    } else {
                    	HAL_Delay(2);
                        return true;
                    }

                case LCD_CURSOR_HOME:
                    lcdData = LCD_BIT_CURSOR_HOME;

                    if (lcdWriteByte((uint8_t)0x00, &lcdData) == false) {
                        return false;
                    } else {
                    	HAL_Delay(2);
                        return true;
                    }

                case LCD_CURSOR_DIR_RIGHT:
                    lcdParams.entryBits |= LCD_BIT_CURSOR_DIR_RIGHT;
                    break;

                case LCD_CURSOR_DIR_LEFT:
                    lcdParams.entryBits |= LCD_BIT_CURSOR_DIR_LEFT;
                    break;

                case LCD_DISPLAY_SHIFT:
                    lcdParams.entryBits |= LCD_BIT_DISPLAY_SHIFT;
                    break;

                default:
                    return false;
            }

            break;

        case LCD_PARAM_UNSET:
            switch (command) {
                case LCD_DISPLAY:
                    lcdParams.modeBits &= ~LCD_BIT_DISPLAY_ON;
                    break;

                case LCD_CURSOR:
                    lcdParams.modeBits &= ~LCD_BIT_CURSOR_ON;
                    break;

                case LCD_CURSOR_BLINK:
                    lcdParams.modeBits &= ~LCD_BIT_BLINK_ON;
                    break;

                case LCD_CURSOR_DIR_RIGHT:
                    lcdParams.entryBits &= ~LCD_BIT_CURSOR_DIR_RIGHT;
                    break;

                case LCD_CURSOR_DIR_LEFT:
                    lcdParams.entryBits &= ~LCD_BIT_CURSOR_DIR_LEFT;
                    break;

                case LCD_DISPLAY_SHIFT:
                    lcdParams.entryBits &= ~LCD_BIT_DISPLAY_SHIFT;
                    break;

                default:
                    return false;
            }

            break;

        default:
            return false;
    }

    /* Now lets send the command */
    switch (command) {
        case LCD_DISPLAY:
        case LCD_CURSOR:
        case LCD_CURSOR_BLINK:
            lcdData = LCD_BIT_DISPLAY_CONTROL | lcdParams.modeBits;
            break;

        case LCD_CURSOR_DIR_RIGHT:
        case LCD_CURSOR_DIR_LEFT:
        case LCD_DISPLAY_SHIFT:
            lcdData = LCD_BIT_ENTRY_MODE | lcdParams.entryBits;
            break;

        default:
            break;
    }

    return lcdWriteByte((uint8_t)0x00, &lcdData);
}

/**
 * @brief  Turn display's Backlight On or Off
 * @param  command LCD_BIT_BACKIGHT_ON to turn display On
 *                 LCD_BIT_BACKIGHT_OFF (or 0x00) to turn display Off
 * @return         true if success
 */
bool lcdBacklight(uint8_t command) {
    lcdParams.backlight = command;

    if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, &lcdParams.backlight, 1) != HAL_OK) {
        return false;
    }

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
    	HAL_Delay(1);
    }

    return true;
}

/**
 * @brief  Set cursor position on the display
 * @param  column counting from 0
 * @param  line   counting from 0
 * @return        true if success
 */
bool lcdSetCursorPosition(uint8_t column, uint8_t line) {
    // We will setup offsets for 4 lines maximum
    static const uint8_t lineOffsets[4] = { 0x00, 0x40, 0x14, 0x54 };

    if ( line >= lcdParams.lines ) {
        line = lcdParams.lines - 1;
    }

    uint8_t lcdCommand = LCD_BIT_SETDDRAMADDR | (column + lineOffsets[line]);

    return lcdWriteByte(0x00, &lcdCommand);
}

void lcdSetCursor() {
	uint8_t line = currentCurson / lcdParams.columns;
	uint8_t cursor = line > 0 ? currentCurson++ - lcdParams.columns : currentCurson++;
	lcdSetCursorPosition(cursor, line);
}

/**
 * @brief  Print string from cursor position
 * @param  data   Pointer to string
 * @param  length Number of symbols to print
 * @return        true if success
 */
bool lcdPrintString(char* data, uint8_t start) {
	currentCurson = start;
    for (size_t i = 0; i < strlen(data); i++) {
    	if (lcdPrintChar(data[i]) == false) {
    		return false;
    	}
    }

    return true;
}

/**
 * @brief Loading custom Chars to one of the 8 cells in CGRAM
 * @note  You can create your custom chars according to
 *        documentation page 15.
 *        It consists of array of 8 bytes.
 *        Each byte is line of dots. Lower bits are dots.
 * @param  cell     Number of cell from 0 to 7 where to upload
 * @param  charMap  Pointer to Array of dots
 *                  Example: { 0x07, 0x09, 0x09, 0x09, 0x09, 0x1F, 0x11 }
 * @return          true if success
 */
bool loadCustomCharToLCD(uint8_t location, uint8_t* charMap) {

    uint8_t command = LCD_BIT_SETCGRAMADDR | (location << 3);
    if (lcdWriteByte((uint8_t) 0x00, &command) == false) {
        return false;
    }

    for (uint8_t i = 1; i < 9; ++i) {
        if (lcdWriteByte(LCD_BIT_RS, &charMap[i]) == false) {
            return false;
        }
    }

    return true;
}

uint8_t getPositionInCGRam(uint8_t alphabet) {
	for (int i = 0; i < MAX_CGRAM_LOCATION; i++) {
		if (alphabet_cgram_locations[i] == alphabet) {
			return i;
		}
	}
	return MAX_CGRAM_LOCATION + 1;
}

uint8_t localization(uint8_t data) {

	if (data < 168) { // check is English
		lcdSetCursor();
		return data;
	}

	for (int i = 0; i < RUSSIAN_ENG_SAME_COUNT; i++) {
		if (russian_eng_same[i][0] == data) {
			lcdSetCursor();
			return russian_eng_same[i][1];
		}
	}

	for (int i = 0; i < RUSSIAN_LOCALIZATION_COUNT; i++) {
		if (russian_alphabet[i][0] == data) {
			uint8_t location = getPositionInCGRam(russian_alphabet[i][0]);
			if (location < MAX_CGRAM_LOCATION + 1) {
				lcdSetCursor();
				return location;
			} else {
				current_cgram_location++;

				if (current_cgram_location == MAX_CGRAM_LOCATION) {
					current_cgram_location = 0;
				}

				alphabet_cgram_locations[current_cgram_location] = *russian_alphabet[i];

				loadCustomCharToLCD(current_cgram_location, russian_alphabet[i]);

				lcdSetCursor();
				return current_cgram_location;
			}
		}
	}

	return data;
}

/**
 * @brief  Print single char at cursor position
 * @param  data Symbol to print
 * @return      true if success
 */
bool lcdPrintChar(uint8_t data) {
	uint8_t local = localization(data);
    return lcdWriteByte(LCD_BIT_RS, &local);
}

/**
 * @brief  Local function to send data to display
 * @param  rsRwBits State of RS and R/W bits
 * @param  data     Pointer to byte to send
 * @return          true if success
 */
static bool lcdWriteByte(uint8_t rsRwBits, uint8_t* data) {

    /* Higher 4 bits*/
    lcdCommandBuffer[0] = rsRwBits | LCD_BIT_E | lcdParams.backlight | (*data & 0xF0);  // Send data and set strobe
    lcdCommandBuffer[1] = lcdCommandBuffer[0];                                          // Strobe turned on
    lcdCommandBuffer[2] = rsRwBits | lcdParams.backlight | (*data & 0xF0);              // Turning strobe off

    /* Lower 4 bits*/
    lcdCommandBuffer[3] = rsRwBits | LCD_BIT_E | lcdParams.backlight | ((*data << 4) & 0xF0);  // Send data and set strobe
    lcdCommandBuffer[4] = lcdCommandBuffer[3];                                                 // Strobe turned on
    lcdCommandBuffer[5] = rsRwBits | lcdParams.backlight | ((*data << 4) & 0xF0);              // Turning strobe off


    if (HAL_I2C_Master_Transmit_DMA(lcdParams.hi2c, lcdParams.address, (uint8_t*)lcdCommandBuffer, 6) != HAL_OK) {
        return false;
    }

    while (HAL_I2C_GetState(lcdParams.hi2c) != HAL_I2C_STATE_READY) {
    	HAL_Delay(1);
    }

    return true;
}
