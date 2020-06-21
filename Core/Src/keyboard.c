/*
 * keyboard.c
 *
 *  Created on: Jun 13, 2020
 *      Author: Bearice
 */
#include <stdio.h>

#include "main.h"
#include "usbd_hid_composite_if.h"

uint8_t scan_matrix[14][8] =
  {
    { KEY_1, KEY_TAB, KEY_Q, KEY_CAPSLOCK, KEY_A, KEY_LEFTSHIFT, KEY_Z, KEY_GRAVE },
    { KEY_3, KEY_W, KEY_E, KEY_S, KEY_D, KEY_X, KEY_C, KEY_2 },
    { KEY_5, KEY_R, KEY_T, KEY_F, KEY_G, KEY_V, KEY_B, KEY_4 },
    { KEY_7, KEY_Y, KEY_U, KEY_H, KEY_J, KEY_N, KEY_M, KEY_6 },
    { KEY_9, KEY_I, KEY_O, KEY_K, KEY_L, KEY_COMMA, KEY_DOT, KEY_8 },
    { KEY_MINUS, KEY_P, KEY_LEFTBRACE, KEY_SEMICOLON, KEY_APOSTROPHE, KEY_SLASH, KEY_RIGHTSHIFT, KEY_0 },
    { KEY_BACKSPACE, KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_ENTER, 0, KEY_RIGHTCTRL, 0, KEY_EQUAL },
    { KEY_HOME, KEY_DELETE, KEY_END, 0, KEY_UP, KEY_DOWN, KEY_LEFT, 0 },
    { 0, KEY_PAGEDOWN, 0, 0, 0, KEY_RIGHT, 0, KEY_PAGEUP },
    { 0, KEY_LEFTALT, KEY_LEFTCTRL, KEY_LEFTMETA, KEY_SPACE, KEY_RIGHTALT, 0, 0 },
    { KEY_F1, KEY_F2, KEY_F3, 0, 0, 0, 0, KEY_ESC },
    { KEY_F5, KEY_F6, 0, 0, 0, 0, 0, KEY_F4 },
    { KEY_F8, KEY_F9, 0, 0, 0, 0, 0, KEY_F7 },
    { KEY_F11, KEY_F12, 0, 0, 0, 0, 0, KEY_F10 } };

typedef struct
{
  uint8_t modifiers;
  uint8_t reserved;
  uint8_t keys[6];
} KeyReport;

static void
_sendReport (KeyReport *keys)
{
  uint8_t buf[8] = {
	keys->modifiers, keys->reserved,
	keys->keys[0], keys->keys[1], keys->keys[2], keys->keys[3], keys->keys[4],keys->keys[5]};

  printf("Report %02x%02x%02x%02x%02x%02x%02x%02x\r\n",
	 keys->modifiers, keys->reserved,
	 keys->keys[0], keys->keys[1], keys->keys[2], keys->keys[3], keys->keys[4],keys->keys[5]);
  HID_Composite_keyboard_sendReport (buf, 8);
  HAL_I2C_Master_Transmit(&hi2c1, 4<<1, buf, 4,10);
}

static KeyReport _keyReport =
  { 0 };

static size_t
_keyPress (uint8_t k)
{
  printf("keyPress(0x%02x)\r\n",k);
  if(!k)return 0;
  if(k>=KEY_LEFTCTRL && k <= KEY_RIGHTMETA){
      _keyReport.modifiers |= 1<<(k-KEY_LEFTCTRL);
  }
  uint8_t i;
  if (_keyReport.keys[0] != k && _keyReport.keys[1] != k && _keyReport.keys[2] != k && _keyReport.keys[3] != k
      && _keyReport.keys[4] != k && _keyReport.keys[5] != k)
    {

      for (i = 0; i < 6; i++)
	{
	  if (_keyReport.keys[i] == 0x00)
	    {
	      _keyReport.keys[i] = k;
	      break;
	    }
	}
      if (i == 6)
	{
	  return 0;
	}
    }
  _sendReport (&_keyReport);
  return 1;
}

static size_t
_keyRelease (uint8_t k)
{
  printf("keyRelease(0x%02x)\r\n",k);
  if(!k)return 0;
  if(k>=KEY_LEFTCTRL && k <= KEY_RIGHTMETA){
      _keyReport.modifiers &= ~(1<<(k-KEY_LEFTCTRL));
  }
  uint8_t i;
  for (i = 0; i < 6; i++)
    {
      if (0 != k && _keyReport.keys[i] == k)
	{
	  _keyReport.keys[i] = 0x00;
	}
    }

  _sendReport (&_keyReport);
  return 1;
}

static void
_keyReleaseAll (void)
{
  _keyReport.keys[0] = 0;
  _keyReport.keys[1] = 0;
  _keyReport.keys[2] = 0;
  _keyReport.keys[3] = 0;
  _keyReport.keys[4] = 0;
  _keyReport.keys[5] = 0;
  _keyReport.modifiers = 0;
  _sendReport (&_keyReport);
}

#define nelem(x) (sizeof(x)/sizeof(x[0]))
static uint16_t scan_pins[] =
  {
  GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11,
  GPIO_PIN_12, GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15,
  GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_8,
  GPIO_PIN_9, GPIO_PIN_15 };
#define scan_len nelem(scan_pins)

static uint8_t
scan_row (int row)
{
  GPIO_TypeDef *GPIOx = row == scan_len - 1 ? GPIOA : GPIOB;
  uint16_t pin = scan_pins[row];
  HAL_GPIO_WritePin (GPIOx, pin, GPIO_PIN_RESET);
  delay_us (10);
  uint8_t ret = ~(GPIOA->IDR & 0xff);
  HAL_GPIO_WritePin (GPIOx, pin, GPIO_PIN_SET);
  delay_us (10);
  return ret;
}

void
printBin (uint8_t var)
{
  for (uint8_t test = 0x80; test; test >>= 1)
    {
      putchar (var & test ? '1' : '0');
    }
  puts ("\r");
}

static uint8_t old_status[14] =
  { 0 };

void
keyboard_scan ()
{
  for (int i = 0; i < scan_len; i++)
    {
      uint8_t r = scan_row (i);
      uint8_t o = old_status[i];
      if (o != r)
	{
//	  printf ("%d=", i);
//	  printBin (r);
	  uint8_t x = o^r;
	  for (int k = 0; k < 8; k++)
	    {
	      if ((x >> k) & 1)
		{
		  uint8_t c = scan_matrix[i][k];
		  if ((r >> k) & 1)
		    {
		      _keyPress (c);
		    }
		  else
		    {
		      _keyRelease (c);
		    }
		}
	    }
	}
      old_status[i] = r;
    }
  //puts("===\r");
}

void
keyboard_update_led (uint8_t value)
{
  HAL_GPIO_WritePin (GPIOB, GPIO_PIN_2, !!value);
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_8, !value);
}

