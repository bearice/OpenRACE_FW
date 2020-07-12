/**
 ******************************************************************************
 * @file    usbd_hid_composite.c
 * @author  MCD Application Team
 * @brief   This file provides the HID core functions.
 *
 * @verbatim
 *
 *          ===================================================================
 *                                HID Class  Description
 *          ===================================================================
 *           This module manages the HID class V1.11 following the "Device Class
 *Definition for Human Interface Devices (HID) Version 1.11 Jun 27, 2001". This
 *driver implements the following aspects of the specification:
 *             - The Boot Interface Subclass
 *             - The Mouse protocol
 *             - Usage Page : Generic Desktop
 *             - Usage : Joystick
 *             - Collection : Application
 *
 * @note     In HS mode and when the DMA is used, all variables and data
 *structures dealing with the DMA during the transaction process should be
 *32-bit aligned.
 *
 *
 *  @endverbatim
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2015 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      http://www.st.com/SLA0044
 *
 ******************************************************************************
 */

#include "usbd_hid_composite.h"
#include "keyboard.h"
#include "usbd_ctlreq.h"

// Class Callbacks
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
static uint8_t USBD_COMPOSITE_HID_Setup(USBD_HandleTypeDef *pdev,
                                        USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_MOUSE_Setup(USBD_HandleTypeDef *pdev,
                                    USBD_SetupReqTypedef *req);
static uint8_t USBD_HID_KEYBOARD_Setup(USBD_HandleTypeDef *pdev,
                                       USBD_SetupReqTypedef *req);
static uint8_t *USBD_HID_GetCfgDesc(uint16_t *length);
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length);
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev);

USBD_ClassTypeDef USBD_COMPOSITE_HID = {
    USBD_HID_Init,
    USBD_HID_DeInit,
    USBD_COMPOSITE_HID_Setup,
    NULL,                 /*EP0_TxSent*/
    USBD_HID_EP0_RxReady, /*EP0_RxReady*/
    USBD_HID_DataIn,      /*DataIn*/
    USBD_HID_DataOut,     /*DataOut*/
    NULL,                 /*SOF */
    NULL,                 // IsoINIncomplete
    NULL,                 // IsoOUTIncomplete
    USBD_HID_GetCfgDesc,
    USBD_HID_GetCfgDesc,
    USBD_HID_GetCfgDesc,
    USBD_HID_GetDeviceQualifierDesc,
};

/* USB HID device FS Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_CfgDesc[USB_COMPOSITE_HID_CONFIG_DESC_SIZ] __ALIGN_END = {
    0x09,                                      /* bLength: Configuration Descriptor size */
    USB_DESC_TYPE_CONFIGURATION,               /* bDescriptorType: Configuration */
    LOBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ), /* wTotalLength: Bytes returned */
    HIBYTE(USB_COMPOSITE_HID_CONFIG_DESC_SIZ), /* wTotalLength: Bytes returned */
    0x02,                                      /* bNumInterfaces: 2 interface */
    0x01,                                      /* bConfigurationValue: Configuration value */
    0x00,                                      /* iConfiguration: Index of string descriptor describing the configuration */
    0xC0,                                      /* bmAttributes: bus powered and no Support Remote Wake-up */
    0x32,                                      /* MaxPower 100 mA: this current is used for detecting Vbus*/

    /************** Descriptor of Mouse interface ****************/
    /* 09 */
    0x09,                    /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE, /*bDescriptorType: Interface descriptor type*/
    HID_MOUSE_INTERFACE,     /*bInterfaceNumber: Number of Interface*/
    0x00,                    /*bAlternateSetting: Alternate setting*/
    0x01,                    /*bNumEndpoints*/
    0x03,                    /*bInterfaceClass: HID*/
    0x01,                    /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x02,                    /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0x00,                    /*iInterface: Index of string descriptor*/

    /******************** Descriptor of Mouse HID ********************/
    /* 18 */
    0x09,                       /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE,        /*bDescriptorType: HID*/
    0x11,                       /*bcdHID: HID Class Spec release number*/
    0x01, 0x00,                 /*bCountryCode: Hardware target country*/
    0x01,                       /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,                       /*bDescriptorType*/
    HID_MOUSE_REPORT_DESC_SIZE, /*wItemLength: Total length of Report descriptor*/
    0x00,

    /******************** Descriptor of Mouse endpoint ********************/
    /* 27 */
    0x07,                   /*bLength: Endpoint Descriptor size*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
    HID_MOUSE_EPIN_ADDR,    /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,                   /*bmAttributes: Interrupt endpoint*/
    HID_MOUSE_EPIN_SIZE,    /*wMaxPacketSize: 4 Byte max */
    0x00, HID_FS_BINTERVAL, /*bInterval: Polling Interval*/

    /************** Descriptor of HID Keyboard interface ****************/
    /* 34 */
    0x09,                    /*bLength: Interface Descriptor size*/
    USB_DESC_TYPE_INTERFACE, /*bDescriptorType: Interface descriptor type*/
    HID_KEYBOARD_INTERFACE,  /*bInterfaceNumber: Number of Interface*/
    0x00,                    /*bAlternateSetting: Alternate setting*/
#ifdef HID_USE_OUTPUT_EP
    0x02, /*bNumEndpoints*/
#else
    0x01, /*bNumEndpoints*/
#endif
    0x03, /*bInterfaceClass: HID*/
    0x01, /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
    0x01, /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
    0x00, /*iInterface: Index of string descriptor*/

    /******************** HID Descriptor ********************/
    /* 43 */
    0x09,                          /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE,           /*bDescriptorType: HID*/
    0x11,                          /*bcdHID: HID Class Spec release number*/
    0x01, 0x00,                    /*bCountryCode: Hardware target country*/
    0x01,                          /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,                          /*bDescriptorType*/
    HID_KEYBOARD_REPORT_DESC_SIZE, /*wItemLength: Total length of Report descriptor*/
    0x00,

    /******************** Descriptor of Keyboard endpoint ********************/
    /* 52 */
    0x07,                   /*bLength: Endpoint Descriptor size*/
    USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/
    HID_KEYBOARD_EPIN_ADDR, /*bEndpointAddress: Endpoint Address (IN)*/
    0x03,                   /*bmAttributes: Interrupt endpoint*/
    HID_KEYBOARD_EPIN_SIZE, /*wMaxPacketSize: 8 Byte max */
    0x00, HID_FS_BINTERVAL, /*bInterval: Polling Interval*/

#ifdef HID_USE_OUTPUT_EP
    /******************** Descriptor of Keyboard endpoint ********************/
    /* 59 */
    0x07,                    /*bLength: Endpoint Descriptor size*/
    USB_DESC_TYPE_ENDPOINT,  /*bDescriptorType:*/
    HID_KEYBOARD_EPOUT_ADDR, /*bEndpointAddress: Endpoint Address (OUT)*/
    0x03,                    /*bmAttributes: Interrupt endpoint*/
    HID_KEYBOARD_EPOUT_SIZE, /*wMaxPacketSize: 1 Byte max */
    0x00, HID_FS_BINTERVAL,  /*bInterval: Polling Interval (10 ms)*/
#endif
    /* 66 */
};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_MOUSE_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END = {
    0x09,                       /*bLength: HID Descriptor size*/
    HID_DESCRIPTOR_TYPE,        /*bDescriptorType: HID*/
    0x11,                       /*bcdHID: HID Class Spec release number*/
    0x01,                       /*bcdHID: HID Class Spec release number*/
    0x00,                       /*bCountryCode: Hardware target country*/
    0x01,                       /*bNumDescriptors: Number of HID class descriptors to follow*/
    0x22,                       /*bDescriptorType*/
    HID_MOUSE_REPORT_DESC_SIZE, /*wItemLength: Total length of Report descriptor*/
    0x00,
};

/* USB HID device Configuration Descriptor */
__ALIGN_BEGIN static uint8_t USBD_KEYBOARD_HID_Desc[USB_HID_DESC_SIZ] __ALIGN_END = {
    0x09,                          // bLength: HID Descriptor size
    HID_DESCRIPTOR_TYPE,           // bDescriptorType: HID
    0x11,                          // bcdHID: HID Class Spec release number
    0x01,                          // bcdHID: HID Class Spec release number
    0x00,                          // bCountryCode: Hardware target country
    0x01,                          // bNumDescriptors: Number of HID class descriptors to follow
    0x22,                          // bDescriptorType
    HID_KEYBOARD_REPORT_DESC_SIZE, // wItemLength: Total length of Report descriptor
    0x00,                          // bReserved
                                   // 10 bytes
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_HID_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END = {
    USB_LEN_DEV_QUALIFIER_DESC,     // bLength
    USB_DESC_TYPE_DEVICE_QUALIFIER, // bDescriptorType (Device Qualifier)
    0x00,                           // bcdUSB 2.00
    0x02,                           // bcdUSB 2.00
    0x00,                           // bDeviceClass (Use class information in the Interface Descriptors)
    0x00,                           // bDeviceSubClass
    0x00,                           // bDeviceProtocol
    0x40,                           // bMaxPacketSize0 64
    0x01,                           // bNumConfigurations 1
    0x00,                           // bReserved
                                    // 10 bytes
};

__ALIGN_BEGIN static uint8_t HID_MOUSE_ReportDesc[HID_MOUSE_REPORT_DESC_SIZE] __ALIGN_END = {
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x09, 0x02, // Usage (Mouse)
    0xA1, 0x01, // Collection (Application)

    0x09, 0x01, //   Usage (Pointer)
    0xA1, 0x00, //   Collection (Physical)

    0x05, 0x09, //     Usage Page (Button)
    0x19, 0x01, //     Usage Minimum (0x01)
    0x29, 0x03, //     Usage Maximum (0x03)
    0x15, 0x00, //     Logical Minimum (0)
    0x25, 0x01, //     Logical Maximum (1)
    0x95, 0x03, //     Report Count (3)
    0x75, 0x01, //     Report Size (1)
    0x81, 0x02, //     Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x95, 0x01, //     Report Count (1)
    0x75, 0x05, //     Report Size (5)
    0x81, 0x01, //     Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x05, 0x01, //     Usage Page (Generic Desktop Ctrls)
    0x09, 0x30, //     Usage (X)
    0x09, 0x31, //     Usage (Y)
    0x09, 0x38, //     Usage (Wheel)
    0x15, 0x81, //     Logical Minimum (-127)
    0x25, 0x7F, //     Logical Maximum (127)
    0x75, 0x08, //     Report Size (8)
    0x95, 0x03, //     Report Count (3)
    0x81, 0x06, //     Input (Data,Var,Rel,No Wrap,Linear,Preferred State,No Null Position)

    0xC0, //   End Collection

    0x09, 0x3C, //   Usage (Motion Wakeup)
    0x05, 0xFF, //   Usage Page (Reserved 0xFF)
    0x09, 0x01, //   Usage (0x01)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x02, //   Report Count (2)
    0xB1, 0x22, //   Feature (Data,Var,Abs,No Wrap,Linear,No Preferred State,No Null Position,Non-volatile)

    0x75, 0x06, //   Report Size (6)
    0x95, 0x01, //   Report Count (1)
    0xB1, 0x01, //   Feature (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0xC0, // End Collection

    // 74 bytes
};

__ALIGN_BEGIN static uint8_t HID_KEYBOARD_ReportDesc[HID_KEYBOARD_REPORT_DESC_SIZE] __ALIGN_END = {
    0x05, 0x01, // Usage Page (Generic Desktop Ctrls)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)

    0x05, 0x07, //   Usage Page (Kbrd/Keypad)
    0x19, 0xE0, //   Usage Minimum (0xE0)
    0x29, 0xE7, //   Usage Maximum (0xE7)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x08, //   Report Count (8)
    0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x95, 0x01, //   Report Count (1)
    0x75, 0x08, //   Report Size (8)
    0x81, 0x03, //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x25, 0x01, //   Logical Maximum (1)
    0x95, 0x05, //   Report Count (5)
    0x75, 0x01, //   Report Size (1)
    0x05, 0x08, //   Usage Page (LEDs)
    0x19, 0x01, //   Usage Minimum (Num Lock)
    0x29, 0x05, //   Usage Maximum (Kana)
    0x91, 0x02, //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0x95, 0x01, //   Report Count (1)
    0x75, 0x03, //   Report Size (3)
    0x91, 0x03, //   Output (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0x95, 0x06, //   Report Count (6)
    0x75, 0x08, //   Report Size (8)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x65, //   Logical Maximum (101)
    0x05, 0x07, //   Usage Page (Kbrd/Keypad)
    0x19, 0x00, //   Usage Minimum (0x00)
    0x29, 0x65, //   Usage Maximum (0x65)
    0x81, 0x00, //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0xC0, // End Collection

    // 65 bytes
};

/**
 * @brief  USBD_HID_Init
 *         Initialize the HID interface
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_HID_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  UNUSED(cfgidx);
  uint8_t ret = USBD_OK;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev, HID_MOUSE_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_MOUSE_EPIN_SIZE);
  pdev->ep_in[HID_MOUSE_EPIN_ADDR & 0xFU].is_used = 1U;

  /* Open EP IN */
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EPIN_ADDR, USBD_EP_TYPE_INTR, HID_KEYBOARD_EPIN_SIZE);
  pdev->ep_in[HID_KEYBOARD_EPIN_ADDR & 0xFU].is_used = 1U;

#ifdef HID_USE_OUTPUT_EP
  /* Open EP OUT */
  USBD_LL_OpenEP(pdev, HID_KEYBOARD_EPOUT_ADDR, USBD_EP_TYPE_INTR, HID_KEYBOARD_EPOUT_SIZE);
  pdev->ep_out[HID_KEYBOARD_EPOUT_ADDR & 0xFU].is_used = 1;
#endif

  USBD_HID_HandleTypeDef *pCData = pdev->pClassData =
      USBD_malloc(sizeof(USBD_HID_HandleTypeDef));

  if (pCData == NULL) {
    ret = USBD_FAIL;
  } else {
    pCData->Mousestate = HID_IDLE;
    pCData->Keyboardstate = HID_IDLE;

#ifdef HID_USE_OUTPUT_EP
    // set EP_OUT 1 prepared to received the data
    USBD_LL_PrepareReceive(pdev, HID_KEYBOARD_EPOUT_ADDR, &pCData->KeyboardLed, HID_KEYBOARD_EPOUT_SIZE);
#endif
  }

  return ret;
}

/**
 * @brief  USBD_HID_Init
 *         DeInitialize the HID layer
 * @param  pdev: device instance
 * @param  cfgidx: Configuration index
 * @retval status
 */
static uint8_t USBD_HID_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
  UNUSED(cfgidx);
  /* Close HID EPs */
  USBD_LL_CloseEP(pdev, HID_MOUSE_EPIN_ADDR);
  pdev->ep_in[HID_MOUSE_EPIN_ADDR & 0xFU].is_used = 0U;

  USBD_LL_CloseEP(pdev, HID_KEYBOARD_EPIN_ADDR);
  pdev->ep_in[HID_KEYBOARD_EPIN_ADDR & 0xFU].is_used = 0U;

#ifdef HID_USE_OUTPUT_EP
  USBD_LL_CloseEP(pdev, HID_KEYBOARD_EPOUT_ADDR);
  pdev->ep_out[HID_KEYBOARD_EPOUT_ADDR & 0xFU].is_used = 0;
#endif

  /* Free allocated memory */
  if (pdev->pClassData != NULL) {
    USBD_free(pdev->pClassData);
    pdev->pClassData = NULL;
  }

  return USBD_OK;
}

/**
 * @brief  USBD_COMPOSITE_HID_Setup
 *         Handle the HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_COMPOSITE_HID_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  /* Check which interface is targetted by this request */
  if ((req->wIndex & 0x00FF) == HID_KEYBOARD_INTERFACE) {
    return USBD_HID_KEYBOARD_Setup(pdev, req);
  } else {
    return USBD_HID_MOUSE_Setup(pdev, req);
  }
}

/**
 * @brief  USBD_HID_MOUSE_Setup
 *         Handle the HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_HID_MOUSE_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
  case USB_REQ_TYPE_CLASS:
    switch (req->bRequest) {

    case HID_REQ_SET_PROTOCOL:
      hhid->Protocol = (uint8_t)(req->wValue);
      break;

    case HID_REQ_GET_PROTOCOL:
      USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
      break;

    case HID_REQ_SET_IDLE:
      hhid->IdleState = (uint8_t)(req->wValue >> 8);
      break;

    case HID_REQ_GET_IDLE:
      USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest) {
    case USB_REQ_GET_STATUS:
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;
    case USB_REQ_GET_DESCRIPTOR:
      if (req->wValue >> 8 == HID_REPORT_DESC) {
        len = MIN(HID_MOUSE_REPORT_DESC_SIZE, req->wLength);
        pbuf = HID_MOUSE_ReportDesc;
      } else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE) {
        pbuf = USBD_MOUSE_HID_Desc;
        len = MIN(USB_HID_DESC_SIZ, req->wLength);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
      }
      USBD_CtlSendData(pdev, pbuf, len);
      break;

    case USB_REQ_GET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        hhid->AltSetting = (uint8_t)(req->wValue);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return ret;
}

enum { HID_INPUT = 1,
       HID_OUTPUT = 2,
       HID_FEATURE = 3 };

/**
 * @brief  USBD_HID_KEYBOARD_Setup
 *         Handle the HID specific requests
 * @param  pdev: instance
 * @param  req: usb requests
 * @retval status
 */
static uint8_t USBD_HID_KEYBOARD_Setup(USBD_HandleTypeDef *pdev,
                                       USBD_SetupReqTypedef *req) {
  USBD_HID_HandleTypeDef *hhid = pdev->pClassData;
  uint16_t len = 0U;
  uint8_t *pbuf = NULL;
  uint16_t status_info = 0U;
  USBD_StatusTypeDef ret = USBD_OK;

  switch (req->bmRequest & USB_REQ_TYPE_MASK) {
  case USB_REQ_TYPE_CLASS:
    switch (req->bRequest) {

    case HID_REQ_SET_PROTOCOL:
      // puts("HID_REQ_SET_PROTOCOL\r");
      hhid->Protocol = (uint8_t)(req->wValue);
      break;

    case HID_REQ_GET_PROTOCOL:
      // puts("HID_REQ_GET_PROTOCOL\r");
      USBD_CtlSendData(pdev, (uint8_t *)&hhid->Protocol, 1U);
      break;

    case HID_REQ_SET_IDLE:
      // puts("HID_REQ_SET_IDLE\r");
      hhid->IdleState = (uint8_t)(req->wValue >> 8);
      break;

    case HID_REQ_GET_IDLE:
      // puts("HID_REQ_GET_IDLE\r");
      USBD_CtlSendData(pdev, (uint8_t *)&hhid->IdleState, 1U);
      break;

    case HID_REQ_GET_REPORT:
      // puts("HID_REQ_GET_REPORT\r");
      if ((req->wValue >> 8) == HID_OUTPUT) {
        USBD_CtlSendData(pdev, &hhid->KeyboardLed, 1);
      }
      break;

    case HID_REQ_SET_REPORT:
      // puts("HID_REQ_SET_REPORT\r");
      if ((req->wValue >> 8) == HID_OUTPUT) {
        USBD_CtlPrepareRx(pdev, &hhid->KeyboardLed, 1);
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest) {
    case USB_REQ_GET_STATUS:
      // puts("USB_REQ_GET_STATUS\r");
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        USBD_CtlSendData(pdev, (uint8_t *)(void *)&status_info, 2U);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;
    case USB_REQ_GET_DESCRIPTOR:
      // puts("USB_REQ_GET_DESCRIPTOR\r");
      if (req->wValue >> 8 == HID_REPORT_DESC) {
        len = MIN(HID_KEYBOARD_REPORT_DESC_SIZE, req->wLength);
        pbuf = HID_KEYBOARD_ReportDesc;
      } else if (req->wValue >> 8 == HID_DESCRIPTOR_TYPE) {
        pbuf = USBD_KEYBOARD_HID_Desc;
        len = MIN(USB_HID_DESC_SIZ, req->wLength);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
        break;
      }
      USBD_CtlSendData(pdev, pbuf, len);

      break;

    case USB_REQ_GET_INTERFACE:
      // puts("USB_REQ_GET_INTERFACE\r");
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        USBD_CtlSendData(pdev, (uint8_t *)&hhid->AltSetting, 1U);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    case USB_REQ_SET_INTERFACE:
      // puts("USB_REQ_SET_INTERFACE\r");
      if (pdev->dev_state == USBD_STATE_CONFIGURED) {
        hhid->AltSetting = (uint8_t)(req->wValue);
      } else {
        USBD_CtlError(pdev, req);
        ret = USBD_FAIL;
      }
      break;

    default:
      USBD_CtlError(pdev, req);
      ret = USBD_FAIL;
      break;
    }
    break;

  default:
    USBD_CtlError(pdev, req);
    ret = USBD_FAIL;
    break;
  }

  return ret;
}

/**
 * @brief  USBD_HID_SendReport
 *         Send HID Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t USBD_HID_MOUSE_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED) {
    if (hhid->Mousestate == HID_IDLE) {
      hhid->Mousestate = HID_BUSY;
      USBD_LL_Transmit(pdev, HID_MOUSE_EPIN_ADDR, report, len);
    }
  }
  return USBD_OK;
}

/**
 * @brief  USBD_HID_SendReport
 *         Send HID Report
 * @param  pdev: device instance
 * @param  buff: pointer to report
 * @retval status
 */
uint8_t USBD_HID_KEYBOARD_SendReport(USBD_HandleTypeDef *pdev, uint8_t *report, uint16_t len) {
  USBD_HID_HandleTypeDef *hhid = (USBD_HID_HandleTypeDef *)pdev->pClassData;

  if (pdev->dev_state == USBD_STATE_CONFIGURED) {
    if (hhid->Keyboardstate == HID_IDLE) {
      hhid->Keyboardstate = HID_BUSY;
      USBD_LL_Transmit(pdev, HID_KEYBOARD_EPIN_ADDR, report, len);
    }
  }
  return USBD_OK;
}

/**
 * @brief  USBD_HID_GetPollingInterval
 *         return polling interval from endpoint descriptor
 * @param  pdev: device instance
 * @retval polling interval
 */
uint32_t USBD_HID_GetPollingInterval(USBD_HandleTypeDef *pdev) {
  uint32_t polling_interval = 0U;

  /* HIGH-speed endpoints */
  if (pdev->dev_speed == USBD_SPEED_HIGH) {
    /* Sets the data transfer polling interval for high speed transfers.
    Values between 1..16 are allowed. Values correspond to interval
    of 2 ^ (bInterval-1). This option (8 ms, corresponds to HID_HS_BINTERVAL
    */
    polling_interval = (((1U << (HID_HS_BINTERVAL - 1U))) / 8U);
  } else { /* LOW and FULL-speed endpoints */
    /* Sets the data transfer polling interval for low and full
   speed transfers */
    polling_interval = HID_FS_BINTERVAL;
  }

  return ((uint32_t)(polling_interval));
}

/**
 * @brief  USBD_HID_GetCfgDesc
 *         return configuration descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t *USBD_HID_GetCfgDesc(uint16_t *length) {
  *length = sizeof(USBD_HID_CfgDesc);
  return USBD_HID_CfgDesc;
}

/**
 * @brief  USBD_HID_DataIn
 *         handle data IN Stage
 * @param  pdev: device instance
 * @param  epnum: endpoint index
 * @retval status
 */
static uint8_t USBD_HID_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {

  /* Ensure that the FIFO is empty before a new transfer, this condition could
   be caused by  a new transfer before the end of the previous transfer */
  USBD_HID_HandleTypeDef *pCData = pdev->pClassData;

  if (epnum == (HID_KEYBOARD_EPIN_ADDR & 0x7F)) {
    pCData->Keyboardstate = HID_IDLE;
  } else if (epnum == (HID_MOUSE_EPIN_ADDR & 0x7F)) {
    pCData->Mousestate = HID_IDLE;
  }
  //	printf("USBD_HID_DataIn(%08x,%d)\r\n", pdev, epnum);
  return USBD_OK;
}

static uint8_t USBD_HID_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
  USBD_HID_HandleTypeDef *pCData = pdev->pClassData;

  USBD_LL_PrepareReceive(pdev, HID_KEYBOARD_EPOUT_ADDR, &pCData->KeyboardLed,
                         HID_KEYBOARD_EPOUT_SIZE);

  printf("USBD_HID_DataOut(%d)=0x%02x\r\n", epnum, pCData->KeyboardLed);
  keyboard_update_led(pCData->KeyboardLed);

  return USBD_OK;
}

static uint8_t USBD_HID_EP0_RxReady(USBD_HandleTypeDef *pdev) {
  // puts("EP0_RxReady\r");
  USBD_HID_HandleTypeDef *pCData = pdev->pClassData;
  keyboard_update_led(pCData->KeyboardLed);

  return USBD_OK;
}

/**
 * @brief  DeviceQualifierDescriptor
 *         return Device Qualifier descriptor
 * @param  length : pointer data length
 * @retval pointer to descriptor buffer
 */
static uint8_t *USBD_HID_GetDeviceQualifierDesc(uint16_t *length) {
  *length = sizeof(USBD_HID_DeviceQualifierDesc);
  return USBD_HID_DeviceQualifierDesc;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
