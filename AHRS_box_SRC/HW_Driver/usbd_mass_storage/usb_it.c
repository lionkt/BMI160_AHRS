

#include "usb_core.h"
#include "usbd_core.h"
#include "usb_conf.h"
#include "usb_hcd_int.h"

extern USB_OTG_CORE_HANDLE  USB_OTG_dev;
extern USB_OTG_CORE_HANDLE  USB_OTG_Core;

/* Private function prototypes -----------------------------------------------*/
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
	extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
	extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

/**
  * @brief  This function handles OTG_HS Handler.
  * @param  None
  * @retval None
  */
#ifdef USE_USB_OTG_HS
	void OTG_HS_IRQHandler(void)
	{
		USBH_OTG_ISR_Handler (&USB_OTG_Core);		
	}
#endif

#ifdef USE_USB_OTG_FS
	void OTG_FS_IRQHandler(void)
	{
		USBD_OTG_ISR_Handler (&USB_OTG_dev);
	}
#endif

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
	/**
	  * @brief  This function handles EP1_IN Handler.
	  * @param  None
	  * @retval None
	  */
	void OTG_HS_EP1_IN_IRQHandler(void)
	{
		USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
	}

	/**
	  * @brief  This function handles EP1_OUT Handler.
	  * @param  None
	  * @retval None
	  */
	void OTG_HS_EP1_OUT_IRQHandler(void)
	{
		USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
	}
#endif

/***************************** 安富莱电子 www.armfly.com (END OF FILE) *********************************/
