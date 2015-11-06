
#ifndef __GADGET_CHIPS_H
#define __GADGET_CHIPS_H

#define gadget_is_amd5536udc(g)		(!strcmp("amd5536udc", (g)->name))
#define gadget_is_at91(g)		(!strcmp("at91_udc", (g)->name))
#define gadget_is_atmel_usba(g)		(!strcmp("atmel_usba_udc", (g)->name))
#define gadget_is_ci13xxx_msm(g)	(!strcmp("ci13xxx_msm", (g)->name))
#define gadget_is_ci13xxx_msm_hsic(g)	(!strcmp("ci13xxx_msm_hsic", (g)->name))
#define gadget_is_ci13xxx_pci(g)	(!strcmp("ci13xxx_pci", (g)->name))
#define gadget_is_dummy(g)		(!strcmp("dummy_udc", (g)->name))
#define gadget_is_dwc3(g)		(!strcmp("dwc3-gadget", (g)->name))
#define gadget_is_fsl_qe(g)		(!strcmp("fsl_qe_udc", (g)->name))
#define gadget_is_fsl_usb2(g)		(!strcmp("fsl-usb2-udc", (g)->name))
#define gadget_is_goku(g)		(!strcmp("goku_udc", (g)->name))
#define gadget_is_imx(g)		(!strcmp("imx_udc", (g)->name))
#define gadget_is_langwell(g)		(!strcmp("langwell_udc", (g)->name))
#define gadget_is_m66592(g)		(!strcmp("m66592_udc", (g)->name))
#define gadget_is_msm72k(g)		(!strcmp("msm72k_udc", (g)->name))
#define gadget_is_musbhdrc(g)		(!strcmp("musb-hdrc", (g)->name))
#define gadget_is_net2272(g)		(!strcmp("net2272", (g)->name))
#define gadget_is_net2280(g)		(!strcmp("net2280", (g)->name))
#define gadget_is_omap(g)		(!strcmp("omap_udc", (g)->name))
#define gadget_is_pch(g)		(!strcmp("pch_udc", (g)->name))
#define gadget_is_pxa(g)		(!strcmp("pxa25x_udc", (g)->name))
#define gadget_is_pxa27x(g)		(!strcmp("pxa27x_udc", (g)->name))
#define gadget_is_r8a66597(g)		(!strcmp("r8a66597_udc", (g)->name))
#define gadget_is_renesas_usbhs(g)	(!strcmp("renesas_usbhs_udc", (g)->name))
#define gadget_is_s3c2410(g)		(!strcmp("s3c2410_udc", (g)->name))
#define gadget_is_s3c_hsotg(g)		(!strcmp("s3c-hsotg", (g)->name))
#define gadget_is_s3c_hsudc(g)		(!strcmp("s3c-hsudc", (g)->name))

static inline int usb_gadget_controller_number(struct usb_gadget *gadget)
{
	if (gadget_is_net2280(gadget))
		return 0x01;
	else if (gadget_is_dummy(gadget))
		return 0x02;
	else if (gadget_is_pxa(gadget))
		return 0x03;
	else if (gadget_is_goku(gadget))
		return 0x06;
	else if (gadget_is_omap(gadget))
		return 0x08;
	else if (gadget_is_pxa27x(gadget))
		return 0x11;
	else if (gadget_is_s3c2410(gadget))
		return 0x12;
	else if (gadget_is_at91(gadget))
		return 0x13;
	else if (gadget_is_imx(gadget))
		return 0x14;
	else if (gadget_is_musbhdrc(gadget))
		return 0x16;
	else if (gadget_is_atmel_usba(gadget))
		return 0x18;
	else if (gadget_is_fsl_usb2(gadget))
		return 0x19;
	else if (gadget_is_amd5536udc(gadget))
		return 0x20;
	else if (gadget_is_m66592(gadget))
		return 0x21;
	else if (gadget_is_fsl_qe(gadget))
		return 0x22;
	else if (gadget_is_ci13xxx_pci(gadget))
		return 0x23;
	else if (gadget_is_langwell(gadget))
		return 0x24;
	else if (gadget_is_r8a66597(gadget))
		return 0x25;
	else if (gadget_is_s3c_hsotg(gadget))
		return 0x26;
	else if (gadget_is_pch(gadget))
		return 0x27;
	else if (gadget_is_ci13xxx_msm(gadget))
		return 0x28;
	else if (gadget_is_renesas_usbhs(gadget))
		return 0x29;
	else if (gadget_is_s3c_hsudc(gadget))
		return 0x30;
	else if (gadget_is_net2272(gadget))
		return 0x31;
	else if (gadget_is_dwc3(gadget))
		return 0x32;
	else if (gadget_is_msm72k(gadget))
		return 0x33;
	else if (gadget_is_ci13xxx_msm_hsic(gadget))
		return 0x34;

	return -ENOENT;
}


static inline bool gadget_supports_altsettings(struct usb_gadget *gadget)
{
	
	if (gadget_is_pxa(gadget))
		return false;

	
	if (gadget_is_pxa27x(gadget))
		return false;

	
	return true;
}

#endif 
