/*
 * Misc utility routines for accessing chip-specific features
 * of the SiliconBackplane-based Broadcom chips.
 *
 * Copyright (C) 1999-2012, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 * $Id: siutils.c 325758 2012-04-04 19:12:18Z $
 */

#include <bcm_cfg.h>
#include <typedefs.h>
#include <bcmdefs.h>
#include <osl.h>
#include <bcmutils.h>
#include <siutils.h>
#include <bcmdevs.h>
#include <hndsoc.h>
#include <sbchipc.h>
#include <pcicfg.h>
#include <sbpcmcia.h>
#include <sbsocram.h>
#include <bcmsdh.h>
#include <sdio.h>
#include <sbsdio.h>
#include <sbhnddma.h>
#include <sbsdpcmdev.h>
#include <bcmsdpcm.h>
#include <hndpmu.h>

#include "siutils_priv.h"

int bcm_chip_is_4330b1 = 0;
int bcm_chip_is_4330 = 0;
static si_info_t *si_doattach(si_info_t *sii, uint devid, osl_t *osh, void *regs,
                              uint bustype, void *sdh, char **vars, uint *varsz);
static bool si_buscore_prep(si_info_t *sii, uint bustype, uint devid, void *sdh);
static bool si_buscore_setup(si_info_t *sii, chipcregs_t *cc, uint bustype, uint32 savewin,
	uint *origidx, void *regs);



static uint32 si_gpioreservation = 0;


int do_4360_pcie2_war = 0;

extern si_t	*local_sih;
si_t *
si_attach(uint devid, osl_t *osh, void *regs,
                       uint bustype, void *sdh, char **vars, uint *varsz)
{
	si_info_t *sii;

	
	if ((sii = MALLOC(osh, sizeof (si_info_t))) == NULL) {
		SI_ERROR(("si_attach: malloc failed! malloced %d bytes\n", MALLOCED(osh)));
		return (NULL);
	}

	local_sih = &sii->pub;
	if (si_doattach(sii, devid, osh, regs, bustype, sdh, vars, varsz) == NULL) {
		MFREE(osh, sii, sizeof(si_info_t));
		local_sih = NULL;
		return (NULL);
	}
	sii->vars = vars ? *vars : NULL;
	sii->varsz = varsz ? *varsz : 0;

	return (si_t *)sii;
}

static si_info_t ksii;

static uint32	wd_msticks;		

si_t *
si_kattach(osl_t *osh)
{
	static bool ksii_attached = FALSE;

	if (!ksii_attached) {
		void *regs = NULL;
		regs = REG_MAP(SI_ENUM_BASE, SI_CORE_SIZE);
#ifdef HTC_KlocWork
		if(!osh){
			SI_ERROR(("[HTCKW] si_kattach: osh is NULL\n"));
			return NULL;
		}
#endif
		if (si_doattach(&ksii, BCM4710_DEVICE_ID, osh, regs,
		                SI_BUS, NULL,
		                osh != SI_OSH ? &ksii.vars : NULL,
		                osh != SI_OSH ? &ksii.varsz : NULL) == NULL) {
			SI_ERROR(("si_kattach: si_doattach failed\n"));
			REG_UNMAP(regs);
			return NULL;
		}
		REG_UNMAP(regs);

		
		if (PMUCTL_ENAB(&ksii.pub)) {
				
				wd_msticks = 32;
		} else {
			wd_msticks = ALP_CLOCK / 1000;
		}

		ksii_attached = TRUE;
		SI_MSG(("si_kattach done. ccrev = %d, wd_msticks = %d\n",
		        ksii.pub.ccrev, wd_msticks));
	}

	return &ksii.pub;
}


static bool
si_buscore_prep(si_info_t *sii, uint bustype, uint devid, void *sdh)
{
	
	if (BUSTYPE(bustype) == PCMCIA_BUS)
		sii->memseg = TRUE;


	if (BUSTYPE(bustype) == SDIO_BUS) {
		int err;
		uint8 clkset;

		
		clkset = SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_ALP_AVAIL_REQ;
		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR, clkset, &err);
		if (!err) {
			uint8 clkval;

			
			clkval = bcmsdh_cfg_read(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR, NULL);
			if ((clkval & ~SBSDIO_AVBITS) == clkset) {
				SPINWAIT(((clkval = bcmsdh_cfg_read(sdh, SDIO_FUNC_1,
					SBSDIO_FUNC1_CHIPCLKCSR, NULL)), !SBSDIO_ALPAV(clkval)),
					PMU_MAX_TRANSITION_DLY);
				if (!SBSDIO_ALPAV(clkval)) {
					SI_ERROR(("timeout on ALPAV wait, clkval 0x%02x\n",
						clkval));
					return FALSE;
				}
				clkset = SBSDIO_FORCE_HW_CLKREQ_OFF | SBSDIO_FORCE_ALP;
				bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_CHIPCLKCSR,
					clkset, &err);
				OSL_DELAY(65);
			}
		}

		
		bcmsdh_cfg_write(sdh, SDIO_FUNC_1, SBSDIO_FUNC1_SDIOPULLUP, 0, NULL);
	}


	return TRUE;
}

static bool
si_buscore_setup(si_info_t *sii, chipcregs_t *cc, uint bustype, uint32 savewin,
	uint *origidx, void *regs)
{
	bool pci, pcie, pcie_gen2 = FALSE;
	uint i;
	uint pciidx, pcieidx, pcirev, pcierev;

	cc = si_setcoreidx(&sii->pub, SI_CC_IDX);
	ASSERT((uintptr)cc);
#ifdef HTC_KlocWork
    if(cc == NULL){
        SI_ERROR(("[HTCKW] si_buscore_setup: cc is NULL\n"));
        return FALSE;
    }
#endif
	
	sii->pub.ccrev = (int)si_corerev(&sii->pub);

	
	if (sii->pub.ccrev >= 11)
		sii->pub.chipst = R_REG(sii->osh, &cc->chipstatus);

	
	sii->pub.cccaps = R_REG(sii->osh, &cc->capabilities);
	

	if (sii->pub.ccrev >= 35)
		sii->pub.cccaps_ext = R_REG(sii->osh, &cc->capabilities_ext);

	
	if (sii->pub.cccaps & CC_CAP_PMU) {
		sii->pub.pmucaps = R_REG(sii->osh, &cc->pmucapabilities);
		sii->pub.pmurev = sii->pub.pmucaps & PCAP_REV_MASK;
	}

	SI_MSG(("Chipc: rev %d, caps 0x%x, chipst 0x%x pmurev %d, pmucaps 0x%x\n",
		sii->pub.ccrev, sii->pub.cccaps, sii->pub.chipst, sii->pub.pmurev,
		sii->pub.pmucaps));

	
	sii->pub.buscoretype = NODEV_CORE_ID;
	sii->pub.buscorerev = (uint)NOREV;
	sii->pub.buscoreidx = BADIDX;

	pci = pcie = FALSE;
	pcirev = pcierev = (uint)NOREV;
	pciidx = pcieidx = BADIDX;

	for (i = 0; i < sii->numcores; i++) {
		uint cid, crev;

		si_setcoreidx(&sii->pub, i);
		cid = si_coreid(&sii->pub);
		crev = si_corerev(&sii->pub);

		
		SI_VMSG(("CORE[%d]: id 0x%x rev %d base 0x%x regs 0x%p\n",
		        i, cid, crev, sii->coresba[i], sii->regs[i]));

		if (BUSTYPE(bustype) == PCI_BUS) {
			if (cid == PCI_CORE_ID) {
				pciidx = i;
				pcirev = crev;
				pci = TRUE;
			} else if ((cid == PCIE_CORE_ID) || (cid == PCIE2_CORE_ID)) {
				pcieidx = i;
				pcierev = crev;
				pcie = TRUE;
				if (cid == PCIE2_CORE_ID)
					pcie_gen2 = TRUE;
			}
		} else if ((BUSTYPE(bustype) == PCMCIA_BUS) &&
		           (cid == PCMCIA_CORE_ID)) {
			sii->pub.buscorerev = crev;
			sii->pub.buscoretype = cid;
			sii->pub.buscoreidx = i;
		}
		else if (((BUSTYPE(bustype) == SDIO_BUS) ||
		          (BUSTYPE(bustype) == SPI_BUS)) &&
		         ((cid == PCMCIA_CORE_ID) ||
		          (cid == SDIOD_CORE_ID))) {
			sii->pub.buscorerev = crev;
			sii->pub.buscoretype = cid;
			sii->pub.buscoreidx = i;
		}

		
		if ((savewin && (savewin == sii->coresba[i])) ||
		    (regs == sii->regs[i]))
			*origidx = i;
	}

	if (pci) {
		sii->pub.buscoretype = PCI_CORE_ID;
		sii->pub.buscorerev = pcirev;
		sii->pub.buscoreidx = pciidx;
	} else if (pcie) {
		if (pcie_gen2)
			sii->pub.buscoretype = PCIE2_CORE_ID;
		else
			sii->pub.buscoretype = PCIE_CORE_ID;
		sii->pub.buscorerev = pcierev;
		sii->pub.buscoreidx = pcieidx;
	}

	SI_VMSG(("Buscore id/type/rev %d/0x%x/%d\n", sii->pub.buscoreidx, sii->pub.buscoretype,
	         sii->pub.buscorerev));

	if (BUSTYPE(sii->pub.bustype) == SI_BUS && (CHIPID(sii->pub.chip) == BCM4712_CHIP_ID) &&
	    (sii->pub.chippkg != BCM4712LARGE_PKG_ID) && (CHIPREV(sii->pub.chiprev) <= 3))
		OR_REG(sii->osh, &cc->slow_clk_ctl, SCC_SS_XTAL);


	if ((BUSTYPE(bustype) == SDIO_BUS) || (BUSTYPE(bustype) == SPI_BUS)) {
		if (si_setcore(&sii->pub, ARM7S_CORE_ID, 0) ||
		    si_setcore(&sii->pub, ARMCM3_CORE_ID, 0))
			si_core_disable(&sii->pub, 0);
	}

	
	si_setcoreidx(&sii->pub, *origidx);

	return TRUE;
}




static si_info_t *
si_doattach(si_info_t *sii, uint devid, osl_t *osh, void *regs,
                       uint bustype, void *sdh, char **vars, uint *varsz)
{
	struct si_pub *sih = &sii->pub;
	uint32 w, savewin;
	chipcregs_t *cc;
	char *pvars = NULL;
	uint origidx;

	ASSERT(GOODREGS(regs));

	bzero((uchar*)sii, sizeof(si_info_t));

	savewin = 0;

	sih->buscoreidx = BADIDX;

	sii->curmap = regs;
	sii->sdh = sdh;
	sii->osh = osh;



	
	if (bustype == PCI_BUS) {
		savewin = OSL_PCI_READ_CONFIG(sii->osh, PCI_BAR0_WIN, sizeof(uint32));
		if (!GOODCOREADDR(savewin, SI_ENUM_BASE))
			savewin = SI_ENUM_BASE;
		OSL_PCI_WRITE_CONFIG(sii->osh, PCI_BAR0_WIN, 4, SI_ENUM_BASE);
		if (!regs)
			return NULL;
		cc = (chipcregs_t *)regs;
	} else if ((bustype == SDIO_BUS) || (bustype == SPI_BUS)) {
		cc = (chipcregs_t *)sii->curmap;
	} else {
		cc = (chipcregs_t *)REG_MAP(SI_ENUM_BASE, SI_CORE_SIZE);
	}

	sih->bustype = bustype;
	if (bustype != BUSTYPE(bustype)) {
		SI_ERROR(("si_doattach: bus type %d does not match configured bus type %d\n",
			bustype, BUSTYPE(bustype)));
		return NULL;
	}

	
	if (!si_buscore_prep(sii, bustype, devid, sdh)) {
		SI_ERROR(("si_doattach: si_core_clk_prep failed %d\n", bustype));
		return NULL;
	}

#ifdef HTC_KlocWork
	if (!cc) {
		SI_ERROR(("%s: chipcommon register space is null \n", __FUNCTION__));
		return NULL;
	}
#endif
	w = R_REG(osh, &cc->chipid);
	sih->socitype = (w & CID_TYPE_MASK) >> CID_TYPE_SHIFT;
	
	sih->chip = w & CID_ID_MASK;
	sih->chiprev = (w & CID_REV_MASK) >> CID_REV_SHIFT;
	if (sih->chip == BCM4330_CHIP_ID) {
		printf("sih->chiprev = %d\n", sih->chiprev);
		bcm_chip_is_4330 = 1;
		if (sih->chiprev == 3)
			bcm_chip_is_4330b1 = 1;
	}
	sih->chippkg = (w & CID_PKG_MASK) >> CID_PKG_SHIFT;

	if ((CHIPID(sih->chip) == BCM4329_CHIP_ID) && (sih->chiprev == 0) &&
		(sih->chippkg != BCM4329_289PIN_PKG_ID)) {
		sih->chippkg = BCM4329_182PIN_PKG_ID;
	}
	sih->issim = IS_SIM(sih->chippkg);

	
	if (CHIPTYPE(sii->pub.socitype) == SOCI_SB) {
		SI_MSG(("Found chip type SB (0x%08x)\n", w));
		sb_scan(&sii->pub, regs, devid);
	} else if (CHIPTYPE(sii->pub.socitype) == SOCI_AI) {
		SI_MSG(("Found chip type AI (0x%08x)\n", w));
		
		ai_scan(&sii->pub, (void *)(uintptr)cc, devid);
	} else if (CHIPTYPE(sii->pub.socitype) == SOCI_UBUS) {
		SI_MSG(("Found chip type UBUS (0x%08x), chip id = 0x%4x\n", w, sih->chip));
		
		ub_scan(&sii->pub, (void *)(uintptr)cc, devid);
	} else {
		SI_ERROR(("Found chip of unknown type (0x%08x)\n", w));
		return NULL;
	}
	
	if (sii->numcores == 0) {
		SI_ERROR(("si_doattach: could not find any cores\n"));
		return NULL;
	}
	
	origidx = SI_CC_IDX;
	if (!si_buscore_setup(sii, cc, bustype, savewin, &origidx, regs)) {
		SI_ERROR(("si_doattach: si_buscore_setup failed\n"));
		goto exit;
	}

	if (CHIPID(sih->chip) == BCM4322_CHIP_ID && (((sih->chipst & CST4322_SPROM_OTP_SEL_MASK)
		>> CST4322_SPROM_OTP_SEL_SHIFT) == (CST4322_OTP_PRESENT |
		CST4322_SPROM_PRESENT))) {
		SI_ERROR(("%s: Invalid setting: both SPROM and OTP strapped.\n", __FUNCTION__));
		return NULL;
	}

	
	if ((sii->pub.ccrev == 0x25) && ((CHIPID(sih->chip) == BCM43236_CHIP_ID ||
	                                  CHIPID(sih->chip) == BCM43235_CHIP_ID ||
	                                  CHIPID(sih->chip) == BCM43234_CHIP_ID ||
	                                  CHIPID(sih->chip) == BCM43238_CHIP_ID) &&
	                                 (CHIPREV(sii->pub.chiprev) <= 2))) {

		if ((cc->chipstatus & CST43236_BP_CLK) != 0) {
			uint clkdiv;
			clkdiv = R_REG(osh, &cc->clkdiv);
			
			clkdiv = (clkdiv & ~CLKD_OTP) | (14 << CLKD_OTP_SHIFT);
			W_REG(osh, &cc->clkdiv, clkdiv);
			SI_ERROR(("%s: set clkdiv to %x\n", __FUNCTION__, clkdiv));
		}
		OSL_DELAY(10);
	}

	if (bustype == PCI_BUS) {

	}

	pvars = NULL;
	BCM_REFERENCE(pvars);



		if (sii->pub.ccrev >= 20) {
			uint32 gpiopullup = 0, gpiopulldown = 0;
			cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);
#ifdef HTC_KlocWork
    if (cc == NULL) {
        SI_ERROR(("[HTCKW] si_doattach: cc is NULL-2\n"));
        return NULL;
    }
#endif
			ASSERT(cc != NULL);

			
			if ((CHIPID(sih->chip) == BCM4314_CHIP_ID) ||
				(CHIPID(sih->chip) == BCM43142_CHIP_ID)) {
				gpiopullup |= 0x402e0;
				gpiopulldown |= 0x20500;
			}

			W_REG(osh, &cc->gpiopullup, gpiopullup);
			W_REG(osh, &cc->gpiopulldown, gpiopulldown);
			si_setcoreidx(sih, origidx);
		}


	
	ASSERT(!si_taclear(sih, FALSE));

	return (sii);

exit:

	return NULL;
}

void
si_detach(si_t *sih)
{
	si_info_t *sii;
	uint idx;


	sii = SI_INFO(sih);

	if (sii == NULL)
		return;

	if (BUSTYPE(sih->bustype) == SI_BUS)
		for (idx = 0; idx < SI_MAXCORES; idx++)
			if (sii->regs[idx]) {
				REG_UNMAP(sii->regs[idx]);
				sii->regs[idx] = NULL;
			}



#if !defined(BCMBUSTYPE) || (BCMBUSTYPE == SI_BUS)
	if (sii != &ksii)
#endif	
		MFREE(sii->osh, sii, sizeof(si_info_t));
}

void *
si_osh(si_t *sih)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	return sii->osh;
}

void
si_setosh(si_t *sih, osl_t *osh)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	if (sii->osh != NULL) {
		SI_ERROR(("osh is already set....\n"));
		ASSERT(!sii->osh);
	}
	sii->osh = osh;
}

void
si_register_intr_callback(si_t *sih, void *intrsoff_fn, void *intrsrestore_fn,
                          void *intrsenabled_fn, void *intr_arg)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	sii->intr_arg = intr_arg;
	sii->intrsoff_fn = (si_intrsoff_t)intrsoff_fn;
	sii->intrsrestore_fn = (si_intrsrestore_t)intrsrestore_fn;
	sii->intrsenabled_fn = (si_intrsenabled_t)intrsenabled_fn;
	sii->dev_coreid = sii->coreid[sii->curidx];
}

void
si_deregister_intr_callback(si_t *sih)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	sii->intrsoff_fn = NULL;
}

uint
si_intflag(si_t *sih)
{
	si_info_t *sii = SI_INFO(sih);

	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_intflag(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return R_REG(sii->osh, ((uint32 *)(uintptr)
			    (sii->oob_router + OOB_STATUSA)));
	else {
		ASSERT(0);
		return 0;
	}
}

uint
si_flag(si_t *sih)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_flag(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_flag(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_flag(sih);
	else {
		ASSERT(0);
		return 0;
	}
}

void
si_setint(si_t *sih, int siflag)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		sb_setint(sih, siflag);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		ai_setint(sih, siflag);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		ub_setint(sih, siflag);
	else
		ASSERT(0);
}

uint
si_coreid(si_t *sih)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	return sii->coreid[sii->curidx];
}

uint
si_coreidx(si_t *sih)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	return sii->curidx;
}

uint
si_coreunit(si_t *sih)
{
	si_info_t *sii;
	uint idx;
	uint coreid;
	uint coreunit;
	uint i;

	sii = SI_INFO(sih);
	coreunit = 0;

	idx = sii->curidx;

	ASSERT(GOODREGS(sii->curmap));
	coreid = si_coreid(sih);

	
	for (i = 0; i < idx; i++)
		if (sii->coreid[i] == coreid)
			coreunit++;

	return (coreunit);
}

uint
si_corevendor(si_t *sih)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_corevendor(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_corevendor(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_corevendor(sih);
	else {
		ASSERT(0);
		return 0;
	}
}

bool
si_backplane64(si_t *sih)
{
	return ((sih->cccaps & CC_CAP_BKPLN64) != 0);
}

uint
si_corerev(si_t *sih)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_corerev(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_corerev(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_corerev(sih);
	else {
		ASSERT(0);
		return 0;
	}
}

uint
si_findcoreidx(si_t *sih, uint coreid, uint coreunit)
{
	si_info_t *sii;
	uint found;
	uint i;

	sii = SI_INFO(sih);

	found = 0;

	for (i = 0; i < sii->numcores; i++)
		if (sii->coreid[i] == coreid) {
			if (found == coreunit)
				return (i);
			found++;
		}

	return (BADIDX);
}

uint
si_corelist(si_t *sih, uint coreid[])
{
	si_info_t *sii;

	sii = SI_INFO(sih);

	bcopy((uchar*)sii->coreid, (uchar*)coreid, (sii->numcores * sizeof(uint)));
	return (sii->numcores);
}

void *
si_coreregs(si_t *sih)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	ASSERT(GOODREGS(sii->curmap));

	return (sii->curmap);
}

void *
si_setcore(si_t *sih, uint coreid, uint coreunit)
{
	uint idx;

	if (!local_sih || (sih != local_sih))
		return (NULL);
	
	idx = si_findcoreidx(sih, coreid, coreunit);
	if (!GOODIDX(idx)){
		SI_ERROR(("GOODIDX Error\n"));
		return (NULL);
	}

	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_setcoreidx(sih, idx);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_setcoreidx(sih, idx);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_setcoreidx(sih, idx);
	else {
		ASSERT(0);
		return NULL;
	}
}

void *
si_setcoreidx(si_t *sih, uint coreidx)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_setcoreidx(sih, coreidx);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_setcoreidx(sih, coreidx);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_setcoreidx(sih, coreidx);
	else {
		ASSERT(0);
		return NULL;
	}
}

void *
si_switch_core(si_t *sih, uint coreid, uint *origidx, uint *intr_val)
{
	void *cc;
	si_info_t *sii;

	sii = SI_INFO(sih);

	if (SI_FAST(sii)) {
		*origidx = coreid;
		if (coreid == CC_CORE_ID)
			return (void *)CCREGS_FAST(sii);
		else if (coreid == sih->buscoretype)
			return (void *)PCIEREGS(sii);
	}
	INTR_OFF(sii, *intr_val);
	*origidx = sii->curidx;
	cc = si_setcore(sih, coreid, 0);
	ASSERT(cc != NULL);

	return cc;
}

void
si_restore_core(si_t *sih, uint coreid, uint intr_val)
{
	si_info_t *sii;

	sii = SI_INFO(sih);
	if (SI_FAST(sii) && ((coreid == CC_CORE_ID) || (coreid == sih->buscoretype)))
		return;

	si_setcoreidx(sih, coreid);
	INTR_RESTORE(sii, intr_val);
}

int
si_numaddrspaces(si_t *sih)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_numaddrspaces(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_numaddrspaces(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_numaddrspaces(sih);
	else {
		ASSERT(0);
		return 0;
	}
}

uint32
si_addrspace(si_t *sih, uint asidx)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_addrspace(sih, asidx);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_addrspace(sih, asidx);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_addrspace(sih, asidx);
	else {
		ASSERT(0);
		return 0;
	}
}

uint32
si_addrspacesize(si_t *sih, uint asidx)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_addrspacesize(sih, asidx);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_addrspacesize(sih, asidx);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_addrspacesize(sih, asidx);
	else {
		ASSERT(0);
		return 0;
	}
}

void
si_coreaddrspaceX(si_t *sih, uint asidx, uint32 *addr, uint32 *size)
{
	
	if (CHIPTYPE(sih->socitype) == SOCI_AI)
		ai_coreaddrspaceX(sih, asidx, addr, size);
	else
		*size = 0;
}

uint32
si_core_cflags(si_t *sih, uint32 mask, uint32 val)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_core_cflags(sih, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_core_cflags(sih, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_core_cflags(sih, mask, val);
	else {
		ASSERT(0);
		return 0;
	}
}

void
si_core_cflags_wo(si_t *sih, uint32 mask, uint32 val)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		sb_core_cflags_wo(sih, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		ai_core_cflags_wo(sih, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		ub_core_cflags_wo(sih, mask, val);
	else
		ASSERT(0);
}

uint32
si_core_sflags(si_t *sih, uint32 mask, uint32 val)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_core_sflags(sih, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_core_sflags(sih, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_core_sflags(sih, mask, val);
	else {
		ASSERT(0);
		return 0;
	}
}

bool
si_iscoreup(si_t *sih)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_iscoreup(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_iscoreup(sih);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_iscoreup(sih);
	else {
		ASSERT(0);
		return FALSE;
	}
}

uint
si_wrapperreg(si_t *sih, uint32 offset, uint32 mask, uint32 val)
{
	
	if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return (ai_wrap_reg(sih, offset, mask, val));
	return 0;
}

uint
si_corereg(si_t *sih, uint coreidx, uint regoff, uint mask, uint val)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		return sb_corereg(sih, coreidx, regoff, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		return ai_corereg(sih, coreidx, regoff, mask, val);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		return ub_corereg(sih, coreidx, regoff, mask, val);
	else {
		ASSERT(0);
		return 0;
	}
}

void
si_core_disable(si_t *sih, uint32 bits)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		sb_core_disable(sih, bits);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		ai_core_disable(sih, bits);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		ub_core_disable(sih, bits);
}

void
si_core_reset(si_t *sih, uint32 bits, uint32 resetbits)
{
	if (CHIPTYPE(sih->socitype) == SOCI_SB)
		sb_core_reset(sih, bits, resetbits);
	else if (CHIPTYPE(sih->socitype) == SOCI_AI)
		ai_core_reset(sih, bits, resetbits);
	else if (CHIPTYPE(sih->socitype) == SOCI_UBUS)
		ub_core_reset(sih, bits, resetbits);
}

int
si_corebist(si_t *sih)
{
	uint32 cflags;
	int result = 0;

	
	cflags = si_core_cflags(sih, 0, 0);

	
	si_core_cflags(sih, ~0, (SICF_BIST_EN | SICF_FGC));

	
	SPINWAIT(((si_core_sflags(sih, 0, 0) & SISF_BIST_DONE) == 0), 100000);

	if (si_core_sflags(sih, 0, 0) & SISF_BIST_ERROR)
		result = BCME_ERROR;

	
	si_core_cflags(sih, 0xffff, cflags);

	return result;
}

static uint32
factor6(uint32 x)
{
	switch (x) {
	case CC_F6_2:	return 2;
	case CC_F6_3:	return 3;
	case CC_F6_4:	return 4;
	case CC_F6_5:	return 5;
	case CC_F6_6:	return 6;
	case CC_F6_7:	return 7;
	default:	return 0;
	}
}

uint32
si_clock_rate(uint32 pll_type, uint32 n, uint32 m)
{
	uint32 n1, n2, clock, m1, m2, m3, mc;

	n1 = n & CN_N1_MASK;
	n2 = (n & CN_N2_MASK) >> CN_N2_SHIFT;

	if (pll_type == PLL_TYPE6) {
		if (m & CC_T6_MMASK)
			return CC_T6_M1;
		else
			return CC_T6_M0;
	} else if ((pll_type == PLL_TYPE1) ||
	           (pll_type == PLL_TYPE3) ||
	           (pll_type == PLL_TYPE4) ||
	           (pll_type == PLL_TYPE7)) {
		n1 = factor6(n1);
		n2 += CC_F5_BIAS;
	} else if (pll_type == PLL_TYPE2) {
		n1 += CC_T2_BIAS;
		n2 += CC_T2_BIAS;
		ASSERT((n1 >= 2) && (n1 <= 7));
		ASSERT((n2 >= 5) && (n2 <= 23));
	} else if (pll_type == PLL_TYPE5) {
		return (100000000);
	} else
		ASSERT(0);
	
	if ((pll_type == PLL_TYPE3) ||
	    (pll_type == PLL_TYPE7)) {
		clock = CC_CLOCK_BASE2 * n1 * n2;
	} else
		clock = CC_CLOCK_BASE1 * n1 * n2;

	if (clock == 0)
		return 0;

	m1 = m & CC_M1_MASK;
	m2 = (m & CC_M2_MASK) >> CC_M2_SHIFT;
	m3 = (m & CC_M3_MASK) >> CC_M3_SHIFT;
	mc = (m & CC_MC_MASK) >> CC_MC_SHIFT;

	if ((pll_type == PLL_TYPE1) ||
	    (pll_type == PLL_TYPE3) ||
	    (pll_type == PLL_TYPE4) ||
	    (pll_type == PLL_TYPE7)) {
		m1 = factor6(m1);
		if ((pll_type == PLL_TYPE1) || (pll_type == PLL_TYPE3))
			m2 += CC_F5_BIAS;
		else
			m2 = factor6(m2);
		m3 = factor6(m3);

		switch (mc) {
		case CC_MC_BYPASS:	return (clock);
		case CC_MC_M1:		return (clock / m1);
		case CC_MC_M1M2:	return (clock / (m1 * m2));
		case CC_MC_M1M2M3:	return (clock / (m1 * m2 * m3));
		case CC_MC_M1M3:	return (clock / (m1 * m3));
		default:		return (0);
		}
	} else {
		ASSERT(pll_type == PLL_TYPE2);

		m1 += CC_T2_BIAS;
		m2 += CC_T2M2_BIAS;
		m3 += CC_T2_BIAS;
		ASSERT((m1 >= 2) && (m1 <= 7));
		ASSERT((m2 >= 3) && (m2 <= 10));
		ASSERT((m3 >= 2) && (m3 <= 7));

		if ((mc & CC_T2MC_M1BYP) == 0)
			clock /= m1;
		if ((mc & CC_T2MC_M2BYP) == 0)
			clock /= m2;
		if ((mc & CC_T2MC_M3BYP) == 0)
			clock /= m3;

		return (clock);
	}
}


void
si_watchdog(si_t *sih, uint ticks)
{
	uint nb, maxt;

	if (PMUCTL_ENAB(sih)) {

		if ((CHIPID(sih->chip) == BCM4319_CHIP_ID) &&
		    (CHIPREV(sih->chiprev) == 0) && (ticks != 0)) {
			si_corereg(sih, SI_CC_IDX, OFFSETOF(chipcregs_t, clk_ctl_st), ~0, 0x2);
			si_setcore(sih, USB20D_CORE_ID, 0);
			si_core_disable(sih, 1);
			si_setcore(sih, CC_CORE_ID, 0);
		}

			nb = (sih->ccrev < 26) ? 16 : ((sih->ccrev >= 37) ? 32 : 24);
		if (nb == 32)
			maxt = 0xffffffff;
		else
			maxt = ((1 << nb) - 1);

		if (ticks == 1)
			ticks = 2;
		else if (ticks > maxt)
			ticks = maxt;

		si_corereg(sih, SI_CC_IDX, OFFSETOF(chipcregs_t, pmuwatchdog), ~0, ticks);
	} else {
		maxt = (1 << 28) - 1;
		if (ticks > maxt)
			ticks = maxt;

		si_corereg(sih, SI_CC_IDX, OFFSETOF(chipcregs_t, watchdog), ~0, ticks);
	}
}

void
si_watchdog_ms(si_t *sih, uint32 ms)
{
	si_watchdog(sih, wd_msticks * ms);
}

uint32 si_watchdog_msticks(void)
{
	return wd_msticks;
}

bool
si_taclear(si_t *sih, bool details)
{
	return FALSE;
}



static uint
si_slowclk_src(si_info_t *sii)
{
	chipcregs_t *cc;

	ASSERT(SI_FAST(sii) || si_coreid(&sii->pub) == CC_CORE_ID);

	if (sii->pub.ccrev < 6) {
		if ((BUSTYPE(sii->pub.bustype) == PCI_BUS) &&
		    (OSL_PCI_READ_CONFIG(sii->osh, PCI_GPIO_OUT, sizeof(uint32)) &
		     PCI_CFG_GPIO_SCS))
			return (SCC_SS_PCI);
		else
			return (SCC_SS_XTAL);
	} else if (sii->pub.ccrev < 10) {
		cc = (chipcregs_t *)si_setcoreidx(&sii->pub, sii->curidx);
#ifdef HTC_KlocWork
		if (cc == NULL) {
			SI_ERROR(("[HTCKW] si_slowclk_src: cc is NULL-1\n"));
			return -1;
		}
#endif
		return (R_REG(sii->osh, &cc->slow_clk_ctl) & SCC_SS_MASK);
	} else	
		return (SCC_SS_XTAL);
}

static uint
si_slowclk_freq(si_info_t *sii, bool max_freq, chipcregs_t *cc)
{
	uint32 slowclk;
	uint div;

	ASSERT(SI_FAST(sii) || si_coreid(&sii->pub) == CC_CORE_ID);

	
	ASSERT(R_REG(sii->osh, &cc->capabilities) & CC_CAP_PWR_CTL);

	slowclk = si_slowclk_src(sii);
	if (sii->pub.ccrev < 6) {
		if (slowclk == SCC_SS_PCI)
			return (max_freq ? (PCIMAXFREQ / 64) : (PCIMINFREQ / 64));
		else
			return (max_freq ? (XTALMAXFREQ / 32) : (XTALMINFREQ / 32));
	} else if (sii->pub.ccrev < 10) {
		div = 4 *
		        (((R_REG(sii->osh, &cc->slow_clk_ctl) & SCC_CD_MASK) >> SCC_CD_SHIFT) + 1);
		if (slowclk == SCC_SS_LPO)
			return (max_freq ? LPOMAXFREQ : LPOMINFREQ);
		else if (slowclk == SCC_SS_XTAL)
			return (max_freq ? (XTALMAXFREQ / div) : (XTALMINFREQ / div));
		else if (slowclk == SCC_SS_PCI)
			return (max_freq ? (PCIMAXFREQ / div) : (PCIMINFREQ / div));
		else
			ASSERT(0);
	} else {
		
		div = R_REG(sii->osh, &cc->system_clk_ctl) >> SYCC_CD_SHIFT;
		div = 4 * (div + 1);
		return (max_freq ? XTALMAXFREQ : (XTALMINFREQ / div));
	}
	return (0);
}

static void
si_clkctl_setdelay(si_info_t *sii, void *chipcregs)
{
	chipcregs_t *cc = (chipcregs_t *)chipcregs;
	uint slowmaxfreq, pll_delay, slowclk;
	uint pll_on_delay, fref_sel_delay;

	pll_delay = PLL_DELAY;


	slowclk = si_slowclk_src(sii);
	if (slowclk != SCC_SS_XTAL)
		pll_delay += XTAL_ON_DELAY;

	
	slowmaxfreq = si_slowclk_freq(sii, (sii->pub.ccrev >= 10) ? FALSE : TRUE, cc);

	pll_on_delay = ((slowmaxfreq * pll_delay) + 999999) / 1000000;
	fref_sel_delay = ((slowmaxfreq * FREF_DELAY) + 999999) / 1000000;

	W_REG(sii->osh, &cc->pll_on_delay, pll_on_delay);
	W_REG(sii->osh, &cc->fref_sel_delay, fref_sel_delay);
}

void
si_clkctl_init(si_t *sih)
{
	si_info_t *sii;
	uint origidx = 0;
	chipcregs_t *cc;
	bool fast;

	if (!CCCTL_ENAB(sih))
		return;

	sii = SI_INFO(sih);
	fast = SI_FAST(sii);
	if (!fast) {
		origidx = sii->curidx;
		if ((cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0)) == NULL)
			return;
	} else if ((cc = (chipcregs_t *)CCREGS_FAST(sii)) == NULL)
		return;
	ASSERT(cc != NULL);

	
	if (sih->ccrev >= 10)
		SET_REG(sii->osh, &cc->system_clk_ctl, SYCC_CD_MASK,
		        (ILP_DIV_1MHZ << SYCC_CD_SHIFT));

	si_clkctl_setdelay(sii, (void *)(uintptr)cc);

	if (!fast)
		si_setcoreidx(sih, origidx);
}


void *
si_gpiosetcore(si_t *sih)
{
	return (si_setcoreidx(sih, SI_CC_IDX));
}

uint32
si_gpiocontrol(si_t *sih, uint32 mask, uint32 val, uint8 priority)
{
	uint regoff;

	regoff = 0;

	if ((priority != GPIO_HI_PRIORITY) &&
	    (BUSTYPE(sih->bustype) == SI_BUS) && (val || mask)) {
		mask = priority ? (si_gpioreservation & mask) :
			((si_gpioreservation | mask) & ~(si_gpioreservation));
		val &= mask;
	}

	regoff = OFFSETOF(chipcregs_t, gpiocontrol);
	return (si_corereg(sih, SI_CC_IDX, regoff, mask, val));
}

uint32
si_gpioouten(si_t *sih, uint32 mask, uint32 val, uint8 priority)
{
	uint regoff;

	regoff = 0;

	if ((priority != GPIO_HI_PRIORITY) &&
	    (BUSTYPE(sih->bustype) == SI_BUS) && (val || mask)) {
		mask = priority ? (si_gpioreservation & mask) :
			((si_gpioreservation | mask) & ~(si_gpioreservation));
		val &= mask;
	}

	regoff = OFFSETOF(chipcregs_t, gpioouten);
	return (si_corereg(sih, SI_CC_IDX, regoff, mask, val));
}

uint32
si_gpioout(si_t *sih, uint32 mask, uint32 val, uint8 priority)
{
	uint regoff;

	regoff = 0;

	if ((priority != GPIO_HI_PRIORITY) &&
	    (BUSTYPE(sih->bustype) == SI_BUS) && (val || mask)) {
		mask = priority ? (si_gpioreservation & mask) :
			((si_gpioreservation | mask) & ~(si_gpioreservation));
		val &= mask;
	}

	regoff = OFFSETOF(chipcregs_t, gpioout);
	return (si_corereg(sih, SI_CC_IDX, regoff, mask, val));
}

uint32
si_gpioreserve(si_t *sih, uint32 gpio_bitmask, uint8 priority)
{
	if ((BUSTYPE(sih->bustype) != SI_BUS) || (!priority)) {
		ASSERT((BUSTYPE(sih->bustype) == SI_BUS) && (priority));
		return 0xffffffff;
	}
	
	if ((!gpio_bitmask) || ((gpio_bitmask) & (gpio_bitmask - 1))) {
		ASSERT((gpio_bitmask) && !((gpio_bitmask) & (gpio_bitmask - 1)));
		return 0xffffffff;
	}

	
	if (si_gpioreservation & gpio_bitmask)
		return 0xffffffff;
	
	si_gpioreservation |= gpio_bitmask;

	return si_gpioreservation;
}


uint32
si_gpiorelease(si_t *sih, uint32 gpio_bitmask, uint8 priority)
{
	if ((BUSTYPE(sih->bustype) != SI_BUS) || (!priority)) {
		ASSERT((BUSTYPE(sih->bustype) == SI_BUS) && (priority));
		return 0xffffffff;
	}
	
	if ((!gpio_bitmask) || ((gpio_bitmask) & (gpio_bitmask - 1))) {
		ASSERT((gpio_bitmask) && !((gpio_bitmask) & (gpio_bitmask - 1)));
		return 0xffffffff;
	}

	
	if (!(si_gpioreservation & gpio_bitmask))
		return 0xffffffff;

	
	si_gpioreservation &= ~gpio_bitmask;

	return si_gpioreservation;
}

uint32
si_gpioin(si_t *sih)
{
	uint regoff;

	regoff = OFFSETOF(chipcregs_t, gpioin);
	return (si_corereg(sih, SI_CC_IDX, regoff, 0, 0));
}

uint32
si_gpiointpolarity(si_t *sih, uint32 mask, uint32 val, uint8 priority)
{
	uint regoff;

	
	if ((BUSTYPE(sih->bustype) == SI_BUS) && (val || mask)) {
		mask = priority ? (si_gpioreservation & mask) :
			((si_gpioreservation | mask) & ~(si_gpioreservation));
		val &= mask;
	}

	regoff = OFFSETOF(chipcregs_t, gpiointpolarity);
	return (si_corereg(sih, SI_CC_IDX, regoff, mask, val));
}

uint32
si_gpiointmask(si_t *sih, uint32 mask, uint32 val, uint8 priority)
{
	uint regoff;

	
	if ((BUSTYPE(sih->bustype) == SI_BUS) && (val || mask)) {
		mask = priority ? (si_gpioreservation & mask) :
			((si_gpioreservation | mask) & ~(si_gpioreservation));
		val &= mask;
	}

	regoff = OFFSETOF(chipcregs_t, gpiointmask);
	return (si_corereg(sih, SI_CC_IDX, regoff, mask, val));
}

uint32
si_gpioled(si_t *sih, uint32 mask, uint32 val)
{
	if (sih->ccrev < 16)
		return 0xffffffff;

	
	return (si_corereg(sih, SI_CC_IDX, OFFSETOF(chipcregs_t, gpiotimeroutmask), mask, val));
}

uint32
si_gpiotimerval(si_t *sih, uint32 mask, uint32 gpiotimerval)
{
	if (sih->ccrev < 16)
		return 0xffffffff;

	return (si_corereg(sih, SI_CC_IDX,
		OFFSETOF(chipcregs_t, gpiotimerval), mask, gpiotimerval));
}

uint32
si_gpiopull(si_t *sih, bool updown, uint32 mask, uint32 val)
{
	uint offs;

	if (sih->ccrev < 20)
		return 0xffffffff;

	offs = (updown ? OFFSETOF(chipcregs_t, gpiopulldown) : OFFSETOF(chipcregs_t, gpiopullup));
	return (si_corereg(sih, SI_CC_IDX, offs, mask, val));
}

uint32
si_gpioevent(si_t *sih, uint regtype, uint32 mask, uint32 val)
{
	uint offs;

	if (sih->ccrev < 11)
		return 0xffffffff;

	if (regtype == GPIO_REGEVT)
		offs = OFFSETOF(chipcregs_t, gpioevent);
	else if (regtype == GPIO_REGEVT_INTMSK)
		offs = OFFSETOF(chipcregs_t, gpioeventintmask);
	else if (regtype == GPIO_REGEVT_INTPOL)
		offs = OFFSETOF(chipcregs_t, gpioeventintpolarity);
	else
		return 0xffffffff;

	return (si_corereg(sih, SI_CC_IDX, offs, mask, val));
}

void *
si_gpio_handler_register(si_t *sih, uint32 event,
	bool level, gpio_handler_t cb, void *arg)
{
	si_info_t *sii;
	gpioh_item_t *gi;

	ASSERT(event);
	ASSERT(cb != NULL);

	sii = SI_INFO(sih);
	if (sih->ccrev < 11)
		return NULL;

	if ((gi = MALLOC(sii->osh, sizeof(gpioh_item_t))) == NULL)
		return NULL;

	bzero(gi, sizeof(gpioh_item_t));
	gi->event = event;
	gi->handler = cb;
	gi->arg = arg;
	gi->level = level;

	gi->next = sii->gpioh_head;
	sii->gpioh_head = gi;

	return (void *)(gi);
}

void
si_gpio_handler_unregister(si_t *sih, void *gpioh)
{
	si_info_t *sii;
	gpioh_item_t *p, *n;

	sii = SI_INFO(sih);
	if (sih->ccrev < 11)
		return;

	ASSERT(sii->gpioh_head != NULL);
	if ((void*)sii->gpioh_head == gpioh) {
		sii->gpioh_head = sii->gpioh_head->next;
		MFREE(sii->osh, gpioh, sizeof(gpioh_item_t));
		return;
	} else {
		p = sii->gpioh_head;
		n = p->next;
		while (n) {
			if ((void*)n == gpioh) {
				p->next = n->next;
				MFREE(sii->osh, gpioh, sizeof(gpioh_item_t));
				return;
			}
			p = n;
			n = n->next;
		}
	}

	ASSERT(0); 
}

void
si_gpio_handler_process(si_t *sih)
{
	si_info_t *sii;
	gpioh_item_t *h;
	uint32 level = si_gpioin(sih);
	uint32 levelp = si_gpiointpolarity(sih, 0, 0, 0);
	uint32 edge = si_gpioevent(sih, GPIO_REGEVT, 0, 0);
	uint32 edgep = si_gpioevent(sih, GPIO_REGEVT_INTPOL, 0, 0);

	sii = SI_INFO(sih);
	for (h = sii->gpioh_head; h != NULL; h = h->next) {
		if (h->handler) {
			uint32 status = (h->level ? level : edge) & h->event;
			uint32 polarity = (h->level ? levelp : edgep) & h->event;

			
			if (status ^ polarity)
				h->handler(status, h->arg);
		}
	}

	si_gpioevent(sih, GPIO_REGEVT, edge, edge); 
}

uint32
si_gpio_int_enable(si_t *sih, bool enable)
{
	uint offs;

	if (sih->ccrev < 11)
		return 0xffffffff;

	offs = OFFSETOF(chipcregs_t, intmask);
	return (si_corereg(sih, SI_CC_IDX, offs, CI_GPIO, (enable ? CI_GPIO : 0)));
}


static uint
socram_banksize(si_info_t *sii, sbsocramregs_t *regs, uint8 idx, uint8 mem_type)
{
	uint banksize, bankinfo;
	uint bankidx = idx | (mem_type << SOCRAM_BANKIDX_MEMTYPE_SHIFT);

	ASSERT(mem_type <= SOCRAM_MEMTYPE_DEVRAM);

	W_REG(sii->osh, &regs->bankidx, bankidx);
	bankinfo = R_REG(sii->osh, &regs->bankinfo);
	banksize = SOCRAM_BANKINFO_SZBASE * ((bankinfo & SOCRAM_BANKINFO_SZMASK) + 1);
	return banksize;
}

void
si_socdevram(si_t *sih, bool set, uint8 *enable, uint8 *protect, uint8 *remap)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;
	sbsocramregs_t *regs;
	bool wasup;
	uint corerev;

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	if (!set)
		*enable = *protect = *remap = 0;

	
	if (!(regs = si_setcore(sih, SOCRAM_CORE_ID, 0)))
		goto done;

	
	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, 0, 0);

	corerev = si_corerev(sih);
	if (corerev >= 10) {
		uint32 extcinfo;
		uint8 nb;
		uint8 i;
		uint32 bankidx, bankinfo;

		extcinfo = R_REG(sii->osh, &regs->extracoreinfo);
		nb = ((extcinfo & SOCRAM_DEVRAMBANK_MASK) >> SOCRAM_DEVRAMBANK_SHIFT);
		for (i = 0; i < nb; i++) {
			bankidx = i | (SOCRAM_MEMTYPE_DEVRAM << SOCRAM_BANKIDX_MEMTYPE_SHIFT);
			W_REG(sii->osh, &regs->bankidx, bankidx);
			bankinfo = R_REG(sii->osh, &regs->bankinfo);
			if (set) {
				bankinfo &= ~SOCRAM_BANKINFO_DEVRAMSEL_MASK;
				bankinfo &= ~SOCRAM_BANKINFO_DEVRAMPRO_MASK;
				bankinfo &= ~SOCRAM_BANKINFO_DEVRAMREMAP_MASK;
				if (*enable) {
					bankinfo |= (1 << SOCRAM_BANKINFO_DEVRAMSEL_SHIFT);
					if (*protect)
						bankinfo |= (1 << SOCRAM_BANKINFO_DEVRAMPRO_SHIFT);
					if ((corerev >= 16) && *remap)
						bankinfo |=
							(1 << SOCRAM_BANKINFO_DEVRAMREMAP_SHIFT);
				}
				W_REG(sii->osh, &regs->bankinfo, bankinfo);
			}
			else if (i == 0) {
				if (bankinfo & SOCRAM_BANKINFO_DEVRAMSEL_MASK) {
					*enable = 1;
					if (bankinfo & SOCRAM_BANKINFO_DEVRAMPRO_MASK)
						*protect = 1;
					if (bankinfo & SOCRAM_BANKINFO_DEVRAMREMAP_MASK)
						*remap = 1;
				}
			}
		}
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);
}

bool
si_socdevram_remap_isenb(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;
	sbsocramregs_t *regs;
	bool wasup, remap = FALSE;
	uint corerev;
	uint32 extcinfo;
	uint8 nb;
	uint8 i;
	uint32 bankidx, bankinfo;

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	
	if (!(regs = si_setcore(sih, SOCRAM_CORE_ID, 0)))
		goto done;

	
	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, 0, 0);

	corerev = si_corerev(sih);
	if (corerev >= 16) {
		extcinfo = R_REG(sii->osh, &regs->extracoreinfo);
		nb = ((extcinfo & SOCRAM_DEVRAMBANK_MASK) >> SOCRAM_DEVRAMBANK_SHIFT);
		for (i = 0; i < nb; i++) {
			bankidx = i | (SOCRAM_MEMTYPE_DEVRAM << SOCRAM_BANKIDX_MEMTYPE_SHIFT);
			W_REG(sii->osh, &regs->bankidx, bankidx);
			bankinfo = R_REG(sii->osh, &regs->bankinfo);
			if (bankinfo & SOCRAM_BANKINFO_DEVRAMREMAP_MASK) {
				remap = TRUE;
				break;
			}
		}
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);
	return remap;
}

bool
si_socdevram_pkg(si_t *sih)
{
	if (si_socdevram_size(sih) > 0)
		return TRUE;
	else
		return FALSE;
}

uint32
si_socdevram_size(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;
	uint32 memsize = 0;
	sbsocramregs_t *regs;
	bool wasup;
	uint corerev;

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	
	if (!(regs = si_setcore(sih, SOCRAM_CORE_ID, 0)))
		goto done;

	
	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, 0, 0);

	corerev = si_corerev(sih);
	if (corerev >= 10) {
		uint32 extcinfo;
		uint8 nb;
		uint8 i;

		extcinfo = R_REG(sii->osh, &regs->extracoreinfo);
		nb = (((extcinfo & SOCRAM_DEVRAMBANK_MASK) >> SOCRAM_DEVRAMBANK_SHIFT));
		for (i = 0; i < nb; i++)
			memsize += socram_banksize(sii, regs, i, SOCRAM_MEMTYPE_DEVRAM);
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);

	return memsize;
}

uint32
si_socdevram_remap_size(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;
	uint32 memsize = 0, banksz;
	sbsocramregs_t *regs;
	bool wasup;
	uint corerev;
	uint32 extcinfo;
	uint8 nb;
	uint8 i;
	uint32 bankidx, bankinfo;

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	
	if (!(regs = si_setcore(sih, SOCRAM_CORE_ID, 0)))
		goto done;

	
	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, 0, 0);

	corerev = si_corerev(sih);
	if (corerev >= 16) {
		extcinfo = R_REG(sii->osh, &regs->extracoreinfo);
		nb = (((extcinfo & SOCRAM_DEVRAMBANK_MASK) >> SOCRAM_DEVRAMBANK_SHIFT));

		if ((corerev == 16) && (nb == 5))
			nb = 4;

		for (i = 0; i < nb; i++) {
			bankidx = i | (SOCRAM_MEMTYPE_DEVRAM << SOCRAM_BANKIDX_MEMTYPE_SHIFT);
			W_REG(sii->osh, &regs->bankidx, bankidx);
			bankinfo = R_REG(sii->osh, &regs->bankinfo);
			if (bankinfo & SOCRAM_BANKINFO_DEVRAMREMAP_MASK) {
				banksz = socram_banksize(sii, regs, i, SOCRAM_MEMTYPE_DEVRAM);
				memsize += banksz;
			} else {
				
				break;
			}
		}
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);

	return memsize;
}

uint32
si_socram_size(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;

	sbsocramregs_t *regs;
	bool wasup;
	uint corerev;
	uint32 coreinfo;
	uint memsize = 0;

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	
	if (!(regs = si_setcore(sih, SOCRAM_CORE_ID, 0)))
		goto done;

	
	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, 0, 0);
	corerev = si_corerev(sih);
	coreinfo = R_REG(sii->osh, &regs->coreinfo);

	
	if (corerev == 0)
		memsize = 1 << (16 + (coreinfo & SRCI_MS0_MASK));
	else if (corerev < 3) {
		memsize = 1 << (SR_BSZ_BASE + (coreinfo & SRCI_SRBSZ_MASK));
		memsize *= (coreinfo & SRCI_SRNB_MASK) >> SRCI_SRNB_SHIFT;
	} else if ((corerev <= 7) || (corerev == 12)) {
		uint nb = (coreinfo & SRCI_SRNB_MASK) >> SRCI_SRNB_SHIFT;
		uint bsz = (coreinfo & SRCI_SRBSZ_MASK);
		uint lss = (coreinfo & SRCI_LSS_MASK) >> SRCI_LSS_SHIFT;
		if (lss != 0)
			nb --;
		memsize = nb * (1 << (bsz + SR_BSZ_BASE));
		if (lss != 0)
			memsize += (1 << ((lss - 1) + SR_BSZ_BASE));
	} else {
		uint8 i;
		uint nb = (coreinfo & SRCI_SRNB_MASK) >> SRCI_SRNB_SHIFT;
		for (i = 0; i < nb; i++)
			memsize += socram_banksize(sii, regs, i, SOCRAM_MEMTYPE_RAM);
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);

	return memsize;
}

uint32
si_tcm_size(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;
	uint8 *regs;
	bool wasup;
	uint32 corecap;
	uint memsize = 0;
	uint32 nab = 0;
	uint32 nbb = 0;
	uint32 totb = 0;
	uint32 bxinfo = 0;
	uint32 idx = 0;
	uint32 *arm_cap_reg;
	uint32 *arm_bidx;
	uint32 *arm_binfo;

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	
	if (!(regs = si_setcore(sih, ARMCR4_CORE_ID, 0)))
		goto done;

	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, SICF_CPUHALT, SICF_CPUHALT);

	arm_cap_reg = (uint32 *)(regs + SI_CR4_CAP);
	corecap = R_REG(sii->osh, arm_cap_reg);

	nab = (corecap & ARMCR4_TCBANB_MASK) >> ARMCR4_TCBANB_SHIFT;
	nbb = (corecap & ARMCR4_TCBBNB_MASK) >> ARMCR4_TCBBNB_SHIFT;
	totb = nab + nbb;

	arm_bidx = (uint32 *)(regs + SI_CR4_BANKIDX);
	arm_binfo = (uint32 *)(regs + SI_CR4_BANKINFO);
	for (idx = 0; idx < totb; idx++) {
		W_REG(sii->osh, arm_bidx, idx);

		bxinfo = R_REG(sii->osh, arm_binfo);
		memsize += ((bxinfo & ARMCR4_BSZ_MASK) + 1) * ARMCR4_BSZ_MULT;
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);

	return memsize;
}
uint32
si_socram_srmem_size(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;

	sbsocramregs_t *regs;
	bool wasup;
	uint corerev;
	uint32 coreinfo;
	uint memsize = 0;

	if ((CHIPID(sih->chip) == BCM4334_CHIP_ID) && (CHIPREV(sih->chiprev) < 2)) {
		return (32 * 1024);
	}

	sii = SI_INFO(sih);

	
	INTR_OFF(sii, intr_val);
	origidx = si_coreidx(sih);

	
	if (!(regs = si_setcore(sih, SOCRAM_CORE_ID, 0)))
		goto done;

	
	if (!(wasup = si_iscoreup(sih)))
		si_core_reset(sih, 0, 0);
	corerev = si_corerev(sih);
	coreinfo = R_REG(sii->osh, &regs->coreinfo);

	
	if (corerev >= 16) {
		uint8 i;
		uint nb = (coreinfo & SRCI_SRNB_MASK) >> SRCI_SRNB_SHIFT;
		for (i = 0; i < nb; i++) {
			W_REG(sii->osh, &regs->bankidx, i);
			if (R_REG(sii->osh, &regs->bankinfo) & SOCRAM_BANKINFO_RETNTRAM_MASK)
				memsize += socram_banksize(sii, regs, i, SOCRAM_MEMTYPE_RAM);
		}
	}

	
	if (!wasup)
		si_core_disable(sih, 0);
	si_setcoreidx(sih, origidx);

done:
	INTR_RESTORE(sii, intr_val);

	return memsize;
}


void
si_btcgpiowar(si_t *sih)
{
	si_info_t *sii;
	uint origidx;
	uint intr_val = 0;
	chipcregs_t *cc;

	sii = SI_INFO(sih);

	if (!(sih->cccaps & CC_CAP_UARTGPIO))
		return;

	
	INTR_OFF(sii, intr_val);

	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);
	ASSERT(cc != NULL);

#ifdef HTC_KlocWork
		if(cc == NULL) {
			SI_ERROR(("[HTCKW] si_btcgpiowar: cc is NULL-1\n"));
			return;
		}
#endif

	W_REG(sii->osh, &cc->uart0mcr, R_REG(sii->osh, &cc->uart0mcr) | 0x04);

	
	si_setcoreidx(sih, origidx);

	INTR_RESTORE(sii, intr_val);
}

void
si_chipcontrl_btshd0_4331(si_t *sih, bool on)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;
	uint32 val;
	uint intr_val = 0;

	sii = SI_INFO(sih);

	INTR_OFF(sii, intr_val);

	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	val = R_REG(sii->osh, &cc->chipcontrol);

	
	if (on) {
		
		val |= (CCTRL4331_BT_SHD0_ON_GPIO4);
		W_REG(sii->osh, &cc->chipcontrol, val);
	} else {
		val &= ~(CCTRL4331_BT_SHD0_ON_GPIO4);
		W_REG(sii->osh, &cc->chipcontrol, val);
	}

	
	si_setcoreidx(sih, origidx);

	INTR_RESTORE(sii, intr_val);
}

void
si_chipcontrl_restore(si_t *sih, uint32 val)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);
	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);
	W_REG(sii->osh, &cc->chipcontrol, val);
	si_setcoreidx(sih, origidx);
}

uint32
si_chipcontrl_read(si_t *sih)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;
	uint32 val;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);
	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);
	val = R_REG(sii->osh, &cc->chipcontrol);
	si_setcoreidx(sih, origidx);
	return val;
}

void
si_chipcontrl_epa4331(si_t *sih, bool on)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;
	uint32 val;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	val = R_REG(sii->osh, &cc->chipcontrol);

	if (on) {
		if (sih->chippkg == 9 || sih->chippkg == 0xb) {
			val |= (CCTRL4331_EXTPA_EN | CCTRL4331_EXTPA_ON_GPIO2_5);
			
			W_REG(sii->osh, &cc->chipcontrol, val);
		} else {
			
			if (sih->chiprev > 0) {
				W_REG(sii->osh, &cc->chipcontrol, val |
				      (CCTRL4331_EXTPA_EN) | (CCTRL4331_EXTPA_EN2));
			} else {
				W_REG(sii->osh, &cc->chipcontrol, val | (CCTRL4331_EXTPA_EN));
			}
		}
	} else {
		val &= ~(CCTRL4331_EXTPA_EN | CCTRL4331_EXTPA_EN2 | CCTRL4331_EXTPA_ON_GPIO2_5);
		W_REG(sii->osh, &cc->chipcontrol, val);
	}

	si_setcoreidx(sih, origidx);
}

void
si_chipcontrl_srom4360(si_t *sih, bool on)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;
	uint32 val;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	val = R_REG(sii->osh, &cc->chipcontrol);

	if (on) {
		val &= ~(CCTRL4360_SECI_MODE |
			CCTRL4360_BTSWCTRL_MODE |
			CCTRL4360_EXTRA_FEMCTRL_MODE |
			CCTRL4360_BT_LGCY_MODE |
			CCTRL4360_CORE2FEMCTRL4_ON);

		W_REG(sii->osh, &cc->chipcontrol, val);
	} else {
	}

	si_setcoreidx(sih, origidx);
}

void
si_chipcontrl_epa4331_wowl(si_t *sih, bool enter_wowl)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;
	uint32 val;
	bool sel_chip;

	sel_chip = (CHIPID(sih->chip) == BCM4331_CHIP_ID) ||
		(CHIPID(sih->chip) == BCM43431_CHIP_ID);
	sel_chip &= ((sih->chippkg == 9 || sih->chippkg == 0xb));

	if (!sel_chip)
		return;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	val = R_REG(sii->osh, &cc->chipcontrol);

	if (enter_wowl) {
		val |= CCTRL4331_EXTPA_EN;
		W_REG(sii->osh, &cc->chipcontrol, val);
	} else {
		val |= (CCTRL4331_EXTPA_EN | CCTRL4331_EXTPA_ON_GPIO2_5);
		W_REG(sii->osh, &cc->chipcontrol, val);
	}
	si_setcoreidx(sih, origidx);
}

uint
si_pll_reset(si_t *sih)
{
	uint err = 0;

	return (err);
}

void
si_epa_4313war(si_t *sih)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	
	W_REG(sii->osh, &cc->gpiocontrol,
		R_REG(sii->osh, &cc->gpiocontrol) | GPIO_CTRL_EPA_EN_MASK);

	si_setcoreidx(sih, origidx);
}

void
si_clk_pmu_htavail_set(si_t *sih, bool set_clear)
{
}

void
si_btcombo_p250_4313_war(si_t *sih)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);
	W_REG(sii->osh, &cc->gpiocontrol,
		R_REG(sii->osh, &cc->gpiocontrol) | GPIO_CTRL_5_6_EN_MASK);

	W_REG(sii->osh, &cc->gpioouten,
		R_REG(sii->osh, &cc->gpioouten) | GPIO_CTRL_5_6_EN_MASK);

	si_setcoreidx(sih, origidx);
}
void
si_btc_enable_chipcontrol(si_t *sih)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	
	W_REG(sii->osh, &cc->chipcontrol,
		R_REG(sii->osh, &cc->chipcontrol) | CC_BTCOEX_EN_MASK);

	si_setcoreidx(sih, origidx);
}
void
si_btcombo_43228_war(si_t *sih)
{
	si_info_t *sii;
	chipcregs_t *cc;
	uint origidx;

	sii = SI_INFO(sih);
	origidx = si_coreidx(sih);

	cc = (chipcregs_t *)si_setcore(sih, CC_CORE_ID, 0);

	W_REG(sii->osh, &cc->gpioouten, GPIO_CTRL_7_6_EN_MASK);
	W_REG(sii->osh, &cc->gpioout, GPIO_OUT_7_EN_MASK);

	si_setcoreidx(sih, origidx);
}

bool
si_deviceremoved(si_t *sih)
{
	uint32 w;
	si_info_t *sii;

	sii = SI_INFO(sih);

	switch (BUSTYPE(sih->bustype)) {
	case PCI_BUS:
		ASSERT(sii->osh != NULL);
		w = OSL_PCI_READ_CONFIG(sii->osh, PCI_CFG_VID, sizeof(uint32));
		if ((w & 0xFFFF) != VENDOR_BROADCOM)
			return TRUE;
		break;
	}
	return FALSE;
}

bool
si_is_sprom_available(si_t *sih)
{
	if (sih->ccrev >= 31) {
		si_info_t *sii;
		uint origidx;
		chipcregs_t *cc;
		uint32 sromctrl;

		if ((sih->cccaps & CC_CAP_SROM) == 0)
			return FALSE;

		sii = SI_INFO(sih);
		origidx = sii->curidx;
		cc = si_setcoreidx(sih, SI_CC_IDX);
#ifdef HTC_KlocWork
		if (cc == NULL) {
			SI_ERROR(("[HTCKW] si_is_sprom_available: cc is NULL-1\n"));
			return FALSE;
		}
#endif
		sromctrl = R_REG(sii->osh, &cc->sromcontrol);
		si_setcoreidx(sih, origidx);
		return (sromctrl & SRC_PRESENT);
	}

	switch (CHIPID(sih->chip)) {
	case BCM4312_CHIP_ID:
		return ((sih->chipst & CST4312_SPROM_OTP_SEL_MASK) != CST4312_OTP_SEL);
	case BCM4325_CHIP_ID:
		return (sih->chipst & CST4325_SPROM_SEL) != 0;
	case BCM4322_CHIP_ID:	case BCM43221_CHIP_ID:	case BCM43231_CHIP_ID:
	case BCM43222_CHIP_ID:	case BCM43111_CHIP_ID:	case BCM43112_CHIP_ID:
	case BCM4342_CHIP_ID: {
		uint32 spromotp;
		spromotp = (sih->chipst & CST4322_SPROM_OTP_SEL_MASK) >>
		        CST4322_SPROM_OTP_SEL_SHIFT;
		return (spromotp & CST4322_SPROM_PRESENT) != 0;
	}
	case BCM4329_CHIP_ID:
		return (sih->chipst & CST4329_SPROM_SEL) != 0;
	case BCM4315_CHIP_ID:
		return (sih->chipst & CST4315_SPROM_SEL) != 0;
	case BCM4319_CHIP_ID:
		return (sih->chipst & CST4319_SPROM_SEL) != 0;
	case BCM4336_CHIP_ID:
	case BCM43362_CHIP_ID:
		return (sih->chipst & CST4336_SPROM_PRESENT) != 0;
	case BCM4330_CHIP_ID:
		return (sih->chipst & CST4330_SPROM_PRESENT) != 0;
	case BCM4313_CHIP_ID:
		return (sih->chipst & CST4313_SPROM_PRESENT) != 0;
	case BCM4331_CHIP_ID:
	case BCM43431_CHIP_ID:
		return (sih->chipst & CST4331_SPROM_PRESENT) != 0;
	case BCM43239_CHIP_ID:
		return ((sih->chipst & CST43239_SPROM_MASK) &&
			!(sih->chipst & CST43239_SFLASH_MASK));
	case BCM4324_CHIP_ID:
	case BCM43242_CHIP_ID:
		return ((sih->chipst & CST4324_SPROM_MASK) &&
			!(sih->chipst & CST4324_SFLASH_MASK));
	case BCM4335_CHIP_ID:
		return ((sih->chipst & CST4335_SPROM_MASK) &&
			!(sih->chipst & CST4335_SFLASH_MASK));
	case BCM43131_CHIP_ID:
	case BCM43217_CHIP_ID:
	case BCM43227_CHIP_ID:
	case BCM43228_CHIP_ID:
	case BCM43428_CHIP_ID:
		return (sih->chipst & CST43228_OTP_PRESENT) != CST43228_OTP_PRESENT;
	default:
		return TRUE;
	}
}

uint32 si_get_sromctl(si_t *sih)
{
	chipcregs_t *cc;
	uint origidx;
	uint32 sromctl;
	osl_t *osh;

	osh = si_osh(sih);
	origidx = si_coreidx(sih);
	cc = si_setcoreidx(sih, SI_CC_IDX);
	ASSERT((uintptr)cc);

	sromctl = R_REG(osh, &cc->sromcontrol);

	
	si_setcoreidx(sih, origidx);
	return sromctl;
}

int si_set_sromctl(si_t *sih, uint32 value)
{
	chipcregs_t *cc;
	uint origidx;
	osl_t *osh;

	osh = si_osh(sih);
	origidx = si_coreidx(sih);
	cc = si_setcoreidx(sih, SI_CC_IDX);
	ASSERT((uintptr)cc);

	
	if (si_corerev(sih) < 32)
		return BCME_UNSUPPORTED;

	W_REG(osh, &cc->sromcontrol, value);

	
	si_setcoreidx(sih, origidx);
	return BCME_OK;

}
