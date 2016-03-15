/*
 *  VoodooIntel3945.cpp
 *  VoodooIntel3945
 *
 *  Created by Prashant Vaibhav on 31/08/09.
 *  Copyright 2009 Prashant Vaibhav. All rights reserved.
 *
 */

#include "VoodooIntel3945.h"
#include "if_wpireg.h"
#include "Firmware.h"
#include <sys/sysctl.h>

#pragma mark Defines
#define DPRINTF(x)	{ if (wpi_debug != 0) { printf("VoodooIntel3945: "); printf x; } }
#define DPRINTFN(n, x)	{ if (wpi_debug & n) { printf("VoodooIntel3945: "); printf x; } }
#define	WPI_DEBUG_SET	(wpi_debug != 0)
#define WPI_READ(a)	(*((uint32_t*) (m_Registers + (a))))
#define WPI_WRITE(a, d)	(*((uint32_t*) (m_Registers + (a)))) = (d)
#define abs(x)		(((x) < 0) ? (0-(x)) : (x))

enum {
	WPI_DEBUG_UNUSED	= 0x00000001,   /* Unused */
	WPI_DEBUG_HW		= 0x00000002,   /* Stage 1 (eeprom) debugging */
	WPI_DEBUG_TX		= 0x00000004,   /* Stage 2 TX intrp debugging*/
	WPI_DEBUG_RX		= 0x00000008,   /* Stage 2 RX intrp debugging */
	WPI_DEBUG_CMD		= 0x00000010,   /* Stage 2 CMD intrp debugging*/
	WPI_DEBUG_FIRMWARE	= 0x00000020,   /* firmware(9) loading debug  */
	WPI_DEBUG_DMA		= 0x00000040,   /* DMA (de)allocations/syncs  */
	WPI_DEBUG_SCANNING	= 0x00000080,   /* Stage 2 Scanning debugging */
	WPI_DEBUG_NOTIFY	= 0x00000100,   /* State 2 Noftif intr debug */
	WPI_DEBUG_TEMP		= 0x00000200,   /* TXPower/Temp Calibration */
	WPI_DEBUG_OPS		= 0x00000400,   /* wpi_ops taskq debug */
	WPI_DEBUG_WATCHDOG	= 0x00000800,   /* Watch dog debug */
	WPI_DEBUG_ANY		= 0xffffffff
};

static unsigned int wpi_debug =	0;
SYSCTL_UINT(_debug, OID_AUTO, wpi, CTLFLAG_RW, &wpi_debug, 0, "VoodooIntel3945 wpi driver debug output level");
#define org_voodoo_wireless_debug 3
static const uint8_t wpi_ridx_to_plcp[] = {
	/* OFDM: IEEE Std 802.11a-1999, pp. 14 Table 80 */
	/* R1-R4 */
	0xd, 0xf, 0x5, 0x7, 0x9, 0xb, 0x1, 0x3,
	/* CCK: device-dependent */
	10, 20, 55, 110
};

static const uint8_t wpi_ridx_to_rate[] = {
	12, 18, 24, 36, 48, 72, 96, 108, /* OFDM */
	2, 4, 11, 22 /*CCK */
};

static const uint8_t broadcast_bssid[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

/* First of all, implement RTTI methods required by I/OKit -- */
OSDefineMetaClassAndStructors(VoodooIntel3945, VoodooWirelessDevice)

#pragma mark -
#pragma mark Subclassed functions
IOReturn MyClass::allocateResources
( IOService* provider )
{
	int i;
	
	/* First of all setup debug logging control knob */
	sysctl_register_oid(&sysctl__debug_wpi);
	
	DBG(dbgInfo, "Voodoo Intel 3945 driver version 0.4 starting up...\n");
	
	/* Set up our PCI nub */
	m_PciDevice = OSDynamicCast(IOPCIDevice, provider);
	if (!m_PciDevice) {
		DBG(dbgFatal, "PCI device cast failed.\n");
		goto fail;
	}
	
	m_PciDevice->retain();
	m_PciDevice->open(this);
	
	/* Request D0 power state */
	if (m_PciDevice->requestPowerDomainState(kIOPMPowerOn,
						 (IOPowerConnection *) getParentEntry(gIOPowerPlane),
						 IOPMLowestState) != IOPMNoErr)
	{
		DBG(dbgFatal, "Could not put PCI device in power state D0.\n");
		goto fail;
	}
	
	m_PciDevice->configWrite8(0x41, 0); 	/* This comes from FreeBSD driver */
	m_PciDevice->setBusMasterEnable(true);	/* The adapter uses bus mastering, so turn it on */
	
	/* Map the device memory at BAR0 and get a pointer to it */
	m_DeviceMap = m_PciDevice->mapDeviceMemoryWithRegister(kIOPCIConfigBaseAddress0);
	if (!m_DeviceMap) {
		DBG(dbgFatal, "PCI device memory could not be mapped.\n");
		goto fail;
	}
	DBG(dbgInfo, "PCI device memory at VMaddr 0x%x, size %u\n",
	    m_DeviceMap->getVirtualAddress(), m_DeviceMap->getSize());
	m_Registers		= reinterpret_cast<uint8_t*> (m_DeviceMap->getVirtualAddress());
	
	/* Allocate hardware interrupt source */
	m_WorkLoop		= OSDynamicCast(IO80211WorkLoop, getWorkLoop());
	m_WorkLoop->retain();
	m_InterruptSrc		= IOInterruptEventSource::
				  interruptEventSource(this,
						       OSMemberFunctionCast(IOInterruptEventAction, this,
									    &MyClass::interruptOccurred),
						       m_PciDevice);
	if (!m_InterruptSrc) {
		DBG(dbgFatal, "Could not create an interrupt event source.\n");
		goto fail;
	}
	if (m_WorkLoop->addEventSource(m_InterruptSrc) != kIOReturnSuccess) {
		DBG(dbgFatal, "Could not add interrupt event source to the workloop.\n");
		goto fail;
	}
	m_InterruptSrc->enable(); // Enable immediately in case the HW interrupt is shared
	DBG(dbgInfo, "Interrupt was enabled.\n");
	
	/* Allocate timers */
	m_Timer			= IOTimerEventSource::
				  timerEventSource(this, OSMemberFunctionCast(IOTimerEventSource::Action, this, &MyClass::timerOccurred));
	if (!m_Timer) {
		DBG(dbgFatal, "Could not create a TIMER event source.\n");
		goto fail;
	}
	if (m_WorkLoop->addEventSource(m_Timer) != kIOReturnSuccess) {
		DBG(dbgFatal, "Could not add timer event source to the workloop.\n");
		goto fail;
	}
	m_Timer->setTimeoutMS(10000);
	m_Timer->enable();
	
	/* Allocate firmware memory */
	m_Firmware		= IOBufferMemoryDescriptor::
				  inTaskWithPhysicalMask(kernel_task,
							 kIODirectionInOut | kIOMemoryPhysicallyContiguous,
							 WPI_FW_MAIN_TEXT_MAXSZ + WPI_FW_MAIN_DATA_MAXSZ,
							 0x00000000FFFFFFFFull /* 32bit, 1 byte alignment */);
	if (!m_Firmware) {
		DBG(dbgFatal, "Could not allocate firmware memory.\n");
		goto fail;
	}
	m_Firmware->prepare();
	
	/* Initialize the mbuf cursor for tx */
	m_MbufCursor		= IOMbufLittleMemoryCursor::withSpecification(WPI_MAX_SEG_LEN, WPI_MAX_SCATTER - 1);
	if (!m_MbufCursor) {
		DBG(dbgFatal, "Could not allocate mbuf memory cursor\n");
		goto fail;
	}
	
	/* Put the adapter in an initial state */
	if (resetAdapter() != kIOReturnSuccess) {
		DBG(dbgFatal, "Could not reset adapter.\n");
		goto fail;
	}
	
	/* Print hardware revision info */
	memLock();
	{
	uint32_t tmp = memRead(WPI_MEM_PCIDEV);
	DBG(dbgInfo, "Hardware Revision (0x%X)\n", tmp);
	}
	memUnlock();
	
	/*
	 * ------------------------- Allocate Rings ---------------------
	 */
	
	/* Allocate shared page */
	m_SharedPage		=	IOBufferMemoryDescriptor::
					inTaskWithPhysicalMask(kernel_task,
							       kIOMemoryPhysicallyContiguous,
							       sizeof(wpi_shared),
							       0x00000000fffff000ull);
	if (!m_SharedPage) {
		DBG(dbgFatal, "Could not allocate shared page.\n");
		goto fail;
	}
	m_SharedPage->prepare();
	m_SharedPagePtr = (wpi_shared*) m_SharedPage->getBytesNoCopy();
	
	
	
	/* Allocate rx ring */
	m_RxRingMemory		= allocDmaMemory(WPI_RX_RING_COUNT * sizeof(uint32_t),
						 WPI_RING_DMA_ALIGN,
						 (void**) &m_RxRing.rx_pkt_ptr,
						 &m_RxRing.physAdd);
	if (!m_RxRingMemory) {
		DBG(dbgFatal, "Could not allocate Rx ring memory page.\n");
		goto fail;
	}
	
	for (i = 0; i < WPI_RX_RING_COUNT; i++) {
		m_RxRing.mbufs[i] = 0; // initilize to 0 so we can check if allocatePacket failed
		m_RxRing.mbufs[i] = allocatePacket(MCLBYTES); // allocate a packet into which HW will write rx'd packet
		if (m_RxRing.mbufs[i] == 0) {
			DBG(dbgFatal, "Could not allocate packet no. %d during Rx ring allocation!\n", i);
			goto fail;
		}
		m_RxRing.rx_pkt_ptr[i] = mbuf_data_to_physical(mbuf_data(m_RxRing.mbufs[i])); // tell HW where it is
	}
	DBG(dbgInfo, "Rx ring allocated successfully, PHaddr = 0x%x\n", m_RxRing.physAdd);
	
	
	
	/* Allocate Tx and cmd rings (rings 0-3 and ring 4) */
	TxRing* ring; int ring_item_count;
	for (i = 0; i <= 4; i++) {
		if (i < 4) {
			ring = &m_TxRing[i];
			ring_item_count = WPI_TX_RING_COUNT;
		} else {
			ring = &m_CmdRing;
			ring_item_count = WPI_CMD_RING_COUNT;
		}
		
		/*
		 * Here we have to use a dirty hack to make sure it is 16kB aligned. OS X seems to have trouble
		 * aligning to such large boundary, so we allocate unaligned physcont memory, and allocate extra
		 * 16 KB. Then we manually align it by zeroing out the lower bits.
		 */
		ring->descMemory	=	0;
		ring->descMemory	=	allocDmaMemory(ring_item_count * sizeof(wpi_tx_desc),
							       WPI_RING_DMA_ALIGN,
							       (void**) &ring->descriptors,
							       &m_SharedPagePtr->txbase[i]);
		
		if (ring->descMemory == 0) {
			DBG(dbgFatal, "Couldn't allocate desc memory for Tx ring %u\n", i);
			goto fail;
		}
		
		DBG(dbgInfo, "Tx ring %u, VMaddr = 0x%x, PHaddr = 0x%x\n",
		    i, ring->descriptors, m_SharedPagePtr->txbase[i]);
		
		ring->cmdMemory		=	0;
		ring->cmdMemory		=	allocDmaMemory(ring_item_count * sizeof(wpi_tx_cmd),
							       WPI_RING_DMA_ALIGN,
							       (void**) &ring->cmdSlots,
							       &ring->cmdPhysAdd);
		if (ring->cmdMemory == 0) {
			DBG(dbgFatal, "Couldn't allocate cmd memory for Tx ring %u\n", i);
			goto fail;
		}
		ring->cmdMemory->prepare();
		ring->cmdSlots = (wpi_tx_cmd*) ring->cmdMemory->getBytesNoCopy();
		ring->cmdPhysAdd = ring->cmdMemory->getPhysicalAddress();
		
		/* Initialize mbuf array to all zeros */
		for (int mi = 0; mi < ring_item_count; mi++)
			ring->mbufs[mi] = 0;
		
		DBG(dbgInfo, "Tx cmd  %u, VMaddr = 0x%x, PHaddr = 0x%x\n", i, ring->cmdSlots, ring->cmdPhysAdd);
		
		ring->count	= ring_item_count;
		ring->qid	= i;
		ring->current	= 0;
		ring->queued	= 0;
	}
	return kIOReturnSuccess;
	
fail:
	return kIOReturnError;
}


IOReturn MyClass::turnPowerOn( )
{
	uint32_t tmp;
	int ntries, qid;
	
	turnPowerOff();
	resetAdapter();
	
	memLock();
	memWrite(WPI_MEM_CLOCK1, 0xa00);
	IODelay(20);
	tmp = memRead(WPI_MEM_PCIDEV);
	memWrite(WPI_MEM_PCIDEV, tmp | 0x800);
	memUnlock();
	
	powerUp();
	configureHardware();
	
	/* init Rx ring */
	memLock();
	WPI_WRITE(WPI_RX_BASE, m_RxRing.physAdd);
	WPI_WRITE(WPI_RX_RIDX_PTR, m_SharedPage->getPhysicalAddress() + offsetof(wpi_shared, next));
	WPI_WRITE(WPI_RX_WIDX, (WPI_RX_RING_COUNT - 1) & ~7);
	WPI_WRITE(WPI_RX_CONFIG, 0xa9601010);
	memUnlock();
	
	/* init Tx rings */
	memLock();
	memWrite(WPI_MEM_MODE, 2); /* bypass mode */
	memWrite(WPI_MEM_RA, 1);   /* enable RA0 */
	memWrite(WPI_MEM_TXCFG, 0x3f); /* enable all 6 Tx rings */
	memWrite(WPI_MEM_BYPASS1, 0x10000);
	memWrite(WPI_MEM_BYPASS2, 0x30002);
	memWrite(WPI_MEM_MAGIC4, 4);
	memWrite(WPI_MEM_MAGIC5, 5);
	
	WPI_WRITE(WPI_TX_BASE_PTR, m_SharedPage->getPhysicalAddress());
	WPI_WRITE(WPI_MSG_CONFIG, 0xffff05a5);
	
	for (qid = 0; qid < 6; qid++) {
		WPI_WRITE(WPI_TX_CTL(qid), 0);
		WPI_WRITE(WPI_TX_BASE(qid), 0);
		WPI_WRITE(WPI_TX_CONFIG(qid), 0x80200008);
	}
	memUnlock();
	
	/* clear "radio off" and "disable command" bits (reversed logic) */
	WPI_WRITE(WPI_UCODE_CLR, WPI_RADIO_OFF);
	WPI_WRITE(WPI_UCODE_CLR, WPI_DISABLE_CMD);
	m_Flags &= ~(WPI_FLAG_HW_RADIO_OFF | WPI_FLAG_SCANNING);
	
	/* clear any pending interrupts */
	WPI_WRITE(WPI_INTR, 0xffffffff);
	
	/* enable interrupts */
	WPI_WRITE(WPI_MASK, WPI_INTR_MASK);
	
	WPI_WRITE(WPI_UCODE_CLR, WPI_RADIO_OFF);
	WPI_WRITE(WPI_UCODE_CLR, WPI_RADIO_OFF);
	
	if (uploadFirmware() != kIOReturnSuccess) {
		DBG(dbgFatal, "A problem occurred loading the firmware to the driver\n");
		return kIOReturnError;
	}
	
	/* At this point the firmware is up and running. If the hardware
	 * RF switch is turned off thermal calibration will fail, though
	 * the card is still happy to continue to accept commands, catch
	 * this case and fail if radio is off.
	 */
	memLock();
	tmp = memRead(WPI_MEM_HW_RADIO_OFF);
	memUnlock();
	
	if (!(tmp & 0x1)) {
		m_Flags |= WPI_FLAG_HW_RADIO_OFF;
		DBG(dbgFatal,"Radio Transmitter is switched off, failing power on\n");
		return kIOReturnError;
	}
	
	/* wait for thermal sensors to calibrate */
	for (ntries = 0; ntries < 1000; ntries++) {
		if ((m_Temperature = (int)WPI_READ(WPI_TEMPERATURE)) != 0)
			break;
		IODelay(10);
	}
	
	if (ntries == 1000) {
		DBG(dbgFatal, "Timeout waiting for thermal sensors calibration\n");
		return kIOReturnTimeout;
	}
	DPRINTFN(WPI_DEBUG_TEMP,("Temperature %d\n", m_Temperature));
	
	
	/* Set the default power-up channel before configuring */
	m_CurrentChannel.number	= 11; // XXX
	m_CurrentChannel.flags	= IEEE::Channel::default11BGChannelFlags;
	
	if (configure() != kIOReturnSuccess) {
		DBG(dbgFatal, "Device config failed\n");
		return kIOReturnError;
	}
	
	m_AssocState = staInit;
	
	return kIOReturnSuccess;
}


IOReturn MyClass::turnPowerOff( )
{
	uint32_t tmp;
	int ac;

	m_Flags &= ~(WPI_FLAG_HW_RADIO_OFF | WPI_FLAG_SCANNING);
	// TODO: watchdog and calibration timer reset
	
	/* disable interrupts */
	WPI_WRITE(WPI_MASK, 0);
	WPI_WRITE(WPI_INTR, WPI_INTR_MASK);
	WPI_WRITE(WPI_INTR_STATUS, 0xff);
	WPI_WRITE(WPI_INTR_STATUS, 0x00070000);
	
	memLock();
	memWrite(WPI_MEM_MODE, 0);
	memUnlock();
	
	/* reset all Tx rings */
	for (ac = 0; ac < 4; ac++)
		resetTxRing(&m_TxRing[ac]);
	resetTxRing(&m_CmdRing);
	
	/* reset Rx ring */
	resetRxRing(&m_RxRing);
	
	memLock();
	memWrite(WPI_MEM_CLOCK2, 0x200);
	memUnlock();
	
	IODelay(5);
	
	stopMaster();
	
	tmp = WPI_READ(WPI_RESET);
	WPI_WRITE(WPI_RESET, tmp | WPI_SW_RESET);
	m_Flags &= ~(WPI_FLAG_BUSY);
	
	return kIOReturnSuccess;
}


void MyClass::freeResources
( IOService* provider )
{
	int i;
	
	sysctl_unregister_oid(&sysctl__debug_wpi);
	
	if (m_Timer) m_Timer->disable();
	RELEASE(m_Timer);
	
	if (m_Firmware) m_Firmware->complete();
	RELEASE(m_Firmware);
	
	if (m_RxRingMemory) m_RxRingMemory->complete();
	RELEASE(m_RxRingMemory);
	
	/* Free Rx mbufs if any */
	for (i = 0; i < WPI_RX_RING_COUNT; i++) {
		if (m_RxRing.mbufs[i] != 0) {
			freePacket(m_RxRing.mbufs[i]);
			m_RxRing.mbufs[i] = 0;
		}
	}
	
	/* Free tx and cmd rings */
	TxRing* ring;
	for (i = 0; i <= 4; i++) {
		if (i < 4)
			ring = &m_TxRing[i];
		else
			ring = &m_CmdRing;
		if (ring->descMemory) {
			ring->descMemory->complete();
			RELEASE(ring->descMemory);
		}
		if (ring->cmdMemory) {
			ring->cmdMemory->complete();
			RELEASE(ring->cmdMemory);
		}
	}
	
	if (m_InterruptSrc) m_InterruptSrc->disable();
	RELEASE(m_InterruptSrc);
	
	if (m_DeviceMap) m_DeviceMap->unmap();
	RELEASE(m_DeviceMap);
	
	if (m_PciDevice) m_PciDevice->close(this);
	RELEASE(m_PciDevice);
	
	return;
}


IOReturn MyClass::startScan
( const ScanParameters* params, const IEEE::ChannelList* channels )
{
	/*
	 * Tonight we dine in hell
	 */
	wpi_scan_hdr*	hdr;
	wpi_scan_chan*	chan;
	uint8_t		cmd_data[360]; // buffer to hold our scan command
	uint8_t*	frm;
	int		i;
	bool		directed = false;
	
	DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Starting a scan...\n"));
	
	IEEE::ManagementFrameHeader* wh;
	IEEE::Channel::Flags	chflags;
	
	hdr = (wpi_scan_hdr*) cmd_data;
	bzero(hdr, sizeof(wpi_scan_hdr));
	
	/*
	 * Move to the next channel if no packets are received within 5 msecs
	 * after sending the probe request (this helps to reduce the duration
	 * of active scans).
	 */
	hdr->quiet	= 10; // was 5
	hdr->threshold	= 1;
	
	DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Dwell time %u, rest time %u\n", params->dwellTime, params->restTime));
	
	switch (params->scanPhyMode) {
		case IEEE::phyModeAuto:
		case IEEE::dot11B:
		case IEEE::dot11G:
			chflags = IEEE::Channel::band2GHz;
			break;
		case IEEE::dot11A:
			chflags = IEEE::Channel::band5GHz;
			break;
		default:
			DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: phy mode %d not supported, defaulting to bg\n",
			    (int)params->scanPhyMode));
			chflags = IEEE::Channel::band2GHz;
	};
	
	if (chflags & IEEE::Channel::band5GHz) {
		DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: 5 GHz mode\n"));
		hdr->tx.rate	= wpi_ridx_to_plcp[WPI_OFDM6]; /* send probe requests at 6Mbps */
		hdr->promotion	= 1; /* Enable crc checking */
	} else {
		DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: 2.4 GHz mode\n"));
		hdr->flags	= WPI_CONFIG_24GHZ | WPI_CONFIG_AUTO;
		hdr->tx.rate	= wpi_ridx_to_plcp[WPI_CCK1]; /* send probe requests at 1Mbps */
	}
	hdr->tx.id		= WPI_ID_BROADCAST;
	hdr->tx.lifetime	= WPI_LIFETIME_INFINITE;
	hdr->tx.flags		= WPI_TX_AUTO_SEQ;
	
	bzero(hdr->scan_essids, sizeof(hdr->scan_essids));
	
	if (params->ssid)
		if (params->ssid->getLength() > 0)
			directed = true;
	
	if (directed) {
		DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Directed for SSID '%s' of length %u\n",
					      params->ssid->getBytesNoCopy(), params->ssid->getLength()));
		hdr->scan_essids[0].id		= IEEE::IE::ieSSID;
		hdr->scan_essids[0].esslen	= MIN(params->ssid->getLength(), 32);
		memcpy(hdr->scan_essids[0].essid, params->ssid->getBytesNoCopy(), hdr->scan_essids[0].esslen);
	} else {
		DPRINTFN(WPI_DEBUG_SCANNING, ( "SCAN: Broadcast mode\n"));
	}
	
	/*
	 * Build a probe request frame.
	 */
	wh = (IEEE::ManagementFrameHeader*) &hdr->scan_essids[4]; // i.e. just after the last essid
	bzero(wh, sizeof(*wh));
	
	wh->hdr.protocolVersion	= 0;
	wh->hdr.type		= IEEE::WiFiFrameHeader::ManagementFrame;
	wh->hdr.subtype		= IEEE::WiFiFrameHeader::ProbeRequest;
	
	bcopy(broadcast_bssid, wh->da, 6);
	bcopy(broadcast_bssid, wh->bssid, 6);
	bcopy(&m_MacAddress, wh->sa, 6);
	
	frm = (uint8_t*) (wh + 1);
	
	/* add essid IE, the hardware will fill this in for us */
	*frm++ = IEEE::IE::ieSSID;
	*frm++ = 0;
	
	/* add supported rates IE */
	*frm++ = IEEE::IE::ieSupportedRates;
	*frm++ = 8; // max size of this IE is 8
	/* We'll add all the CCK rates, and some OFDM rates. Remaining will be added in extra supported rates IE.
	 * As for which ones are "basic" .. I think all of them could be, but I don't know. */
	*frm++ = (uint8_t) (IEEE::rate1Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate2Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate5Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate11Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate18Mbps);
	*frm++ = (uint8_t) (IEEE::rate24Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate36Mbps);
	*frm++ = (uint8_t) (IEEE::rate54Mbps);
	
	/* add supported xrates IE */
	*frm++ = IEEE::IE::ieExtendedSupportedRates;
	*frm++ = 4; // there are 4 more OFDM rates
	*frm++ = (uint8_t) (IEEE::rate6Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate9Mbps);
	*frm++ = (uint8_t) (IEEE::rate12Mbps | IEEE::rateIsBasic);
	*frm++ = (uint8_t) (IEEE::rate48Mbps);
	
	/* setup length of probe request */
	hdr->tx.len = (frm - (uint8_t *)wh);
	DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Probe request frame length: %d\n", hdr->tx.len));
	
	/*
	 * Construct information about the channel that we
	 * want to scan. The firmware expects this to be directly
	 * after the scan probe request
	 */
	chan = (wpi_scan_chan*) frm;
	DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Told to scan %u channels, frm=0x%x\n", channels->numItems, frm));
	// we are limiting the scan to only those channels which fit in 360 bytes
	for (i = 1; i <= 11; i++) {
		hdr->nchan++;
		chan->chan	= i;
		chan->flags	= 0;
		chan->gain_dsp	= 0x6e; /* Default level */
		
		if (params->scanType == ScanParameters::scanTypeActive)
		{
			chan->flags |= WPI_CHAN_ACTIVE;
			if (directed)
				chan->flags |= WPI_CHAN_DIRECT;
		}
		chan->active	= 20;
		chan->passive	= 200; // was dwellTime
		chan->gain_radio= 0x28;
		
		chan++;
		frm += sizeof (wpi_scan_chan);
		DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Ch %u, cmd_len %u, frm=0x%x, hdr=0x%x\n", i,
					      frm - (uint8_t*)hdr, frm, hdr));
	}
	
	hdr->len	= frm - (uint8_t *)hdr;
	
	DPRINTFN(WPI_DEBUG_SCANNING, ("SCAN: Sending start command with cmd_len=%u, probe_len=%u, nchan=%u\n",
				      hdr->len, hdr->tx.len, hdr->nchan));
	
	IOReturn err = sendCommand(WPI_CMD_SCAN, cmd_data, hdr->len, 0);
	if (err == kIOReturnSuccess)
		m_Flags |= WPI_FLAG_SCANNING;
	else
		m_Flags &= ~(WPI_FLAG_SCANNING);
	return err;
}


void MyClass::abortScan( )
{
	m_Flags &= ~(WPI_FLAG_SCANNING);
	return;
}


IOReturn MyClass::associate
( const AssociationParameters* params )
{
#pragma mark Association 7-step procedure
	/*---------------------------------------------------------------------------------------------
	 In Soviet Russia, hell dines in us.
	 
	 So how this works is the following:
		1. Prepare the card for auth/assoc by doing some stuff
		2. Send auth request frame, set state to auth_in_progress
		3. Receive auth response. If success, go to 4, else send msg that auth failed
		4. Set state to assoc_in_progress. Send assoc request frame
		5. Receive assoc response. If success, go to 6, else send msg that assoc failed
		6. Store AID from assoc response, prepare card for RUN state by doing more stuff
		7. Send msg that assoc succeeded, set state to assoc_run
	 In this function, we will only perform steps 1 and 2.
	 ---------------------------------------------------------------------------------------------*/
	if (m_AssocState != staInit) {
		DBG(dbgWarning, "Assoc status not in intial state\n");
		return kIOReturnBusy;
	}
	
	/*
	 * Step 1
	 */
	wpi_node_info	node;
	DBG(dbgInfo, "Auth/association 7-step-gymnastic begins...\n");
	/* First make a copy of the association parameters as they'll be used later */
	if (m_AssocParams.ssid)
		m_AssocParams.ssid->release();
	bcopy(params, &m_AssocParams, sizeof(m_AssocParams));
	m_AssocParams.ssid = OSData::withData(params->ssid);
	
	/* Check if we are in .11b only mode */
	bool dot11bMode = true;
	for (int i = 0; i < params->supportedRates.numItems; i++)
		if ( WPI_RATE_IS_OFDM(0x7f & params->supportedRates.rate[i]) ) {
			dot11bMode = false;
			break;
		}
	
	/* update adapter's configuration */
	bcopy(&params->bssid, m_Config.bssid, 6);
	m_Config.associd	= 0;
	m_Config.filter		&= ~(WPI_FILTER_BSS);
	m_Config.chan		= params->channel.number;
	if (params->channel.flags & IEEE::Channel::band2GHz)
		m_Config.flags	|= (WPI_CONFIG_AUTO | WPI_CONFIG_24GHZ);

	if (params->channel.flags & IEEE::Channel::band5GHz) {
		m_Config.cck_mask  = 0;
		m_Config.ofdm_mask = 0x15;
	} else if (dot11bMode) {
		m_Config.cck_mask  = 0x03;
		m_Config.ofdm_mask = 0;
	} else {
		/* XXX assume 802.11b/g */
		m_Config.cck_mask  = 0x0f;
		m_Config.ofdm_mask = 0xff;
	}
	
	DPRINTF(("config chan %d flags %x cck %x ofdm %x\n",
		 m_Config.chan, m_Config.flags, m_Config.cck_mask, m_Config.ofdm_mask));
	
	if (sendCommand(WPI_CMD_CONFIGURE, &m_Config, sizeof (wpi_config), 1) != kIOReturnSuccess) {
		DBG(dbgWarning, "Couldn't configure during auth/assoc\n");
		return kIOReturnError;
	}
	
	/* configuration has changed, set Tx power accordingly */
	if (setTxPower(params->channel, 1) != kIOReturnSuccess) {
		DBG(dbgWarning, "Couldn't set TX power during auth/assoc\n");
		return kIOReturnError;
	}
	
	/* add default node */
	bzero(&node, sizeof node);
	bcopy(&params->bssid, node.bssid, 6);
	node.id		= WPI_ID_BSS;
	node.rate	= (params->channel.flags & IEEE::Channel::band5GHz) ? plcpSignal(12) : plcpSignal(2);
	node.action	= WPI_ACTION_SET_RATE;
	node.antenna	= WPI_ANTENNA_BOTH;
	if (sendCommand(WPI_CMD_ADD_NODE, &node, sizeof node, 1) != kIOReturnSuccess) {
		DBG(dbgWarning, "Couldn't add node during auth/assoc\n");
		return kIOReturnError;
	}
	DBG(dbgWarning, "ASSOC: Step 1 done\n");
	
	IOSleep(50); // hack
	
	/*
	 * Step 2
	 */
	m_AssocState = staAuthTry;
	AuthenticationFrame af;
	bzero(&af, sizeof(af));
	
	af.hdr.hdr.type	= IEEE::WiFiFrameHeader::ManagementFrame;
	af.hdr.hdr.subtype = IEEE::WiFiFrameHeader::Authentication;
	
	bcopy(&m_MacAddress, af.hdr.sa, 6);
	bcopy(broadcast_bssid, af.hdr.bssid, 6);
	bcopy(&params->bssid, af.hdr.da, 6);
	
	af.algorithm	= 0; // open
	af.sequence	= 1; // request
	
	DBG(dbgWarning, "ASSOC: Step 2 (almost) done\n");
	return sendManagementFrame((uint8_t*) &af, sizeof(af));
}

IOReturn MyClass::disassociate( )
{
	return kIOReturnSuccess;
}


void MyClass::getHardwareInfo
( HardwareInfo* info )
{
	info->manufacturer	= OSString::withCString("Intel");
	info->model		= OSString::withCString("PRO/Wireless 3945ABG");
	info->hardwareRevision	= OSString::withCString("1");
	info->driverVersion	= OSString::withCString("0.4");
	info->firmwareVersion	= OSString::withCString("15.32.2.9");
	
	readPromData(WPI_EEPROM_MAC, &info->hardwareAddress, 6);
	readPromData(WPI_EEPROM_MAC, &m_MacAddress, 6); // for our own use
	
	
	if (m_SupportsA)
		info->supportedModes = (IEEE::PHYModes) (IEEE::dot11B | IEEE::dot11G | IEEE::dot11A);
	else
		info->supportedModes = (IEEE::PHYModes) (IEEE::dot11B | IEEE::dot11G);
	
	/*
	 * Read the EEPROM to get the channels we support
	 * Adapted from wpi_read_eeprom_channels()
	 */
	const wpi_chan_band*	band;
	wpi_eeprom_chan		channels[WPI_MAX_CHAN_PER_BAND];
	IEEE::Channel*		c;
	int			chan, i, n;
	info->supportedChannels.numItems = 0;
	
	for (n = 0; n < WPI_CHAN_BANDS_COUNT; n++) {
		band = &wpi_bands[n];
		bzero(channels, sizeof(channels));
		readPromData(band->addr, channels, band->nchan * sizeof(wpi_eeprom_chan));
		
		for (i = 0; i < band->nchan; i++) {
			if (!(channels[i].flags & WPI_EEPROM_CHAN_VALID)) {
				DPRINTFN(WPI_DEBUG_HW, ("Channel Not Valid: %d, band %d\n", band->chan[i],n));
				continue;
			}
			
			chan	= band->chan[i];
			c	= &info->supportedChannels.channel[info->supportedChannels.numItems++];
			
			if (n == 0) {	/* 2GHz band */
				c->number	= chan;
				c->flags	= IEEE::Channel::default11BGChannelFlags;
			} else {	/* 5GHz band */
				c->number	= chan;
				c->flags	= IEEE::Channel::default11AChannelFlags;
			}
			
			if (!(channels[i].flags & WPI_EEPROM_CHAN_ACTIVE)) {
				/* Active scanning not supported, turn off active scanning flag */
				c->flags &= ~(IEEE::Channel::supportsActiveScanning);
			}
			
			/* save maximum allowed power for this channel */
			m_MaxPower[chan] = channels[i].maxpwr;
			
			DPRINTF(("adding chan %d flags=0x%x maxpwr=%d active=%s, offset %d\n",
				 chan, channels[i].flags, m_MaxPower[chan],
				 (c->flags & IEEE::Channel::supportsActiveScanning) ? "true" : "false",
				 info->supportedChannels.numItems));
		}
	}
	
	DBG(dbgInfo, "Total channels supported = %u\n", info->supportedChannels.numItems);
	
	/*
	 * Now read the EEPROM for Tx power groups. Adapted from wpi_read_eeprom_group().
	 * This data is used internally only.
	 */
	wpi_power_group*	group;
	wpi_eeprom_group	rgroup;
	
	for (n = 0; n < WPI_POWER_GROUPS_COUNT; n++) {
		group = &m_PowerGroups[n];
		readPromData(WPI_EEPROM_POWER_GRP + n * 32, &rgroup, sizeof rgroup);
		
		/* save power group information */
		group->chan   = rgroup.chan;
		group->maxpwr = rgroup.maxpwr;
		/* temperature at which the samples were taken */
		group->temp   = (int16_t) (rgroup.temp);
		
		DPRINTF(("power group %d: chan=%d maxpwr=%d temp=%d\n", n, group->chan, group->maxpwr, group->temp));
		
		for (i = 0; i < WPI_SAMPLES_COUNT; i++) {
			group->samples[i].index = rgroup.samples[i].index;
			group->samples[i].power = rgroup.samples[i].power;
			
			DPRINTF(("\tsample %d: index=%d power=%d\n",
				 i, group->samples[i].index, group->samples[i].power));
		}
	}
	
	/* XXX: Are all rates "basic" ? */
	info->supportedRates.numItems	= 12;
	info->supportedRates.rate[0]	= (IEEE::DataRate) (IEEE::rate1Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[1]	= (IEEE::DataRate) (IEEE::rate2Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[2]	= (IEEE::DataRate) (IEEE::rate5Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[3]	= (IEEE::DataRate) (IEEE::rate11Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[4]	= (IEEE::DataRate) (IEEE::rate6Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[5]	= (IEEE::DataRate) (IEEE::rate9Mbps);
	info->supportedRates.rate[6]	= (IEEE::DataRate) (IEEE::rate12Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[7]	= (IEEE::DataRate) (IEEE::rate18Mbps);
	info->supportedRates.rate[8]	= (IEEE::DataRate) (IEEE::rate24Mbps	|  IEEE::rateIsBasic);
	info->supportedRates.rate[9]	= (IEEE::DataRate) (IEEE::rate36Mbps);
	info->supportedRates.rate[10]	= (IEEE::DataRate) (IEEE::rate48Mbps);
	info->supportedRates.rate[11]	= (IEEE::DataRate) (IEEE::rate54Mbps);
	
	info->maxTxPower	= 15; // FIXME: find and put proper value here
	info->snrUnit		= HardwareInfo::unit_dBm;
	info->powerSavingModes	= powerSaveAlwaysOn;
	info->maxPacketSize	= 2300;
	info->txQueueSize	= WPI_TX_RING_COUNT * 4;
	
	info->capabilities.ShortSlot	 = true;
	info->capabilities.ShortPreamble = true;
	
	/* Read some EEPROM values for our own use */
	readPromData(WPI_EEPROM_CAPABILITIES,	&m_HWCap, 1);
	readPromData(WPI_EEPROM_REVISION,	&m_HWRev, 2);
	readPromData(WPI_EEPROM_TYPE,		&m_HWType,1);
}


IOReturn MyClass::getConfiguration
( HardwareConfigType type, void* param )
{
	return kIOReturnUnsupported;
}


IOReturn MyClass::setConfiguration
( HardwareConfigType type, void* param )
{
	return kIOReturnUnsupported;
}

IOReturn MyClass::outputFrame
( TxFrameHeader hdr, mbuf_t m )
{
	TxRing*		ring = &m_TxRing[0];
	wpi_tx_desc*	desc;
	wpi_tx_cmd*	cmd;
	wpi_cmd_data*	tx;
	IEEE::TxDataFrameHeader* wh;
	int i, nsegs, rate, hdrlen, ismcast;
	
	/* Prevent sending data while hardware is busy or scan is in progress 
	if (m_Flags & WPI_FLAG_BUSY || m_Flags & WPI_FLAG_SCANNING) {
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	*/
	
	if (ring->queued > ring->count - 8) {
		DBG(dbgWarning, "Packet dropped, queue full\n");
		freePacket(m);
		return kIOReturnOutputDropped;
	}
	
	wh	= (IEEE::TxDataFrameHeader*) mbuf_data(m);
	if (wh->hdr.type == IEEE::WiFiFrameHeader::ManagementFrame)
		ring = &m_TxRing[0];
	
	desc	= &ring->descriptors[ring->current];
	
	hdrlen	= sizeof(IEEE::TxDataFrameHeader);
	ismcast = wh->bssid[0] & 0x01;
	
	cmd		= &ring->cmdSlots[ring->current];
	cmd->code	= WPI_CMD_TX_DATA;
	cmd->flags	= 0;
	cmd->qid	= ring->qid;
	cmd->idx	= ring->current;
	
	tx		= (wpi_cmd_data*) cmd->data;
	tx->flags	= WPI_TX_AUTO_SEQ;
	tx->timeout	= 0;
	tx->ofdm_mask	= 0xff;
	tx->cck_mask	= 0x0f;
	tx->lifetime	= WPI_LIFETIME_INFINITE;
	tx->id		= ismcast ? WPI_ID_BROADCAST : WPI_ID_BSS;
	tx->len		= mbuf_len(m);
	
	if (!ismcast) {
		tx->flags |= (WPI_TX_NEED_ACK);
		if (mbuf_len(m) > /* RTS threshold */ 2346) {
			tx->flags |= (WPI_TX_NEED_RTS|WPI_TX_FULL_TXOP);
			tx->rts_ntries = 7;
		}
	}
	
	/* pick a rate */
	if (wh->hdr.type == IEEE::WiFiFrameHeader::ManagementFrame) {
		if (wh->hdr.subtype == IEEE::WiFiFrameHeader::AssocRequest ||
		    wh->hdr.subtype == IEEE::WiFiFrameHeader::ReassocRequest)
			tx->timeout = 3;
		else
			tx->timeout = 2;
		rate = IEEE::rate1Mbps;
	} else if (ismcast) {
		rate = IEEE::rate1Mbps;
	} else {
		ieee80211_amrr_choose(&amrr, &amrrNode, &m_TxRateIndex, &m_AssocParams.supportedRates);
		rate = (0x7f & m_AssocParams.supportedRates.rate[m_TxRateIndex]);
		DPRINTFN(WPI_DEBUG_TX, ("Choosing Tx rate %u Mbps (index %u)\n", rate/2, m_TxRateIndex));
	}
	tx->rate = plcpSignal(rate);
	
	/* be very persistant at sending frames out */
	tx->data_ntries = 15;
	
	/* save and trim IEEE802.11 header */
	mbuf_copydata(m, 0, sizeof(IEEE::TxDataFrameHeader), &tx->wh);
	mbuf_adj(m, hdrlen);
	
	/* Set the actual data addresses into hw tx desc */
	IOPhysicalSegment segs[WPI_MAX_SCATTER-1];
	
	nsegs = m_MbufCursor->getPhysicalSegmentsWithCoalesce(m, segs, WPI_MAX_SCATTER-1);
	
	if (!nsegs) {
		DBG(dbgWarning, "Could not get physical segments for mbuf. Dropping packet.\n");
		if (m) freePacket(m);
		return kIOReturnOutputDropped;
	}
	
	DPRINTFN(WPI_DEBUG_TX, ("sending data: qid=%d idx=%d len=%d nsegs=%d\n",
				ring->qid, ring->current, mbuf_len(m), nsegs));
	
	/* first scatter/gather segment is used by the tx data command */
	desc->flags = (WPI_PAD32(mbuf_len(m)) << 28 | (1 + nsegs) << 24);
	desc->segs[0].addr = (ring->cmdPhysAdd + ring->current * sizeof (wpi_tx_cmd));
	desc->segs[0].len  = (4 + sizeof (wpi_cmd_data));
	for (i = 1; i <= nsegs; i++) {
		desc->segs[i].addr = segs[i - 1].location;
		desc->segs[i].len  = segs[i - 1].length;
	}
	
	ring->queued++;
	ring->mbufs[ring->current] = m;
	
	/* kick ring */
	ring->current = (ring->current + 1) % WPI_TX_RING_COUNT;
	WPI_WRITE(WPI_TX_WIDX, ring->qid << 8 | ring->current);
	
	return kIOReturnSuccess;
}

//___________________________________________________________________________________________________ PRIVATE FUNCTIONS
#pragma mark -
#pragma mark Private HW functions

void MyClass::resetTxRing
( TxRing* ring )
{
	int ntries, i;
	
	memLock();
	
	WPI_WRITE(WPI_TX_CONFIG(ring->qid), 0);
	for (ntries = 0; ntries < 100; ntries++) {
		if (WPI_READ(WPI_TX_STATUS) & WPI_TX_IDLE(ring->qid))
			break;
		IODelay(10);
	}
	
	memUnlock();

	if (ntries == 100 && wpi_debug > 0)
		DBG(dbgWarning, "Timeout resetting Tx ring %d\n", ring->qid);
	
	for (i = 0; i < ring->count; i++) {
		if (ring->mbufs[i]) {
			freePacket(ring->mbufs[i]);
			ring->mbufs[i] = 0;
		}
	}
	
	ring->queued	= 0;
	ring->current	= 0;
}


void MyClass::resetRxRing
( RxRing* ring )
{
	int ntries;
	
	memLock();
	
	WPI_WRITE(WPI_RX_CONFIG, 0);
	for (ntries = 0; ntries < 100; ntries++) {
		if (WPI_READ(WPI_RX_STATUS) & WPI_RX_IDLE)
			break;
		IODelay(10);
	}
	
	memUnlock();
	
	if (ntries == 100 && wpi_debug > 0)
		DBG(dbgWarning, "Timeout resetting Rx ring\n");

	ring->current = 0;
}


IOReturn MyClass::uploadFirmware( )
{
	const wpi_firmware_hdr* hdr = (wpi_firmware_hdr*) Intel3945FirmwareImage;
	const uint8_t	*itext, *idata, *rtext, *rdata, *btext;
	uint32_t	itextsz, idatasz, rtextsz, rdatasz, btextsz;
	IOReturn	error = kIOReturnSuccess;
	
	DPRINTFN(WPI_DEBUG_FIRMWARE, ("Attempting Loading Firmware from wpi_fw module\n"));
	
	/*   |  RUNTIME FIRMWARE   |    INIT FIRMWARE    | BOOT FW  |
	 |HDR|<--TEXT-->|<--DATA-->|<--TEXT-->|<--DATA-->|<--TEXT-->| */
	
	rtextsz = hdr->rtextsz;
	rdatasz = hdr->rdatasz;
	itextsz = hdr->itextsz;
	idatasz = hdr->idatasz;
	btextsz = hdr->btextsz;
	
	/* check that all firmware segments are present */
	if (Intel3945FirmwareImage_len < sizeof(wpi_firmware_hdr)
	    + rtextsz + rdatasz + itextsz + idatasz + btextsz)
	{
		DBG(dbgFatal, "Firmware size incorrect!\n");
		error = kIOReturnError;
		goto fail;
	}
	
	/* get pointers to firmware segments */
	rtext = (const uint8_t *)(hdr + 1);
	rdata = rtext + rtextsz;
	itext = rdata + rdatasz;
	idata = itext + itextsz;
	btext = idata + idatasz;
	
	DPRINTFN(WPI_DEBUG_FIRMWARE,
		 ("Firmware Version: Major %d, Minor %d, Driver %d, "
		  "runtime (text: %u, data: %u) init (text: %u, data %u) boot (text %u)\n",
		  (hdr->version & 0xff000000) >> 24,
		  (hdr->version & 0x00ff0000) >> 16,
		  (hdr->version & 0x0000ffff),
		  rtextsz, rdatasz,
		  itextsz, idatasz, btextsz));
	
	DPRINTFN(WPI_DEBUG_FIRMWARE,("rtext 0x%x\n", *(const uint32_t *)rtext));
	DPRINTFN(WPI_DEBUG_FIRMWARE,("rdata 0x%x\n", *(const uint32_t *)rdata));
	DPRINTFN(WPI_DEBUG_FIRMWARE,("itext 0x%x\n", *(const uint32_t *)itext));
	DPRINTFN(WPI_DEBUG_FIRMWARE,("idata 0x%x\n", *(const uint32_t *)idata));
	DPRINTFN(WPI_DEBUG_FIRMWARE,("btext 0x%x\n", *(const uint32_t *)btext));
	
	/* sanity checks */
	if (rtextsz > WPI_FW_MAIN_TEXT_MAXSZ ||
	    rdatasz > WPI_FW_MAIN_DATA_MAXSZ ||
	    itextsz > WPI_FW_INIT_TEXT_MAXSZ ||
	    idatasz > WPI_FW_INIT_DATA_MAXSZ ||
	    btextsz > WPI_FW_BOOT_TEXT_MAXSZ ||
	    (btextsz & 3) != 0)
	{
		DBG(dbgFatal, "Firmware invalid\n");
		error = kIOReturnError;
		goto fail;
	}
	
	/* copy initialization images into pre-allocated DMA-safe memory */
	memcpy(m_Firmware->getBytesNoCopy(), idata, idatasz);
	memcpy((uint8_t*) m_Firmware->getBytesNoCopy() + WPI_FW_INIT_DATA_MAXSZ, itext, itextsz);
	
	/* tell adapter where to find initialization images */
	memLock();
	memWrite(WPI_MEM_DATA_BASE, m_Firmware->getPhysicalAddress());
	memWrite(WPI_MEM_DATA_SIZE, idatasz);
	memWrite(WPI_MEM_TEXT_BASE, m_Firmware->getPhysicalAddress() + WPI_FW_INIT_DATA_MAXSZ);
	memWrite(WPI_MEM_TEXT_SIZE, itextsz);
	memUnlock();
	
	/* load firmware boot code */
	if ((error = uploadMicrocode(btext, btextsz)) != kIOReturnSuccess) {
		DBG(dbgFatal, "Failed to load microcode\n");
		goto fail;
	}
	
	/* now press "execute" */
	WPI_WRITE(WPI_RESET, 0);
	
	/* wait at most one second for the first alive notification */
	AbsoluteTime deadline;
	clock_interval_to_deadline(1000, kMillisecondScale, reinterpret_cast<uint64_t*> (&deadline));
	if (m_WorkLoop->sleepGateDeadline(&m_FirmwareLoaded, THREAD_INTERRUPTIBLE, deadline) != THREAD_AWAKENED) {
		DBG(dbgWarning, "Firmware upload 1 timed out\n");
		error = kIOReturnTimeout;
		goto fail;
	}
	
	/* copy runtime images into pre-allocated DMA-safe memory */
	memcpy(m_Firmware->getBytesNoCopy(), rdata, rdatasz);
	memcpy((uint8_t*) m_Firmware->getBytesNoCopy() + WPI_FW_MAIN_DATA_MAXSZ, rtext, rtextsz);
	
	/* tell adapter where to find runtime images */
	memLock();
	memWrite(WPI_MEM_DATA_BASE, m_Firmware->getPhysicalAddress());
	memWrite(WPI_MEM_DATA_SIZE, rdatasz);
	memWrite(WPI_MEM_TEXT_BASE, m_Firmware->getPhysicalAddress() + WPI_FW_MAIN_DATA_MAXSZ);
	memWrite(WPI_MEM_TEXT_SIZE, WPI_FW_UPDATED | rtextsz);
	memUnlock();
	
	/* wait at most one second for the first alive notification */
	clock_interval_to_deadline(1000, kMillisecondScale, reinterpret_cast<uint64_t*> (&deadline));
	if (m_WorkLoop->sleepGateDeadline(&m_FirmwareLoaded, THREAD_INTERRUPTIBLE, deadline) != THREAD_AWAKENED) {
		DBG(dbgWarning, "Firmware upload 2 timed out\n");
		error = kIOReturnTimeout;
		goto fail;
	}
	
	DPRINTFN(WPI_DEBUG_FIRMWARE, ("Firmware loaded to driver successfully\n"));

fail:
	return error;
}


IOReturn MyClass::uploadMicrocode
( const uint8_t* data, size_t size )
{
	/*
	 * The firmware text and data segments are transferred to the NIC using DMA.
	 * The driver just copies the firmware into DMA-safe memory and tells the NIC
	 * where to find it.
	 */
	IOReturn error = kIOReturnSuccess;
	int ntries;
	
	DPRINTFN(WPI_DEBUG_HW, ("Loading microcode  size 0x%x\n", size));
	
	size /= sizeof(uint32_t);
	
	memLock();
	
	memWriteRegion(WPI_MEM_UCODE_BASE, (const uint32_t *) data, size);
	
	memWrite(WPI_MEM_UCODE_SRC, 0);
	memWrite(WPI_MEM_UCODE_DST, WPI_FW_TEXT);
	memWrite(WPI_MEM_UCODE_SIZE, size);
	
	/* run microcode */
	memWrite(WPI_MEM_UCODE_CTL, WPI_UC_RUN);
	
	/* wait while the adapter is busy copying the firmware */
	for (error = 0, ntries = 0; ntries < 1000; ntries++) {
		uint32_t status = WPI_READ(WPI_TX_STATUS);
		DPRINTFN(WPI_DEBUG_HW, ("firmware status=0x%x, val=0x%x, result=0x%x\n",
					status, WPI_TX_IDLE(6), status & WPI_TX_IDLE(6)));
		if (status & WPI_TX_IDLE(6)) {
			DPRINTFN(WPI_DEBUG_HW, ("Status Match! - ntries = %d\n", ntries));
			break;
		}
		IODelay(10);
	}
	if (ntries == 1000) {
		DBG(dbgFatal, "timeout transferring microcode\n");
		error = kIOReturnTimeout;
	}
	
	/* start the microcode executing */
	memWrite(WPI_MEM_UCODE_CTL, WPI_UC_ENABLE);
	
	memUnlock();
	
	return error;
}


IOReturn MyClass::resetAdapter( )
{
	uint32_t tmp;
	int ntries;
	
	DPRINTFN(WPI_DEBUG_HW, ("Resetting the card - clearing any uploaded firmware\n"));
	
	/* clear any pending interrupts */
	WPI_WRITE(WPI_INTR, 0xffffffff);
	
	tmp = WPI_READ(WPI_PLL_CTL);
	WPI_WRITE(WPI_PLL_CTL, tmp | WPI_PLL_INIT);
	
	tmp = WPI_READ(WPI_CHICKEN);
	WPI_WRITE(WPI_CHICKEN, tmp | WPI_CHICKEN_RXNOLOS);
	
	tmp = WPI_READ(WPI_GPIO_CTL);
	WPI_WRITE(WPI_GPIO_CTL, tmp | WPI_GPIO_INIT);
	
	/* wait for clock stabilization */
	for (ntries = 0; ntries < 25000; ntries++) {
		if (WPI_READ(WPI_GPIO_CTL) & WPI_GPIO_CLOCK)
			break;
		IODelay(10);
	}
	if (ntries == 25000) {
		DBG(dbgFatal, "Timeout waiting for clock stabilization\n");
		return kIOReturnTimeout;
	}
	
	/* initialize EEPROM */
	tmp = WPI_READ(WPI_EEPROM_STATUS);
	
	if ((tmp & WPI_EEPROM_VERSION) == 0) {
		DBG(dbgFatal, "EEPROM not found\n");
		return kIOReturnIOError;
	}
	WPI_WRITE(WPI_EEPROM_STATUS, tmp & ~(WPI_EEPROM_LOCKED));
	
	return kIOReturnSuccess;
}

IOReturn MyClass::stopMaster( )
{
	uint32_t tmp;
	int ntries;
	
	DPRINTFN(WPI_DEBUG_HW,("Disabling Firmware execution\n"));
	
	tmp = WPI_READ(WPI_RESET);
	WPI_WRITE(WPI_RESET, tmp | WPI_STOP_MASTER | WPI_NEVO_RESET);
	
	tmp = WPI_READ(WPI_GPIO_CTL);
	if ((tmp & WPI_GPIO_PWR_STATUS) == WPI_GPIO_PWR_SLEEP)
		return kIOReturnSuccess; /* already asleep */
	
	for (ntries = 0; ntries < 100; ntries++) {
		if (WPI_READ(WPI_RESET) & WPI_MASTER_DISABLED)
			break;
		IODelay(10);
	}
	if (ntries == 100) {
		DBG(dbgFatal, "Timeout waiting for master\n");
		return kIOReturnTimeout;
	}
	return kIOReturnSuccess;
}

IOReturn MyClass::powerUp( )
{
	uint32_t tmp;
	int ntries;
	
	memLock();
	tmp = memRead(WPI_MEM_POWER);
	memWrite(WPI_MEM_POWER, tmp & ~(0x03000000));
	memUnlock();
	
	for (ntries = 0; ntries < 5000; ntries++) {
		if (WPI_READ(WPI_GPIO_STATUS) & WPI_POWERED)
			break;
		IODelay(10);
	}
	if (ntries == 5000) {
		DBG(dbgFatal, "Timeout waiting for NIC to power up\n");
		return kIOReturnTimeout;
	}
	return 0;
}

void MyClass::configureHardware( )
{
	uint32_t rev, hw;
	
	/* This is straight rip from freebsd wpi driver, which is apparently
	 * a straight rip from linux driver. I have no clue what this does
	 * except that it sets some register based on revision ID etc. */
	
	/* voodoo from the Linux "driver".. */
	hw = WPI_READ(WPI_HWCONFIG);
	rev = m_PciDevice->configRead8(kIOPCIConfigRevisionID);

	if ((rev & 0xc0) == 0x40)
		hw |= WPI_HW_ALM_MB;
	else if (!(rev & 0x80))
		hw |= WPI_HW_ALM_MM;
	
	if (m_HWCap == 0x80)
		hw |= WPI_HW_SKU_MRC;
	
	hw &= ~(WPI_HW_REV_D);
	if ((m_HWRev & 0xf0) == 0xd0)
		hw |= WPI_HW_REV_D;
	
	if (m_HWType > 1)
		hw |= WPI_HW_TYPE_B;
	
	WPI_WRITE(WPI_HWCONFIG, hw);
}

/**
 * Configure the card to listen to a particular channel, this transitions the
 * card into being able to receive frames from remote devices.
 */
IOReturn MyClass::configure( )
{
	wpi_power	power;
	wpi_bluetooth	bluetooth;
	wpi_node_info	node;
	
	/* set power mode */
	bzero(&power, sizeof power);
	power.flags = WPI_POWER_CAM | 0x8;
	
	if (sendCommand(WPI_CMD_SET_POWER_MODE, &power, sizeof power, 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "Could not set power mode\n");
		return kIOReturnError;
	}
	
	/* configure bluetooth coexistence */
	bzero(&bluetooth, sizeof bluetooth);
	bluetooth.flags	= 3;
	bluetooth.lead	= 0xaa;
	bluetooth.kill	= 1;
	
	if (sendCommand(WPI_CMD_BLUETOOTH, &bluetooth, sizeof bluetooth, 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "Could not configure bluetooth co-existence\n");
		return kIOReturnError;
	}
	
	/* configure adapter */
	bzero(&m_Config, sizeof (struct wpi_config));
	bcopy(&m_MacAddress, m_Config.myaddr, 6);
	
	/*set defaults */
	m_Config.chan		 = m_CurrentChannel.number ; // XXX initially set during turnPowerOn()
	m_Config.flags		 = WPI_CONFIG_TSF;
	
	if (m_CurrentChannel.flags & IEEE::Channel::band2GHz)
		m_Config.flags |= (WPI_CONFIG_AUTO | WPI_CONFIG_24GHZ);
	
	m_Config.filter		 = 0;
	m_Config.mode		 = WPI_MODE_STA;
	m_Config.filter		|= WPI_FILTER_MULTICAST;
	m_Config.cck_mask	 = 0x0f;	/* not yet negotiated */
	m_Config.ofdm_mask	 = 0xff;	/* not yet negotiated */
	
	if (sendCommand(WPI_CMD_CONFIGURE, &m_Config, sizeof (wpi_config), 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "Configure command failed\n");
		return kIOReturnError;
	}
	
	/* configuration has changed, set Tx power accordingly */
	if (setTxPower(m_CurrentChannel, 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "could not set Tx power\n");
		return kIOReturnError;
	}
	
	/* add broadcast node */
	bzero(&node, sizeof node);
	bcopy(broadcast_bssid, node.bssid, 6);
	node.id = WPI_ID_BROADCAST;
	node.rate = plcpSignal(2);
	if (sendCommand(WPI_CMD_ADD_NODE, &node, sizeof node, 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "could not add broadcast node\n");
		return kIOReturnError;
	}
	
	/* Setup rate scalling */
	if (mrrSetup() != kIOReturnSuccess) {
		DBG(dbgFatal, "could not setup MRR\n");
		return kIOReturnError;
	}
	
	return kIOReturnSuccess;
}

uint8_t MyClass::plcpSignal
( int rate )
{
	switch (rate) {
		/* CCK rates (returned values are device-dependent) */
		case 2:		return 10;
		case 4:		return 20;
		case 11:	return 55;
		case 22:	return 110;
			
		/* OFDM rates (cf IEEE Std 802.11a-1999, pp. 14 Table 80) */
		case 12:	return 0xd;
		case 18:	return 0xf;
		case 24:	return 0x5;
		case 36:	return 0x7;
		case 48:	return 0x9;
		case 72:	return 0xb;
		case 96:	return 0x1;
		case 108:	return 0x3;
			
		/* unsupported rates (should not get there) */
		default:	return 0;
	};
}

IOReturn MyClass::sendCommand
( int code, const void* buf, int size, bool async )
{
	TxRing*		ring = &m_CmdRing;
	wpi_tx_desc*	desc;
	wpi_tx_cmd*	cmd;
	
	if (!buf) {
		DBG(dbgWarning, "Error - attempt to send command with null pointer\n");
		return kIOReturnError;
	}
	
	if (size < 0 || size > WPI_CMD_DATA_MAXLEN) {
		DBG(dbgWarning, "Error - attempt to send command of size %u\n", size);
		return kIOReturnError;
	}
	
	DPRINTFN(WPI_DEBUG_CMD,("Sending Command code %d size %d\n", code, size));
	
	if (m_Flags & WPI_FLAG_BUSY) {
		DBG(dbgWarning, "%s: cmd %d not sent, busy\n", __func__, code);
		return kIOReturnBusy;
	}
	m_Flags |= WPI_FLAG_BUSY;
	
	desc	= &ring->descriptors[ring->current];
	cmd	= &ring->cmdSlots[ring->current];
	
	cmd->code	= code;
	cmd->flags	= 0;
	cmd->qid	= ring->qid;
	cmd->idx	= ring->current;
	memcpy(cmd->data, buf, size);
	
	desc->flags		= (WPI_PAD32(size) << 28 | 1 << 24);
	desc->segs[0].addr	= (ring->cmdPhysAdd + ring->current * sizeof(wpi_tx_cmd));
	desc->segs[0].len	= 4 + size;
	
	/* kick cmd ring */
	ring->current = (ring->current + 1) % WPI_CMD_RING_COUNT;
	WPI_WRITE(WPI_TX_WIDX, ring->qid << 8 | ring->current);
	
	if (async) {
		// Return immediately
		m_Flags &= ~ WPI_FLAG_BUSY;
		return kIOReturnSuccess;
	} else {
		// Wait at most 1 second
		AbsoluteTime deadline;
		clock_interval_to_deadline(1000, kMillisecondScale, reinterpret_cast<uint64_t*> (&deadline));
		if (m_WorkLoop->sleepGateDeadline(&m_CommandDone, THREAD_INTERRUPTIBLE, deadline) != THREAD_AWAKENED) {
			DBG(dbgWarning, "Command %u timed out\n", ring->current);
			return kIOReturnTimeout;
		} else
			return kIOReturnSuccess;
	}
}

IOReturn MyClass::sendManagementFrame
( const uint8_t* data, const size_t len )
{
	mbuf_t m;
	DBG(dbgInfo, "Sending management frame of length=%u\n", len);
	m = allocatePacket(len);
	if (!m) {
		DBG(dbgWarning, "Couldn't allocate mbuf of len=%d for sending mgmt frame\n", len);
		return kIOReturnError;
	}
	
	mbuf_copyback(m, 0, len, data, MBUF_DONTWAIT);
	TxFrameHeader hdr; // dummy
	
	DUMP_MBUF(m, "MGMT frame");
	
	return outputFrame(hdr, m);
}

void MyClass::sendAssocRequest( )
{
	/*
	 * Step 4
	 */
	uint8_t* frm;
	int numRates, numExtRates;
	
	m_AssocState = staAssocTry;
	AssocRequestFrame ar;
	bzero(&ar, sizeof(ar));
	
	ar.hdr.hdr.type	= IEEE::WiFiFrameHeader::ManagementFrame;
	ar.hdr.hdr.subtype = IEEE::WiFiFrameHeader::AssocRequest;
	bcopy(&m_MacAddress, ar.hdr.sa, 6);
	bcopy(&m_AssocParams.bssid, ar.hdr.bssid, 6);
	bcopy(&m_AssocParams.bssid, ar.hdr.da, 6);
	
	ar.cap.bits.ESS	= true;
	ar.cap.bits.ShortSlotTime = m_AssocParams.capability.bits.ShortSlotTime;
	ar.cap.bits.ShortPreamble = m_AssocParams.capability.bits.ShortPreamble;
	ar.lintval = 10; // FIXME: power management is off so we can put whatever, but use m_AssocParams.beaconInterval;
	
	frm = ar.ie;
	
	/* Add SSID elem ID */
	*frm++ = IEEE::IE::ieSSID;
	*frm++ = m_AssocParams.ssid->getLength();
	bcopy(m_AssocParams.ssid->getBytesNoCopy(), frm, m_AssocParams.ssid->getLength());
	frm += m_AssocParams.ssid->getLength();
	
	/* Sort the list of supported rates for later AMRR usage */
	for (int a = 0; a < m_AssocParams.supportedRates.numItems; a++) {
		int smallest = a;
		for (int b = a; b < m_AssocParams.supportedRates.numItems; b++) {
			if ((0x7f & m_AssocParams.supportedRates.rate[b]) < 
			    (0x7f & m_AssocParams.supportedRates.rate[smallest]))
			{
				smallest = b;
			}
		}
		if (smallest == a) continue;
		IEEE::DataRate temp = m_AssocParams.supportedRates.rate[a];
		m_AssocParams.supportedRates.rate[a] = m_AssocParams.supportedRates.rate[smallest];
		m_AssocParams.supportedRates.rate[smallest] = temp;
	}
	
	DBG(dbgWarning, "ASSOC: Supported rates %u [ ", m_AssocParams.supportedRates.numItems);
	for (int rt = 0; rt < m_AssocParams.supportedRates.numItems; rt++)
		DBGC(dbgWarning, "%u ", (unsigned int) ((0x7f & m_AssocParams.supportedRates.rate[rt])/2));
	DBGC(dbgWarning, "]\n");
	
	/* add supported rates IE */
	numRates = MIN(8, m_AssocParams.supportedRates.numItems);
	*frm++ = IEEE::IE::ieSupportedRates;
	*frm++ = numRates;
	for (int r = 0; r < numRates; r++)
		*frm++ = m_AssocParams.supportedRates.rate[r];
	
	/* add extended supported rates IE if needed */
	numExtRates = m_AssocParams.supportedRates.numItems - numRates;
	if (numExtRates > 0) {
		*frm++ = IEEE::IE::ieExtendedSupportedRates;
		*frm++ = numExtRates;
		for (int er = 0; er < numExtRates; er++)
			*frm++ = m_AssocParams.supportedRates.rate[er + 8];
	}
	
	uint8_t* ar_start = (uint8_t*) &ar;
	size_t framelen = (frm - ar_start);
	
	DBG(dbgWarning, "ASSOC: Step 4 (almost) done\n");
	if (sendManagementFrame((uint8_t*) &ar, framelen) != kIOReturnSuccess) {
		DBG(dbgWarning, "Error while sending assoc request\n");
		report(msgAssociationFailed);
		m_AssocState = staInit;
	} // else we wait for assoc response
}


void MyClass::postAssocProcedure
( int assocID )
{
	/*
	 * Step 6 - we are associated, prepare hardware
	 */
	
	/* Enable TSF */
	wpi_cmd_tsf	tsf;
	uint64_t	val, mod;

	memset(&tsf, 0, sizeof tsf);
	memcpy(&tsf.tstamp, &m_Timestamp, 8);
	tsf.bintval = m_BeaconInterval;
	tsf.lintval = 10;
	
	/* compute remaining time until next beacon */
	val = (uint64_t)m_BeaconInterval * 1024;	/* msec -> usec */
	mod = tsf.tstamp % val;
	tsf.binitval = (uint32_t)(val - mod);
	
	DPRINTF(("Enabling TSF with bintval = %u, binitval = %u, tstamp = %ull\n",
		 tsf.bintval, tsf.binitval, tsf.tstamp));
	
	if (sendCommand(WPI_CMD_TSF, &tsf, sizeof tsf, 1) != 0)
		DBG(dbgWarning, "could not enable TSF\n");		

	
	
	/* update adapter's configuration */
	m_Config.associd = assocID & ~(0xc000);
	/* short preamble/slot time are negotiated when associating */
	m_Config.flags &= ~(WPI_CONFIG_SHPREAMBLE | WPI_CONFIG_SHSLOT);
	if (m_AssocParams.capability.bits.ShortSlotTime)
		m_Config.flags |= (WPI_CONFIG_SHSLOT);
	if (m_AssocParams.capability.bits.ShortPreamble)
		m_Config.flags |= (WPI_CONFIG_SHPREAMBLE);
	m_Config.filter |= (WPI_FILTER_BSS);
	
	DPRINTF(("config chan %d flags %x\n", m_Config.chan, m_Config.flags));
	if (sendCommand(WPI_CMD_CONFIGURE, &m_Config, sizeof (wpi_config), 1) != kIOReturnSuccess) {
		DBG(dbgWarning, "Could not configure card post-association\n");
		goto fail;
	}
	
	if (setTxPower(m_AssocParams.channel, 1) != kIOReturnSuccess) {
		DBG(dbgWarning, "Could not set Tx power post-association\n");
		goto fail;
	}
	
	/* link LED always on while associated */
	setLED(WPI_LED_LINK, 0, 1);
	DBG(dbgWarning, "ASSOC: Step 6 - hardware prepared\n");
	
	/* Initialize AMRR */
	amrr.amrr_min_success_threshold = IEEE80211_AMRR_MIN_SUCCESS_THRESHOLD;
	amrr.amrr_max_success_threshold = IEEE80211_AMRR_MAX_SUCCESS_THRESHOLD;
	ieee80211_amrr_node_init(&amrr, &amrrNode);
	
	/*
	 * Step 7 - set necessary private vars and signal superclass
	 */
	m_AssocState = staAssociated;
	m_CurrentChannel = m_AssocParams.channel;
	m_AssocID = assocID;
	report(msgAssociationDone);
	DBG(dbgWarning, "ASSOC: Step 7 - superclass notified. WE ARE ASSOCIATED!\n");
	return;
	
fail:
	DBG(dbgWarning, "ASSOC: Step 6 - failed!\n");
	m_AssocState = staInit;
	report(msgAssociationFailed);
	return;
}

IOReturn MyClass::setTxPower
( IEEE::Channel c, bool async )
{
	wpi_power_group*	group;
	wpi_cmd_txpower		txpower;
	u_int			chan;
	int			i;
	
	/* get channel number */
	chan = c.number;
	
	/* find the power group to which this channel belongs */
	if (c.flags & IEEE::Channel::band5GHz) {
		for (group = &m_PowerGroups[1]; group < &m_PowerGroups[4]; group++)
			if (chan <= group->chan)
				break;
	} else
		group = &m_PowerGroups[0];
	
	bzero(&txpower, sizeof txpower);
	txpower.band	= (c.flags & IEEE::Channel::band5GHz) ? 0 : 1;
	txpower.channel	= chan;
	
	/* set Tx power for all OFDM and CCK rates */
	for (i = 0; i <= 11 ; i++) {
		/* retrieve Tx power for this channel/rate combination */
		int idx = getPowerIndex(group, c, wpi_ridx_to_rate[i]);
		
		txpower.rates[i].rate = wpi_ridx_to_plcp[i];
		
		if (c.flags & IEEE::Channel::band5GHz) {
			txpower.rates[i].gain_radio	= wpi_rf_gain_5ghz[idx];
			txpower.rates[i].gain_dsp	= wpi_dsp_gain_5ghz[idx];
		} else {
			txpower.rates[i].gain_radio	= wpi_rf_gain_2ghz[idx];
			txpower.rates[i].gain_dsp	= wpi_dsp_gain_2ghz[idx];
		}
		DPRINTFN(WPI_DEBUG_TEMP,("chan %d/rate %d: power index %d\n", chan, wpi_ridx_to_rate[i], idx));
	}
	
	return sendCommand(WPI_CMD_TXPOWER, &txpower, sizeof txpower, async);
}

/*
 * Determine Tx power index for a given channel/rate combination.
 * This takes into account the regulatory information from EEPROM and the
 * current temperature.
 */
int MyClass::getPowerIndex
( wpi_power_group* group, IEEE::Channel c, int rate)
{	
	wpi_power_sample* sample;
	int pwr, idx;
	u_int chan;
	
	/* get channel number */
	chan = c.number;
	
	/* default power is group's maximum power - 3dB */
	pwr = group->maxpwr / 2;
	
	/* decrease power for highest OFDM rates to reduce distortion */
	switch (rate) {
		case 72:	/* 36Mb/s */
			pwr -= (c.flags & IEEE::Channel::band2GHz) ? 0 :  5;
			break;
		case 96:	/* 48Mb/s */
			pwr -= (c.flags & IEEE::Channel::band2GHz) ? 7 : 10;
			break;
		case 108:	/* 54Mb/s */
			pwr -= (c.flags & IEEE::Channel::band2GHz) ? 9 : 12;
			break;
	}
	
	/* never exceed channel's maximum allowed Tx power */
	pwr = min(pwr, m_MaxPower[chan]);
	
	/* retrieve power index into gain tables from samples */
	for (sample = group->samples; sample < &group->samples[3]; sample++)
		if (pwr > sample[1].power)
			break;
	/* fixed-point linear interpolation using a 19-bit fractional part */
	idx = interpolate(pwr, sample[0].power, sample[0].index, sample[1].power, sample[1].index, 19);
	
	/*
	 *  Adjust power index based on current temperature
	 *	- if colder than factory-calibrated: decreate output power
	 *	- if warmer than factory-calibrated: increase output power
	 */
	idx -= (m_Temperature - group->temp) * 11 / 100;
	
	/* decrease power for CCK rates (-5dB) */
	if (!WPI_RATE_IS_OFDM(rate))
		idx += 10;
	
	/* keep power index in a valid range */
	if (idx < 0)
		return 0;
	if (idx > WPI_MAX_PWR_INDEX)
		return WPI_MAX_PWR_INDEX;
	return idx;
}

IOReturn MyClass::mrrSetup( )
{
	wpi_mrr_setup	mrr;
	int		i;
	
	bzero(&mrr, sizeof (wpi_mrr_setup));
	
	/* CCK rates (not used with 802.11a) */
	for (i = WPI_CCK1; i <= WPI_CCK11; i++) {
		mrr.rates[i].flags = 0;
		mrr.rates[i].signal = wpi_ridx_to_plcp[i];
		/* fallback to the immediate lower CCK rate (if any) */
		mrr.rates[i].next = (i == WPI_CCK1) ? WPI_CCK1 : i - 1;
		/* try one time at this rate before falling back to "next" */
		mrr.rates[i].ntries = 1;
	}
	
	/* OFDM rates (not used with 802.11b) */
	for (i = WPI_OFDM6; i <= WPI_OFDM54; i++) {
		mrr.rates[i].flags = 0;
		mrr.rates[i].signal = wpi_ridx_to_plcp[i];
		/* fallback to the immediate lower OFDM rate (if any) */
		/* we allow fallback from OFDM/6 to CCK/2 in 11b/g mode */
		mrr.rates[i].next = (i == WPI_OFDM6) ? WPI_CCK2 : (i - 1);
		/* try one time at this rate before falling back to "next" */
		mrr.rates[i].ntries = 1;
	}
	
	/* setup MRR for control frames */
	mrr.which = WPI_MRR_CTL;
	if (sendCommand(WPI_CMD_MRR_SETUP, &mrr, sizeof mrr, 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "could not setup MRR for control frames\n");
		return kIOReturnError;
	}
	
	/* setup MRR for data frames */
	mrr.which = WPI_MRR_DATA;
	if (sendCommand(WPI_CMD_MRR_SETUP, &mrr, sizeof mrr, 0) != kIOReturnSuccess) {
		DBG(dbgFatal, "could not setup MRR for data frames\n");
		return kIOReturnError;
	}
	
	return kIOReturnSuccess;
}

void MyClass::setLED
( uint8_t which, uint8_t off, uint8_t on )
{
	wpi_cmd_led led;
	
	led.which	= which;
	led.unit	= 100000;	/* on/off in unit of 100ms */
	led.off		= off;
	led.on		= on;
	
	sendCommand(WPI_CMD_SET_LED, &led, sizeof led, 1);
}

#pragma mark -
#pragma mark Interrupt handlers
void MyClass::interruptOccurred
( OSObject* owner, IOInterruptEventSource* intr, int count )
{
	uint32_t r;
	
	r = WPI_READ(WPI_INTR);
	if (r == 0 || r == 0xffffffff) {
		return;
	}
	
	/* disable interrupts */
	WPI_WRITE(WPI_MASK, 0);
	/* ack interrupts */
	WPI_WRITE(WPI_INTR, r);
	
	if (r & (WPI_SW_ERROR | WPI_HW_ERROR)) {	
		DBG(dbgFatal, "fatal firmware error %s\n",
		    (r & WPI_SW_ERROR) ? "(Software Error)" : "(Hardware Error)");
		m_Flags &= ~(WPI_FLAG_BUSY);
		turnPowerOff();
		report(msgPowerOff);
		return;
	}
	
	if (r & WPI_RX_INTR)
		notificationInterrupt();
	
	if (r & WPI_ALIVE_INTR)	/* firmware initialized */
		m_WorkLoop->wakeupGate(&m_FirmwareLoaded, true /*wakeup one thread*/);
	
	/* re-enable interrupts */
	WPI_WRITE(WPI_MASK, WPI_INTR_MASK);
	
	return;
}


void MyClass::timerOccurred
( OSObject* owner, IOTimerEventSource* timer )
{
	int temp;
	if (m_AssocState != staAssociated) {
		m_Timer->setTimeoutMS(10000); // come back 10 sec later
		return;
	}
	/* Calibrate power based on temperature */
	temp = (int)WPI_READ(WPI_TEMPERATURE);
	
	/* sanity-check read value */
	if (temp < -260 || temp > 25) {
		/* this can't be correct, ignore */
		DPRINTFN(WPI_DEBUG_TEMP, ("out-of-range temperature reported: %d\n", temp));
	} else {
		DPRINTFN(WPI_DEBUG_TEMP, ("temperature %d->%d\n", m_Temperature, temp));
		
		/* adjust Tx power if need be */
		if (abs(temp - m_Temperature) <= 6) {
			m_Timer->setTimeoutMS(30000); // come back 30 sec later
			return;
		}
		
		m_Temperature = temp;
		
		if (setTxPower(m_CurrentChannel, 1) != 0) {
			/* just warn, too bad for the automatic calibration... */
			DBG(dbgWarning, "could not adjust Tx power\n");
		}
	}
	
	m_Timer->setTimeoutMS(60000); // calibrated, next calib after 1 min
	return;
}

void MyClass::notificationInterrupt( )
{
	wpi_rx_desc*	desc;
	uint32_t	hw;
	
	hw = m_SharedPagePtr->next;
	while (m_RxRing.current != hw) {
		
		desc = (wpi_rx_desc*) mbuf_data(m_RxRing.mbufs[m_RxRing.current]);
		
		/*DPRINTFN(WPI_DEBUG_NOTIFY,
			 ("notify qid=%x idx=%d flags=%x type=%d len=%d\n",
			  desc->qid,
			  desc->idx,
			  desc->flags,
			  desc->type,
			  desc->len));
		*/
		if (!(desc->qid & 0x80))	/* reply to a command */
			cmdInterrupt(desc);
		
		switch (desc->type) {
			case WPI_RX_DONE:
				/* a 802.11 frame was received */
				rxInterrupt(desc);
				break;
				
			case WPI_TX_DONE:
				/* a 802.11 frame has been transmitted */
				txInterrupt(desc);
				break;
				
			case WPI_UC_READY:
			{
				wpi_ucode_info* uc = (wpi_ucode_info*) (desc + 1);
				
				/* the microcontroller is ready */
				DPRINTF(("microcode alive notification version %x alive %x\n", uc->version, uc->valid));
				
				if (uc->valid != 1) {
					DBG(dbgFatal, "microcontroller initialization failed\n");
					turnPowerOff();
					report(msgPowerOff);
				}
				break;
			}
			case WPI_STATE_CHANGED:
			{
				uint32_t *status = (uint32_t*) (desc + 1);
				
				/* enabled/disabled notification */
				DPRINTF(("state changed to %x\n", *status));
				
				if (*status & 1) {
					DBG(dbgWarning, "Radio transmitter is switched off\n");
					m_Flags |= WPI_FLAG_HW_RADIO_OFF;
					/* Disable firmware commands */
					WPI_WRITE(WPI_UCODE_SET, WPI_DISABLE_CMD);
					/* don't turn off - let user figure it out :P
					turnPowerOff();
					report(msgPowerOff);
					 */
				}
				break;
			}
			case WPI_START_SCAN:
			{
				wpi_start_scan *scan = (wpi_start_scan*) (desc + 1);
				
				DPRINTFN(WPI_DEBUG_SCANNING, ("scanning channel %d status %x\n",
							      scan->chan, scan->status));
				break;
			}
			case WPI_STOP_SCAN:
			{
				wpi_stop_scan *scan = (wpi_stop_scan *)(desc + 1);
				
				DPRINTFN(WPI_DEBUG_SCANNING,
					 ("scan finished nchan=%d status=%d chan=%d\n",
					  scan->nchan, scan->status, scan->chan));
				m_Flags &= ~(WPI_FLAG_SCANNING);
				report(msgScanCompleted);
				break;
			}
			case WPI_MISSED_BEACON:
			{
				wpi_missed_beacon *beacon = (wpi_missed_beacon *)(desc + 1);
				
				if (beacon->consecutive >= 35 /* threshold */) {
					DPRINTF(("Beacon miss: %u >= %u\n", beacon->consecutive, 35));
					report(msgDisassociated);
				}
				break;
			}
		}
		
		m_RxRing.current = (m_RxRing.current + 1) % WPI_RX_RING_COUNT;
	}
	
	/* tell the firmware what we have processed */
	hw = (hw == 0) ? WPI_RX_RING_COUNT - 1 : hw - 1;
	WPI_WRITE(WPI_RX_WIDX, hw & ~7);	
}

void MyClass::rxInterrupt
( wpi_rx_desc* desc )
{
	wpi_rx_stat*	stat;
	wpi_rx_head*	head;
	wpi_rx_tail*	tail;
	IEEE::WiFiFrameHeader* wh;
	
	stat = (wpi_rx_stat* ) (desc + 1);
	
	if (stat->len > WPI_STAT_MAXLEN) {
		DBG(dbgWarning, "invalid rx statistic header\n");
		return;
	}
	
	head = (wpi_rx_head *)((uint8_t*)(stat + 1) + stat->len);
	tail = (wpi_rx_tail *)((uint8_t*)(head + 1) + head->len);
	
	/*DPRINTFN(WPI_DEBUG_RX, ("rx intr: idx=%d len=%d stat len=%d rssi=%d rate=%x chan=%d tstamp=%ju\n",
				ring->current, desc->len, head->len, (int8_t)stat->rssi, head->rate, head->chan,
				(uintmax_t)tail->tstamp));
	*/
	/* discard Rx frames with bad CRC early */
	if ((tail->flags & WPI_RX_NOERROR) != WPI_RX_NOERROR) {
		//DPRINTFN(WPI_DEBUG_RX, ("%s: rx flags error %x\n", __func__, tail->flags));
		return;
	}
	if (head->len < sizeof (IEEE::TxDataFrameHeader)) {
		DPRINTFN(WPI_DEBUG_RX, ("%s: frame too short: %d\n", __func__, head->len));
		return;
	}
	
	/* Check whether this is authentication reply */
	wh = (IEEE::WiFiFrameHeader*) (head + 1);
	if (wh->type	== IEEE::WiFiFrameHeader::ManagementFrame &&
	    wh->subtype	== IEEE::WiFiFrameHeader::Authentication)
	{
		/*
		 * Step 3
		 */
		DBG(dbgWarning, "Received authentication response\n");
		if (m_AssocState != staAuthTry) {
			DBG(dbgWarning, "Auth response received but we are not waiting for it\n");
			return;
		}
		AuthenticationFrame* af = (AuthenticationFrame*) wh;
		if (af->status == 0) {
			DBG(dbgWarning, "ASSOC: Step 3 - auth successful\n");
			m_AssocState = staAssocTry;
			report(msgAuthenticationDone);
			/*
			 * Step 4
			 */
			sendAssocRequest();
		} else {
			DBG(dbgWarning, "ASSOC: Step 3 - auth fail status code %u\n", af->status);
			m_AssocState = staInit;
			report(msgAuthenticationFailed);
		}
		return; // because we don't have to pass this frame to superclass
	}
	
	/* Check whether this is assoc response */
	if (wh->type	== IEEE::WiFiFrameHeader::ManagementFrame &&
	    wh->subtype	== IEEE::WiFiFrameHeader::AssocResponse)
	{
		/*
		 * Step 5
		 */
		DBG(dbgWarning, "Received association response\n");
		if (m_AssocState != staAssocTry) {
			DBG(dbgWarning, "Assoc response received but we are not waiting for it\n");
			return;
		}
		AssocResponseFrame* ar = (AssocResponseFrame*) wh;
		if (ar->status == 0) {
			DBG(dbgWarning, "ASSOC: Step 5 - assoc successful!\n");
			m_AssocState = staAssociated;
			/*
			 * Step 6 and 7
			 */
			postAssocProcedure(ar->aid);
		} else {
			DBG(dbgWarning, "ASSOC: Step 5 - assoc fail status code %u\n", ar->status);
			m_AssocState = staInit;
			report(msgAssociationFailed);
		}
		return;
	}
	
	/* Check for beacon, and update the timestamps */
	if (wh->type == IEEE::WiFiFrameHeader::ManagementFrame && wh->subtype == IEEE::WiFiFrameHeader::Beacon)
	{
		IEEE::ProbeResponseFrameHeader* pr = (IEEE::ProbeResponseFrameHeader*) wh; // same format as beacon
		m_Timestamp	 = pr->timestamp;
		m_BeaconInterval = pr->beaconInterval;
		// we won't 'return' from the function, we'll let this beacon be passed to superclass for further usage
	}
	
	/* Allocate a new packet and copy the data into it (I know, it's not very efficient, but right now we are
	 * at the 'get it working first!' stage - optimization will come later
	 */
	
	/* New method 
	mbuf_t rxM = m_RxRing.mbufs[m_RxRing.current]; // get mbuf
	mbuf_adj(rxM, sizeof(wpi_rx_desc) + sizeof(wpi_rx_stat) + stat->len + sizeof(wpi_rx_head)); // trim HW headers from front
	mbuf_adj(rxM, -(mbuf_pkthdr_len(rxM) - head->len)); // trim extra bytes past the length of data from the end
	// Allocate and set new packet for hardware 
	m_RxRing.mbufs[m_RxRing.current] = 0; // initilize to 0 so we can check if allocatePacket failed
	m_RxRing.mbufs[m_RxRing.current] = allocatePacket(MCLBYTES); // allocate a packet into which HW will write rx'd packet
	if (m_RxRing.mbufs[m_RxRing.current] == 0) {
		DBG(dbgFatal, "Could not allocate packet no. %d during Rx ring allocation!\n", m_RxRing.current);
		return;
	}
	m_RxRing.rx_pkt_ptr[m_RxRing.current] = mbuf_data_to_physical(mbuf_data(m_RxRing.mbufs[m_RxRing.current])); // tell HW where it is
	*/
	
	/* Old method of copying the whole data into a new mbuf - */
	mbuf_t mnew = 0;
	mnew = allocatePacket(head->len);
	if (!mnew) {
		DBG(dbgWarning, "Couldn't allocate new packet for Rx, dropped\n");
		return;
	}
	
	mbuf_copyback(mnew, 0, head->len, (head + 1), MBUF_DONTWAIT);
	
	
	/* Prepare the extra info header */
	RxFrameHeader hdr;
	bzero(&hdr, sizeof hdr);
	hdr.channel.number = head->chan; // FIXME: set channel flags too
	hdr.signalLevel	= (int8_t)(stat->rssi - WPI_RSSI_OFFSET);
	
	switch (head->rate) {
			/* CCK rates */
		case  10: hdr.rate = IEEE::rate1Mbps; break;
		case  20: hdr.rate = IEEE::rate2Mbps; break;
		case  55: hdr.rate = IEEE::rate5Mbps; break;
		case 110: hdr.rate = IEEE::rate11Mbps; break;
			/* OFDM rates */
		case 0xd: hdr.rate = IEEE::rate6Mbps; break;
		case 0xf: hdr.rate = IEEE::rate9Mbps; break;
		case 0x5: hdr.rate = IEEE::rate12Mbps; break;
		case 0x7: hdr.rate = IEEE::rate18Mbps; break;
		case 0x9: hdr.rate = IEEE::rate24Mbps; break;
		case 0xb: hdr.rate = IEEE::rate36Mbps; break;
		case 0x1: hdr.rate = IEEE::rate48Mbps; break;
		case 0x3: hdr.rate = IEEE::rate54Mbps; break;
			/* unknown rate: should not happen */
		default:  hdr.rate = IEEE::rateUnspecified;
	}
	
	/* Pass it up to upper layer */
	inputFrame(hdr, mnew);	
}

void MyClass::txInterrupt
( wpi_rx_desc* desc )
{
	TxRing*		ring = &m_TxRing[desc->qid & 0x3];
	
	wpi_tx_stat*	stat = (wpi_tx_stat*) (desc + 1);
	
	DPRINTFN(WPI_DEBUG_TX, ("tx done: qid=%d idx=%d retries=%d nkill=%d "
				"rate=%x duration=%d status=%x, queued=%u\n", desc->qid, desc->idx,
				stat->ntries, stat->nkill, stat->rate, stat->duration,
				stat->status, ring->queued));
	
	/* Update rate-control statistics */
	amrrNode.amn_txcnt++;
	if (stat->ntries > 0)
		amrrNode.amn_retrycnt++;
	
	if (ring->mbufs[desc->idx]) {
		freePacket(ring->mbufs[desc->idx]);
		ring->mbufs[desc->idx] = 0;
	}
	
	ring->queued--;
}

void MyClass::cmdInterrupt
( wpi_rx_desc* desc )
{
	/*DPRINTFN(WPI_DEBUG_CMD, ("cmd notification qid=%x idx=%d flags=%x "
				 "type=%u len=%d\n", desc->qid, desc->idx,
				 desc->flags, desc->type, desc->len));
	*/
	if ((desc->qid & 7) != 4)
		return;	/* not a command ack */
	
	/* If the command was mapped in a mbuf, free it */
	if (m_CmdRing.mbufs[desc->idx] != 0) {
		freePacket(m_CmdRing.mbufs[desc->idx]);
		m_CmdRing.mbufs[desc->idx] = 0;
	}
	
	m_Flags &= ~(WPI_FLAG_BUSY);
	m_WorkLoop->wakeupGate(&m_CommandDone, true /* wake up one thread only */);
}

#pragma mark -
#pragma mark Utility functions

void MyClass::memLock( )
{
	int ntries;
	uint32_t tmp;
	
	tmp = WPI_READ(WPI_GPIO_CTL);
	WPI_WRITE(WPI_GPIO_CTL, tmp | WPI_GPIO_MAC);
	
	/* spin until we actually get the lock */
	for (ntries = 0; ntries < 100; ntries++) {
		if ((WPI_READ(WPI_GPIO_CTL) & (WPI_GPIO_CLOCK | WPI_GPIO_SLEEP)) == WPI_GPIO_CLOCK)
			break;
		IODelay(10);
	}
	if (ntries == 100)
		DBG(dbgWarning, "Could not lock memory\n");
}

inline void MyClass::memUnlock( )
{
	uint32_t tmp = WPI_READ(WPI_GPIO_CTL);
	WPI_WRITE(WPI_GPIO_CTL, tmp & ~WPI_GPIO_MAC);
}

inline void MyClass::memWrite
( uint32_t offset, uint32_t data )
{
	WPI_WRITE(WPI_WRITE_MEM_ADDR, WPI_MEM_4 | offset);
	WPI_WRITE(WPI_WRITE_MEM_DATA, data);
}

inline uint32_t MyClass::memRead
( uint32_t offset )
{
	WPI_WRITE(WPI_READ_MEM_ADDR, WPI_MEM_4 | offset);
	return WPI_READ(WPI_READ_MEM_DATA);
}

inline void MyClass::memWriteRegion
( uint32_t offset, const uint32_t* data, int ndwords )
{
	for (; ndwords > 0; ndwords--, data++, offset += 4)
		memWrite(offset, *data);
}

/*
 * Read data from the EEPROM.  We access EEPROM through the MAC instead of
 * using the traditional bit-bang method. Data is read up until len bytes have
 * been obtained.
 */
IOReturn MyClass::readPromData
( uint32_t addr, void *data, int len )
{
	int		ntries;
	uint32_t	val;
	uint8_t*	out = (uint8_t*) data;
	
	memLock();
	
	for (; len > 0; len -= 2, addr++) {
		WPI_WRITE(WPI_EEPROM_CTL, addr << 2);
		
		for (ntries = 0; ntries < 10; ntries++) {
			if ((val = WPI_READ(WPI_EEPROM_CTL)) & WPI_EEPROM_READY)
				break;
			IODelay(5);
		}
		
		if (ntries == 10) {
			DBG(dbgWarning, "Could not read EEPROM\n");
			return kIOReturnTimeout;
		}
		
		*out++= val >> 16;
		if (len > 1)
			*out ++= val >> 24;
	}
	
	memUnlock();
	
	return kIOReturnSuccess;
}

IOBufferMemoryDescriptor* MyClass::allocDmaMemory
( size_t size, int alignment, void** vaddr, uint32_t* paddr )
{
	size_t		reqsize;
	uint64_t	phymask;
	int		i;
	
	if (alignment <= PAGE_SIZE) {
		reqsize = size;
		phymask = 0x00000000ffffffffull & (~(alignment - 1));
	} else {
		reqsize = size + alignment;
		phymask = 0x00000000fffff000ull; /* page-aligned */
	}
		
	IOBufferMemoryDescriptor* mem = 0;
	mem = IOBufferMemoryDescriptor::inTaskWithPhysicalMask(kernel_task, kIOMemoryPhysicallyContiguous,
							       reqsize, phymask);
	if (!mem) return 0;
	mem->prepare();
	*paddr = mem->getPhysicalAddress();
	*vaddr = mem->getBytesNoCopy();
	
	/*
	 * Check the alignment and increment by 4096 until we get the
	 * requested alignment. Fail if can't obtain the alignment
	 * we requested.
	 */
	if ((*paddr & (alignment - 1)) != 0) {
		for (i = 0; i < alignment / 4096; i++) {
			if ((*paddr & (alignment - 1 )) == 0)
				break;
			*paddr += 4096;
			*vaddr = ((uint8_t*) *vaddr) + 4096;
		}
		if (i == alignment / 4096) {
			DBG(dbgWarning, "Memory alloc alignment requirement %d was not satisfied\n", alignment);
			mem->complete();
			mem->release();
			return 0;
		}
	}
	return mem;
}
