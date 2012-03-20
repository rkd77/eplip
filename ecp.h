/***************************************************************************
                          ecp.h  -  header file for ECP devices
                             -------------------
        begin           : Thu Nov 9  2000
        last change	: Sun May 20 2001
        version	 	: 0.3
        copyright       : (C) 2001 by Angel Valkov
        email           : whitefang@dir.bg
 ***************************************************************************/

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 ***************************************************************************/

/***************************************************************************
CHANGES:
        v.0.1:  This is the initial version. Macro definitions and code moved
                from ppdebug/main.cpp into this header file and the ecp.cpp file
        v.0.2:	Formating. JavaDoc Style Comments.
        v.0.3:  Replaced some macros with inline function( GNUC compiler
                seems to get better optimized code with inlines instead of
                macro defs )

BUGS:   Too many to be listed ;-)
***************************************************************************/

#ifndef _ECP_H
#define _ECP_H

#if !( defined( LINUX ) || defined ( linux ) ) && defined( __KERNEL__ )
# include "basetype.h"
# include "util.h"
#endif

#include "ioport.h"

#ifndef HWD_DEBUG

/** Define HWD_DEBUG 1 to enable ECR_BUGGY detection
  */
#define HWD_DEBUG 0

#endif

/**
  * Offsets from the IOBASE for SPP/EPP/ECP registers
  */
#define DR              0	/** o  	Data Reg		=> SPP,EPP		*/
#define AFIFO  		0    	/** i/o	Address FIFO		=> ECP                  */
#define SR		1    	/** i  	Status Register 	=> All Modes            */
#define CR		2    	/** o  	Control Register	=> All Modes            */
#define AR_EPP		3    	/** i/o	Address Register 	=> EPP Mode             */
#define DR_EPP		4    	/** i/o	Data Register	 	=> EPP Mode             */
#define DFIFO		0x400	/** i/o	Data FIFO	 	=> All ECP		*/
#define CFGA		0x400	/** i/o	Config. Register A	=> ECP Config Mode      */
#define CFGB		0x401	/** i/o	Config. Register B   	=> ECP Config Mode      */
#define ECR		0x402	/** i/o	Extended Control Reg	=> All ECP              */

/** Offsets from IOEXTENT for ECP registers
  * IOEXTENT might be relocateable in PCI implementations.
  */
#define PCI_DFIFO       0x00
#define PCI_CFGA        0x00
#define PCI_CFGB        0x01
#define PCI_ECR         0x02
/**
  * SPP Control Register bits
  */
#define CR_STB		0x01
#define CR_AUTOFD	0x02
#define CR_INIT		0x04
#define CR_SLCT		0x08
#define CR_IRQ		0x10
#define CR_DIR		0x20

/**
  * SPP Status Register bits
  */
#define SR_SLCTIN	0x10
#define SR_PAPOUT       0x20
#define SR_ACK		0x40
#define SR_BUSY		0x80

/**
  * ECP ECR Register bites
  */
#define FIFO_EMPTY	0x1
#define FIFO_FULL	0x2
#define SRVC_INTR	0x4
#define	DMA_ENABLE	0x8
#define IRQ_ENABLE	0x10
/* The IRQ_ENABLE is with inverted meanning- if SET(1) IRQ
 * generation upon ECP events is disabled
 * so its more clear to call this bit IRQ_DISABLE
 */
#define IRQ_DISABLE     0x10

/**
  * The MSB 3 bits of ECR set the ECP operation mode
  */
#define MODE_SPP	0x00	/** SPP COMPAT mode					*/
#define MODE_PS2        0x20    /** BiDi SPP aka PS2 mode 				*/
#define MODE_SPP_FIFO   0x40    /** HWD assisted SPP mode				*/
#define MODE_ECP_FIFO   0x60    /** ECP Data Tarnasfer mode 				*/
#define MODE_EPP        0x80    /** EPP mode						*/
#define MODE_NONE       0xA0    /** Reserever - not used				*/
#define MODE_FIFO_TEST  0xC0    /** FIFO TEST ECP mode					*/
#define MODE_CONFIG     0xE0    /** ECP CONFIG mode					*/
#define MODE_ALL	0xE0    /** Mask		                                */

/**
  * CFGA Register bits/masks for multibit flags
  */
#define HRECOVER_UNSENT	0x3     /** Number of unsent bytes in the FIFO			*/
#define HRECOVER_PIPE	0x4     /** 0x00 - FIFO not affected ; 0x04 - FIFO affected	*/
#define DBUS_WIDTH	0x70    /** 0x00 - 16bit ; 0x10 - 8bit ; 0x20 - 32 bit		*/
#define IRQ_TRIGGER	0x80    /** 0x00  - by EDGE;	0x80 - by LEVEL			*/

/**
  * CFGB Register bits/masks for mutibit flags
  */
#define ECP_DMA		0x07    /** DMA chanel used by the ECP HWD		        */
#define ECP_IRQ		0x38    /** IRQ line used by the ECP HWD			*/
#define ECP_INT		0x40    /** 0x40 - INT line status on the PIC for this IRQ      */
#define ECP_RLE		0x80	/** 0x80 - RLE Compresion Enabled- if supported         */

/*--------------------------------------------------------------------------*/
#ifdef  READ_CONTROL
#undef  READ_CONTROL
#endif
#define READ_CONTROL( iobase )          in( iobase+CR )
#define WRITE_CONTROL( iobase , val )   out(val,iobase+CR)
#define READ_STATUS(iobase)             in( iobase+SR )
#define WRITE_STATUS(iobase ,val )      out(val,iobase+SR)
#define READ_DATA(iobase)               in( iobase+DR )
#define WRITE_DATA(iobase, val )        out(val,iobase+DR)
/*--------------------------------------------------------------------------*/
#ifdef  __ECP_COMPAT_MACROS__

#define ECP_ECR( iobase )  		in( iobase+ECR  )
#define ECP_CFGA( iobase )  		in( iobase+CFGA )
#define ECP_CFGB( iobase )  		in( iobase+CFGB )
/*--------------------------------------------------------------------------*/
#define ECP_SET_RLE( iobase ) 	out(ECP_CFGB(iobase)|ECP_RLE,iobase+CFGB)
#define ECP_CLEAR_RLE( iobase) 	out(ECP_CFGB(iobase) & ~ECP_RLE,iobase+CFGB)
/*--------------------------------------------------------------------------*/
#define ECP_SRVC_INTR(iobase)		(ECP_ECR(iobase) & SRVC_INTR)
#define ECP_CLEAR_SRVC_INTR(iobase)     out(ECP_ECR(iobase) & ~SRVC_INTR,iobase+ECR)
#define ECP_SET_SRVC_INTR(iobase)       out(ECP_ECR(iobase) | SRVC_INTR,iobase+ECR)
#define DISABLE_SRVC_INTR(iobase)       ECP_SET_SRVC_INTR(iobase)
#define ENABLE_SRVC_INTR(iobase)        {ECP_SET_SRVC_INTR(iobase);ECP_CLEAR_SRVC_INTR(iobase);}
/*--------------------------------------------------------------------------*/
#define ECP_FIFO_FULL( iobase )         (ECP_ECR(iobase) & FIFO_FULL)
#define ECP_FIFO_EMPTY( iobase )	(ECP_ECR(iobase) & FIFO_EMPTY)
/*--------------------------------------------------------------------------*/
#define FIFO_WRITE_BYTE( iobase,data )          out((data),((iobase)+DFIFO))
#define FIFO_READ_BYTE( iobase )	        in(((iobase)+DFIFO))
#define FIFO_WRITE_WORD( iobase , data )        outw((data),((iobase)+DFIFO))
#define FIFO_READ_WORD( iobase )		inw(((iobase)+DFIFO))
#define FIFO_WRITE_DWORD( iobase , data )	outl((data),((iobase)+DFIFO))
#define FIFO_READ_DWORD( iobase )		inl(((iobase)+DFIFO))
/*--------------------------------------------------------------------------*/
#define DISABLE_INTERRUPTS( iobase )    out((in(iobase+CR) &(~CR_IRQ)),iobase+CR)
#define ENABLE_INTERRUPTS( iobase )     out((in(iobase+CR) | CR_IRQ) ,iobase+CR)
#define ENABLE_ECP_DMA( iobase )        out(ECP_ECR(iobase)|DMA_ENABLE , iobase+ECR )
#define DISABLE_ECP_DMA( iobase )       out(ECP_ECR(iobase)&(~DMA_ENABLE) , iobase+ECR )
/*--------------------------------------------------------------------------*/

#endif /* #ifdef __ECP_COMPAT_MACROS__ */

/** This is 1s for irqs 5,7,9,10,11,14,15
  */
#ifndef ALL_IRQS
#  define ALL_IRQS 0xCEA0
#endif

/** This are all DMA channs but the CASCADE(4)
  */
#ifndef ALL_DMAS
#  define ALL_DMAS 0xEF
#endif
/** But Normaly
  * DMA 2 is used by the FDD
  * DMA 0 is also not used by ECP HWD
  */
#ifndef ECP_DMAS
#  define ECP_DMAS 0xEA
#endif

#ifndef __IRQ_LATENCY__
#  define __IRQ_LATENCY__  0
#endif

#ifndef __DMA_LATENCY__
#  define __DMA_LATENCY__ 0
#endif

/*--------------------------------------------------------------------------*/
#define	IRQ_TRIG_LEVEL	"LEVEL"
#define IRQ_TRIG_EDGE   "EDGE"
#define GEN_YES		"YES"
#define GEN_NO		"NO"

#if !defined( __KERNEL__ ) && !defined( KERNEL ) && !defined ( NDIS_VERSION )
#  define strIRQ_TRIGGER( ConfigA ) (ConfigA & IRQ_TRIGGER)?IRQ_TRIG_LEVEL:IRQ_TRIG_EDGE
#  define strPIPE( ConfigA )	  (ConfigA & HRECOVER_PIPE)?GEN_YES:GEN_NO
#  define strRLE( iobase )	  RLE_Supported(iobase)?GEN_YES:GEN_NO
#  define strIRQ_Conflict( iobase ) IRQ_Conflict(iobase)?GEN_YES:GEN_NO
#endif
/*--------------------------------------------------------------------------*/

/** Claimed Resources
  */
#define ECPDEV_IOBASE_CLAIMED   0x1
#define ECPDEV_IOEXT_CLAIMED    0x2
#define ECPDEV_IRQ_CLAIMED      0x4
#define ECPDEV_DMA_CLAIMED      0x8

/* IRQ & DMA detection flags
  */
#define ECPDEV_IRQ_AUTOPROBED	0x10 
#define ECPDEV_IRQ_MISMATCH	0x20
#define ECPDEV_DMA_AUTOPROBED	0x40
#define ECPDEV_DMA_MISMATCH	0x80

/* Suppprted modes
 */
#define ECPDEV_SMODE_SPP        0x0100
#define ECPDEV_SMODE_PS2        0x0200
#define ECPDEV_SMODE_EPP        0x0400
#define ECPDEV_SMODE_ECP        0x0800
#define ECPDEV_SMODE_ALL        0x0f00

/** Misc ECP Related flags
  */
#define ECPDEV_ECR_BUGGY        0x1000
#define ECPDEV_TX_PIPE          0x2000
#define ECPDEV_IRQ_TRIGGER      0x4000 /** if set- PCI( Level Triggered ) else ISA Pulses ( Edge Triggered ) */
#define ECPDEV_RLE_SUPPORTED    0x8000

typedef struct {
        __u16   iobase          ;
        __u16   ioextent        ;
        __u16   irq             ;
        __u16   dma             ;
        __u16   pword           ;
        __u16   fifo_depth      ;
        __u16   write_intr_thr  ;
        __u16   read_intr_thr   ;
        __u16   flags           ;
        __u16   mode            ;
        __u16   phase           ;
        struct{
                __u8    data    ;
                __u8    status  ;
                __u8    control ;
                __u8    configA ;
                __u8    configB ;
                __u8    econtrol;
        }       hwd             ;
} ecp_dev ;

extern  __inline __u8   read_control    ( ecp_dev *dev );
extern  __inline __u8   read_econtrol   ( ecp_dev *dev );
extern  __inline __u8   read_control_hw ( ecp_dev *dev );
extern  __inline void   write_control   ( ecp_dev *dev , __u8 val );
extern  __inline void   write_econtrol  ( ecp_dev *dev , __u8 val );
extern  __inline __u8   frob_control    ( ecp_dev *dev , __u8 mask , __u8 val );
extern  __inline __u8   frob_econtrol   ( ecp_dev *dev , __u8 mask , __u8 val );
extern  __inline __u8   frob_control_hw ( ecp_dev *dev , __u8 mask , __u8 val );
extern  __inline __u8   frob_econtrol_hw( ecp_dev *dev , __u8 mask , __u8 val );
extern  __inline void   disable_interrupt(ecp_dev *dev );
extern  __inline void   enable_interrupt( ecp_dev *dev );
extern  __inline void   set_Xflag       ( ecp_dev *dev );
extern  __inline void   clear_Xflag     ( ecp_dev *dev );

extern  __inline __u8   fifo_read_byte  ( ecp_dev *dev );
extern  __inline __u16  fifo_read_word  ( ecp_dev *dev );
extern  __inline __u32  fifo_read_long  ( ecp_dev *dev );
extern  __inline __u32  fifo_read_pword ( ecp_dev *dev );

extern  __inline void   fifo_write_byte ( ecp_dev *dev , __u8  val );
extern  __inline void   fifo_write_word ( ecp_dev *dev , __u16 val );
extern  __inline void   fifo_write_long ( ecp_dev *dev , __u32 val );
extern  __inline void   fifo_write_pword( ecp_dev *dev , __u32 pword);

extern  __inline __u8   fifo_empty      ( ecp_dev *dev );
extern  __inline __u8   fifo_full       ( ecp_dev *dev );

extern  __inline void   enable_srvc_intr( ecp_dev *dev );
extern  __inline void   disable_srvc_intr(ecp_dev *dev );
extern  __inline void   set_srvc_intr   ( ecp_dev *dev );
extern  __inline void   clear_srvc_intr ( ecp_dev *dev );

extern  __inline void   enable_ecp_dma  ( ecp_dev *dev );
extern  __inline void   disable_ecp_dma ( ecp_dev *dev );

extern  __inline void   ecp_open        ( ecp_dev *dev );
extern  __inline void   ecp_close       ( ecp_dev *dev );
extern  __inline __u8   set_mode_idle   ( ecp_dev *dev );
extern  __inline __u8   set_mode_test   ( ecp_dev *dev );
extern  __inline __u8   set_mode_config ( ecp_dev *dev );
extern  __inline __u8   aquire_ecp_bus  ( ecp_dev *dev );
extern  __inline __u8   set_mode_rdata  ( ecp_dev *dev , __u32 timeout );
extern  __inline __u8   set_mode_fdata  ( ecp_dev *dev , __u32 timeout );
extern  __inline __u8   fifo_send_complete(ecp_dev *dev, __u32 timeout );
extern  __inline __u8   wait_fdata_termination( ecp_dev *dev , __u32 timeout );

extern          void    getPCAPS ( ecp_dev *dev );
extern	        int	ecp_probe( ecp_dev *dev );
extern          int     irq_autoprobe( __u16 irqs , ecp_dev *dev );
extern          int     dma_autoprobe( __u16 dmas , ecp_dev *dev );
#ifdef FIFO_TEST_SUPPORT
extern		int	fifo_falling_blind(ecp_dev* dev,__u32 pattern) ;
#endif
//#define STATUS_IDLE     0xe0
#define STATUS_IDLE     0x60
//#define STATUS_ACK_RRQ  0xd0
#define STATUS_ACK_RRQ  0x50
//#define STATUS_RRQ      0xc0
#define STATUS_RRQ      0x40
#define STATUS_RDATA    0x50
//#define STATUS_RDATA    0xd0

//#define STATUS_BUS_LOCK 0xc0
#define STATUS_BUS_LOCK 0x40

#define E_MODE_OK               0
#define E_MODE_RQ_PADDING       1
#define E_MODE_RQ_COLLISION     2
#define E_MODE_RQ_NACK          3
#define E_MODE_RQ_TIMEOUT       4
#define E_MODE_REMOTE_DOWN      5
#define E_MODE_TERM_TIMEOUT     6

#define ECONTROL(dev )                  ( dev->ioextent+PCI_ECR  )
#define CONFIGA( dev )                  ( dev->ioextent+PCI_CFGA )
#define CONFIGB( dev )                  ( dev->ioextent+PCI_CFGB )
#define DATAFIFO(dev )                  ( dev->ioextent+PCI_DFIFO)
#define ADDRFIFO(dev )                  ( dev->ioextent+PCI_AFIFO)
/*--------------------------------------------------------------------------*/
/** Facility error codes returned by ECP_probe()
  */
#define EOK             0
#define EMODE_CFG       1
#define EMODE_PS2R      2
#define EMODE_TEST      3
#define EREAD_INTR      4
#define EWRITE_INTR     5
#define EFIFO_WRITE     6
#define EFIFO_READ      7
#define EFIFO_WR_DIFF   8

#define MK_FACILITY_ERR(FACILITY,ERR_CODE)      ((ERR_CODE << 8) | FACILITY )
#define FACILITY_FROM_ERR( ERR_CODE )           (ERR_CODE & 0xff )
#define EXT_ERR_FROM_ERR(ERR_CODE)              (ERR_CODE >> 8 ) 
/*--------------------------------------------------------------------------*/
#endif /* !defined _ECP_H  */
