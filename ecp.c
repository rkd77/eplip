/***************************************************************************
			  ecp.c  -  ECP devices related routines
			     -------------------
        begin           : Thu Nov 9 2000
        last change     : Sun May 20 2001
	version		: 0.3
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
        v.0.1A  This is the initial version. Macro definitions and code moved
		from ppdebug/main.cpp into this file and the ecp.h header file
        v.0.2   Clean up. Formating according to linux/doc/CodingStyle.txt
                        ( Kernel hackers coding style )
                Inline functions.
                Added ecp_probe() to perform simplified probings
                        ( usefull for the kernel mode driver )
                JavaDoc style comments.
                        ( Usefull to extract docs from srcs using DoxyGen )
                ioport.h-v.0.2 Adaptation

BUGS:   Too many to be listed ;-)

***************************************************************************/

#if ( defined( LINUX ) || defined ( linux ) ) && defined( __KERNEL__ )
#  define __NO_VERSION__
#  include <linux/autoconf.h>
#  include <linux/module.h>
#  include <linux/kernel.h>
#  include <linux/sched.h>
#  include <linux/interrupt.h>
#  include <linux/delay.h>
#  include <asm/system.h>
#  include <asm/irq.h>
#  include <asm/dma.h>
#endif

#include "ecp.h"

/*--------------------------------------------------------------------------*/
/** @func       ecr_present( ecp_dev *dev )
  * @brief      Checks for ECP Extended Control Register
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @return     __u8    Non zero if ECR is present.
  * @remark     Old style XT ports alias io ports every 0x400, hence accessing ECR
  *             on these cards actually accesses the CTR.
  *             Modern cards don't do this but reading from ECR will return 0xff
  *             regardless of what is written there if the card does NOT support ECP.
  * @pre        The ECP Hardware must be in COMPATIBLE mode.( SPP mode ).
  */
__inline __u8
ecr_present( ecp_dev *dev )
{
        __u8 r;

        write_control( dev, 0xc );
        r = read_control_hw( dev );
        if( ( read_econtrol( dev ) & 0x3 ) == ( r & 0x3 ) ) {
                write_control( dev , (__u8)(r ^ 0x2) ); /** Toggle bit 1 of CR                  */
                r = read_control_hw( dev );
		if( (read_econtrol( dev ) & 0x2 ) == ( r & 0x2 ) )
		        goto no_reg;                    /** Sure that no ECR register exists    */
	};

	if( (read_econtrol( dev ) & 0x3 ) != 0x1 )
		goto no_reg;

	frob_control( dev , 0xe0, 0 );         /** Go to mode 000; SPP, reset FIFO     */

	return 1;

no_reg:
	out(0xc,dev->iobase+CR);
	return 0;
};
/*--------------------------------------------------------------------------*/
/** @func       is_IEEE1284( ecp_dev *dev )
  * @brief      Checks the Hardware is IEEE1284-94 comformant
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @return     __u8 Non zero if IEEE1284-94 HWD
  * @remark     This routine calls ECR_present() and if ECR register
  *             tries to modify FIFO FULL/EMPTY bits which should be
  *             read-only if IEE1284 compliant  ECP hardware is present.
  * @pre        The Hardware must be in COMPATIBLE( SPP mode )
  */
__inline __u8
is_IEEE1284( ecp_dev *dev )
{
        __u8 ecr;

/** Test for ECR presence
  */
        if(!ecr_present( dev ) )
                return 0;

/** Read the Extended Configuration Register
  */
        ecr = read_econtrol( dev );

/** Test if FIFO_EMPTY=1 & FIFO_FULL=0
  */
        if( !( ecr & FIFO_EMPTY) || ( ecr & FIFO_FULL ) )
                return 0;

/** Test if FIFO FULL/EMPTY bits can be changed
  * They are read-only on the ECP HWD.
  */
        frob_econtrol( dev , FIFO_EMPTY|FIFO_FULL , 0|FIFO_FULL );
        ecr=read_econtrol( dev );
        if( !( ecr & FIFO_EMPTY) || ( ecr & FIFO_FULL ) )
                return 0;

        return 1;
};
/*--------------------------------------------------------------------------*/
__inline __u8
ECP_reset( __u16 iobase )
{
        __u8 ECReg;

        out(0x00,iobase+DR);
        out(0x14,iobase+ECR);
        ECReg=in(iobase+ECR);
        if(ECReg & ~(FIFO_FULL|FIFO_EMPTY|SRVC_INTR))
                return 0;
        return 1;
};
/*--------------------------------------------------------------------------*/
__inline __u8
set_mode_fdata( ecp_dev *dev , __u32 timeout )
{
        __u8 status = READ_STATUS( dev->iobase );

        if( (status & 0xf0 )!= STATUS_IDLE ) {
/** Handle Here RQ_PADDING conditions
  */
                return E_MODE_RQ_COLLISION;
        }

        frob_control( dev , CR_STB , CR_STB );

        status = READ_STATUS( dev->iobase );

        if( (status & 0xf0 )!= STATUS_IDLE ) {
/** Handle Here RQ_COLLISION condition
  */

                return E_MODE_RQ_COLLISION ;
        }

        frob_control( dev , CR_STB, 0 );
        while( timeout--) {
                status = READ_STATUS( dev->iobase );
                if( ( status & 0xf0 ) == STATUS_ACK_RRQ ) {

                        write_econtrol( dev , MODE_PS2|SRVC_INTR|IRQ_DISABLE );
                        frob_control  ( dev , CR_DIR|CR_IRQ|CR_SLCT , 0|0|0  );
                        write_econtrol( dev , MODE_ECP_FIFO|SRVC_INTR|IRQ_DISABLE );
                        return E_MODE_OK ;
                }
                udelay(1);
        }

        return  E_MODE_RQ_TIMEOUT ;
}

__inline __u8
aquire_ecp_bus( ecp_dev *dev )
{
        __u8 status ;

        status = READ_STATUS( dev->iobase ) & 0xf0 ;
        if( status == STATUS_IDLE ) {
                frob_control( dev , CR_INIT , 0 );
                status = READ_STATUS ( dev->iobase ) & 0xf0;
                if( status == STATUS_IDLE )
                        return E_MODE_OK ;
        }
        frob_control( dev , CR_INIT , CR_INIT );
        switch( status ) {
                case STATUS_BUS_LOCK:
                case (STATUS_BUS_LOCK &(~SR_ACK)):
                        if( read_control( dev ) & CR_INIT )
                                return E_MODE_RQ_PADDING;
                        else
                                return E_MODE_RQ_COLLISION;

                default: break;
        };
        return E_MODE_REMOTE_DOWN;
}

__inline __u8
wait_fdata_termination( ecp_dev *dev , __u32 timeout )
{
        __u8 sr ;
        while( timeout-- ) {
                sr = READ_STATUS(dev->iobase) & 0xf0 ;
                if( sr == STATUS_IDLE )
                        return E_MODE_OK ;
                udelay(1);
        }
        return E_MODE_TERM_TIMEOUT ;
}
__inline __u8
set_mode_rdata( ecp_dev *dev , __u32 timeout )
{
        __u8 sr = READ_STATUS(dev->iobase);
        if( ( sr & 0xf0 ) != STATUS_RRQ ) {
                return E_MODE_RQ_NACK ;
        }

        frob_control( dev , CR_INIT|CR_SLCT|CR_IRQ , 0|0|0);

        sr = READ_STATUS( dev->iobase ) ;
        while( ( sr & 0x70 ) != STATUS_RDATA) {
                udelay(1);
                if( !(timeout--) ) {
                        frob_control( dev , CR_INIT|CR_IRQ , CR_INIT|CR_IRQ );
                        return E_MODE_RQ_TIMEOUT ;
                }
                sr = READ_STATUS( dev->iobase ) ;
        }

        write_econtrol( dev , MODE_ECP_FIFO|SRVC_INTR|IRQ_DISABLE );

        return E_MODE_OK ;

}
/*--------------------------------------------------------------------------*/
__inline __u8
fifo_send_complete(ecp_dev *dev, __u32 timeout )
{
        while( !fifo_empty( dev ) ){
                if( !timeout ) {
                        return 2 ;
                }
                udelay(1);
                timeout--;
        }

/** check the byte send to the FIFO and moved to the
  * tranciever has been actualy sent.
  */
        while( timeout ) {
                __u8 sr = READ_STATUS( dev->iobase ) & 0xf0 ;
                if(  (sr & SR_BUSY ) || !( sr & SR_SLCTIN ) ) {
                        return 0 ;
                }
                udelay(1);
                timeout--;
        }

        return timeout?0:1 ;
}
__inline void
set_rle( ecp_dev *dev )
{
        __u8 cfgb = in( CONFIGB(dev) )| ECP_RLE;
        out( cfgb , CONFIGB(dev));

}
__inline void
clear_rle( ecp_dev *dev )
{
        __u8 cfgb = in( CONFIGB(dev) ) & (~ECP_RLE);
        out( cfgb , CONFIGB(dev));
}
/*--------------------------------------------------------------------------*/
/** @func RLE_supported( __u16 iobase )
  * @brief Checks if the ECP HWD support RLE compression
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @return     __u8
  * @retval      0      - The HWD does not support compression on data writes.
  * @retval     !0      - The HWD supports RLE compression for data writes.
  * @remark     All IEEE1284 hardware supports decompression.Some ECP ports may
  *             also support compression.            
  */ 
__inline __u8
RLE_supported( ecp_dev *dev )
{
/** Togle RLE bit
  */
        set_rle(dev);

/** Check it's set
  */
        if( in(CONFIGB(dev) ) & ECP_RLE){
                clear_rle( dev );
                return 1;
        };

        return 0;
}
/*--------------------------------------------------------------------------*/
/** @func IRQ_conflict( __u16 iobase )
  * @brief      Checks if the ECP HWD support RLE compression
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @return     __u8
  * @retval     !0      - assigned IRQ Line is used by another device
  * @retval      0      - assigned IRQ Line __may__ not be used by another device
  * @remark     If this function returns 0 that does not mean that the assigned
  *             IRQ line is not used by other devices.Generaly, there is now relyable way
  *             using only hwd to check if an IRQ line is used by more than just
  *             one device - so we need some assistance from the Operating Systems kernel.   
  */
__inline __u8
IRQ_conflict( ecp_dev *dev )
{
        __u8 ConfigB    = in(CONFIGB(dev));

        return !(ConfigB & ECP_INT);
};
/*--------------------------------------------------------------------------*/
/** @func       getIRQ( __u16 iobase )
  * @brief      Reads and decodes the IRQ line used by the ECP HWD from the ECP ConfigB
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @return     __u8 The IRQ Line or 0
  * @retval     0 The IRQ line is not SW configurable through ConfigB( Jumper Selected )
  * @pre        The Hardware must be in ECP CONFIG mode
  */
__inline __u8
getIRQ( ecp_dev *dev )
{
/** Read CFGB register
  */
        __u8 ConfigB=in( CONFIGB(dev) );

	switch((ConfigB & ECP_IRQ)>>3){
/** Jumper Selected or just can not be changed via CFGB
  *  ( the case with some NS and UMC MultiIO chips )
  */
                case 0 :        return 0x0;

/** The rest are: IRQ is set and also is programable
  * ( could be set via CFGB )
  */
	        case 1 :        return 0x7;
	        case 2 :        return 0x9;
	        case 3 :        return 0xA;
	        case 4 :        return 0xB;
	        case 5 :        return 0xE;
	        case 6 :        return 0xF;
	        case 7 :        return 0x5;
	};

	return 0;
};
/*--------------------------------------------------------------------------*/
#ifdef ECP_SET_FUNCTIONS_SUPPORT
/** @func       setIRQ( __u16 iobase , __u8 irq )
  * @brief      Sets the IRQ line used by the ECP HWD through ConfigB register
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @param      irq     The IRQ line to set ConfigB to
  * @return     Non zero if succeded,0 if ConfigB does't read same as written
  * @pre        The ECP hardware must be in ECP CONFIG mode
  */
__inline __u8
setIRQ( ecp_dev *dev , __u8 irq )
{
/** Read CFGB register
  */
        __u8 ConfigB=in( CONFIGB(dev) );
        switch( irq ) {
                case 0x5 : irq = 0x7 ;
                case 0x7 : irq = 0x1 ;
                case 0x9 : irq = 0x2 ;
                case 0xA : irq = 0x3 ;
                case 0xB : irq = 0x4 ;
                case 0xE : irq = 0x5 ;
                case 0xF : irq = 0x6 ;
                default  : return 0  ;
        }

        ConfigB &= ~ECP_IRQ ;
        ConfigB |= irq<< 3  ;
        out( ConfigB , CONFIGB(dev) );

        return ConfigB==in(CONFIGB(dev))?1:0;
}
#endif
/*--------------------------------------------------------------------------*/
/** @func       getDMA( __u16 iobase )
  * @brief      Reads and Decodes the DMA channel number used by the ECP HWD from ConfigB
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @return     __u8    The DMA channel or 0
  * @pre        The ECP hardware must be in ECP CONFIG mode
  */
__inline __u8
getDMA( ecp_dev *dev )
{
        __u8 ConfigB    = in( CONFIGB(dev) );

	return ( ConfigB & ECP_DMA );

/** @note :     0 is "Jumpered 8-bit Channel" and
  *             4 is "Jumpered 16-bit Channel"
  * This means DMA 0 is never used and in neither case
  * is the DMA chanel programable ( could not be set through CFGB )
  * This doesn't exactly mean that the DMA channel selection is hardwired
  * ( again , some NS and UMC MIO chips do not allow setting DMA through
  * CFGB, and probably it's not safe to reconfigure it using the MIO regs
  * once it's set up- by PC-BIOS /  at least it's not portable,
  * since MIO CFG regs. are hardware specific / )
  */
};
/*--------------------------------------------------------------------------*/
#ifdef ECP_SET_FUNCTIONS_SUPPORT
/** @func       setDMA( __u16 iobase , __u8 dma )
  * @brief      Sets the DMA channel used by the ECP HWD through ConfigB register
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @param      dma     The DMA channel to set ConfigB to
  * @return     Non zero if succeded,0 if ConfigB does't read same as written
  * @pre        The ECP hardware must be in ECP CONFIG mode
  */
__inline __u8
setDMA( ecp_dev *dev, __u8 dma )
{
        __u8 ConfigB = in( CONFIGB(dev) );

        if( (dma > 0) && (dma <= 7) && (dma!=4) ){
                ConfigB &= ~ECP_DMA ;
                ConfigB |=dma ;

                return ConfigB==in( CONFIGB(dev) )?1:0;
        }
        else
                return 0;
}
#endif
/*--------------------------------------------------------------------------*/
/** @func       getFIFO_DBUS_WITH( __u8 ConfigA )
  * @brief      Decodes ECP ConfigA bits specifying the DATA Bus Width(PWORD SIZE)
  * @param      configA The contents of the ECP ConfigA register
  * @return     the FIFO DATA Bus with in bits.
  */
__inline __u8
getFIFO_DBUS_WIDTH( __u8 configA )
{
        switch(configA & DBUS_WIDTH ){
                case 0x00 :     return 16;
	        case 0x10 :     return 8 ;
	        case 0x20 :     return 32;
	        default   :     return 0 ;      /** Broken HWD */
        };
};
/*--------------------------------------------------------------------------*/
#ifdef ECP_SET_FUNCTIONS_SUPPORT
/** @func       setFIFO_DBUS_WITH( __u16 iobase , __u16 nbits )
  * @brief      Sets the FIFO DATA Bus Width( PWORD SIZE )
  * @param      dev Pointer to ecp_dev structrure - required for the base and ECP io addresses
  * @param      nbits  the PWORD SIZE(8,16 or 32 bits )
  * @return     __u8   Non zero if ConfigA reads same as written( succeded ),itherwise 0
  * @remark     On almost every hardware this routine will fail, since the IEEE1284 does not
  *             require the hardware to be able to change it's PWORD size( and it's too
  *             expencive to implement such behaviour in hardware). However there is known
  *             to be at least one third party hardware vendor PCI add-on ECP Card that
  *             natively has 32 bit FIFO but for some software compatibility reasons the
  *             PWORD SIZE could be set to 8 bit through ConfigA register - This is one of the
  *             most recent "Warp Nine Engineering" ECP cards.The "Warp Nine Engineering" is specialized
  *             in developing high quality parallel hardaware, protocol analizers and test suits for
  *             the "Next Generation" parallel hardware.
  * @pre        The ECP hardware must be in ECP CONFIG mode.
  *
  */
__inline __u8
setFIFO_DBUS_WITH( ecp_dev *dev , __u16 nbits )
{
        register __u8 configA ;
        configA = in(CONFIGA(dev)) & (~DBUS_WIDTH) ;
        switch ( nbits ){
                case 8  : configA |= 0x10  ;break;
                case 16 :                   break;
                case 32 : configA |= 0x20  ;break;
                default : return 0 ;
        }
        out(configA,CONFIGA(dev));

        return ((configA == in(CONFIGA(dev)))?1:0);

}
#endif
/*--------------------------------------------------------------------------*/
/** @func wgetFIFO_DEPTH( ecp_dev* dev , __u32 pattern )
  * @brief      Determines the FIFO Depth in PWORDs quantity by writing a PWORD of "pattern"
                at a time until the FIFO becomes full
  * @param      dev Pointer to ecp_dev structrure - required for the base io address and pword
  * @param      pattern a 32bit pattern to written as a PWORD to the FIFO
  * @return     0       Too many PWORDs were written to the FIFO
  * @retval    -1       Mulfunctional FIFO EMPTY/FULL truggers
  * @retval    -3       Invalid dev->pword value( PWORD SIZE )
  * @pre        The ECP hardware must be in ECP TEST mode and FIFO must be empty
  * @remark     Normally you should use one of the following patterns - 0x55555555 or 0xAAAAAAAA             
  */
int
wgetFIFO_DEPTH(ecp_dev* dev,__u32 pattern)
{
        register __u8 nbytes=0;

        while(!(fifo_full(dev))){
                
                switch( dev->pword ){
                        case 8  :       fifo_write_byte(dev,((__u8)(pattern & 0xff)));
                                        break;
                        case 16 :       fifo_write_word(dev,((__u16)(pattern & 0xffff)));
                                        break;
                        case 32 :       fifo_write_long(dev,pattern);
                                        break;
                        default :       return -3;
                }

 	        if(!(++nbytes))
	                return  0;
        };

/** FIFO_EMPTY & FIFO_FULL are both set if
  * there is a FIFO or tranciver error
  * or for any reason this FIFO is not operational
  */
        if(fifo_empty(dev))
	        return -1;

        return nbytes;
};
/*--------------------------------------------------------------------------*/
/** @func rgetFIFO_DEPTH( ecp_dev* dev , __u32 pattern )
  * @brief      Determines the FIFO Depth in PWORDs quantity by reading a PWORD
                at a time until the FIFO becomes empty
  * @param      dev Pointer to ecp_dev structrure - required for the base io address and pword
  * @param      pattern a 32bit pattern to be used for comparison with the PWORDs read from the FIFO
  * @return     0       Too many PWORDs were read from the FIFO
  * @retval    -1       Mulfunctional FIFO EMPTY/FULL truggers
  * @retval    -2       PWORD read from the FIFO is not same as the pattern
  * @retval    -3       Invalid dev->pword value( PWORD SIZE )
  * @pre        The ECP hardware must be in ECP TEST mode and FIFO must be full
  *             ( PWORDs of "pattern" should be written to the fifo prior to calling)
  * @remark     This is the inverse of wgetFIFO_DEPTH().First the wgetFIFO_DEPTH() is
  *             called to fill up the FIFO with the desired pattern PWORDs than this routine
  *             is called to read these PWORDs from the FIFO. The parameter "pattern" should be
  *             same and both routines should return the same positive number if the FIFO is operational.
  */
int
rgetFIFO_DEPTH(ecp_dev* dev,__u32 pattern)
{
        __u8 nbytes=0;

        while(!(fifo_empty(dev))) {
                switch( dev->pword ){
                        case 8  :       if (fifo_read_byte(dev)!=((__u8)(pattern & 0xff)) )
	                                        return -2;
                                        break;
                        case 16 :       if (fifo_read_word(dev)!=((__u16)(pattern & 0xffff)) )
	                                        return -2;
                                        break;
                        case 32 :       if (fifo_read_long(dev)!=pattern )
	                                        return -2;
                                        break;
                        default :       return -3 ;
                }
	        if(!(++nbytes))
	                return 0;
        };
/** FIFO_EMPTY & FIFO_FULL are both set if
  * there is a FIFO or transceiver error
  * or for any reason this FIFO is not operational
  */

        if(fifo_full(dev))
	        return -1;

        return nbytes;
};
/*--------------------------------------------------------------------------*/
/** @func   getWRITE_INTR( ecp_dev* dev )
  * @brief  Test the FIFO's Write Interrupt Threshold
  * @param  dev Pointer to a ecp_dev structure - needed for the  base io address and pword size
  * @return int
  * @retval 0           - The SrvcIntr function doesn't work for writes
  * @retval Positive    - Number of bytes
  * @retval Negative    - Negation of the number of bytes, but ECR is buggy
  *
  * A Service Interrupt is generated whenever
  * there are WRITE_INTR bytes free in the FIFO
  * This doesn't mean you'll get continuos
  * interrupts if the FIFO is empty and you don't do anything :)
  * This means if you write FIFO_DEPTH bytes to the FIFO and
  * WRITE_INTR bytes are delivered to the peripheral
  * you get a Service Interrupt. The Service Interrupt event
  * will actually trigger the IRQ line if SrvcIntr were enabled.
  * @note This is not the only case of SrvcIntr's
  *       Receiving and/or DMA'ing from to the FIFO also generates SrvcIntrs.
  *
  * @remark     ECR register is reffered to as <b>"buggy"<b>
  *             if reading ECR clears the SrvcIntr bit @link buggy_ecr
  *
  * @pre        The hardware must be placed in ECP_TEST mode with
  *             Service Interrupts disabled prior to calling this routine.
  *             Also the FIFO must be empty.
  *
  * @post       The hardware is left in ECP_TEST mode with
  *             Service Interrupts enabled and empty FIFO
  *
  */
int
getWRITE_INTR( ecp_dev* dev )
{
        int words=0;

/** Fill up the FIFO
  */
        while(!fifo_full(dev))
                fifo_write_pword( dev , 0x00 ) ;

/** Toggle the SrvcIntr bit in ECR to enable ServiceInterrupts
  */
        enable_srvc_intr(dev);

/** Read from the FIFO until SrvcIntr in the ECR became set
  */
        while(!( read_econtrol(dev) & SRVC_INTR ) ) {

                fifo_read_pword( dev );
	        words++;
/** If we overflow there is something wrong
  * with this HWD and SrvcIntr bit never toggles
  */
                if( !words )
                        break ;
        };

#if HWD_DEBUG >= 1
/** On This HWD SrvcIntr bit is cleared on reading ECR
  */
        if(!( read_econtrol(dev) & SRVC_INTR ) )
                return -words ;
#endif
        
        return words;
};
/*--------------------------------------------------------------------------*/
/**
  * @func   getREAD_INTR( ecp_dev* dev )
  * @brief  Test the FIFO's Read Interrupt Threshold
  * @param  dev Pointer to a ecp_dev structure - needed for the  base io address and pword size
  * @return int
  * @retval 0           - The SrvcIntr function doesn't work for reads
  * @retval Positive    - Number of bytes
  * @retval Negative    - Negation of the number of bytes, but ECR is buggy
  *
  * READ_INTR is similar to  WRITE_INTR ,
  * but it's generated whenever a peripheral has
  * successfully transfered to the Host as many bytes
  * as there are READ_INTR bytes in the FIFO ready
  * for reading by the Host Driver from the FIFO.
  *
  * @pre        The hardware must be placed in ECP_TEST mode with
  *             Service Interrupts disabled prior to calling this routine.
  *             Also the FIFO must be empty.
  *
  * @post       The hardware is left in ECP_TEST mode with
  *             Service Interrupts enabled and empty FIFO
  *
  * @see getWRITE_INTR
  */
int
getREAD_INTR( ecp_dev* dev )
{
        int words=0;

/** Toggle the SrvcIntr bit in ECR to enable ServiceInterrupts
  */
        enable_srvc_intr(dev);

/** Write to the FIFO until SrvcIntr in the ECRECP_reset( dev[0]->iobase ); became set
  */
        while(!( read_econtrol(dev) & SRVC_INTR ) ) {

                fifo_write_pword(dev,0x00);
	        words++;
/** If we overflow there is something wrong
  * with this HWD and SrvcIntr bit never toggles
  */
                if( !words )
                        break ;

        };

#if HWD_DEBUG >=1
/** On This HWD SrvcIntr bit is cleared on reading ECR
  */
        if(! ( read_econtrol(dev) & SRVC_INTR ) )
                words= -words ;
#endif

/** Empty the FIFO
  */
        while(!fifo_empty(dev))
                fifo_read_pword(dev);


        return words;
};
/*--------------------------------------------------------------------------*/
#ifdef FIFO_TEST_SUPPORT
/** @func   fifo_falling_blind(ecp_dev* dev , __u32 pattern )
  * @brief  Test the ECP FIFO buffer is functional using the well known "Falling Blind" algorithm
  * @param  dev Pointer to a ecp_dev structure - needed for the  base io address and pword size
  * @param  pattern The bit pattern for the "Falling Blind"
  * @return int Number of bytes written/read or error code.
  * @retval  positive   - number of bytes written/read
  * @retval  0          - two many pword written/read
  * @retval -1          - Mulfunctional FIFO FULL/EMPTY triggers
  * @retval -2          - PWORD writtent and read differ
  * @retval -3          - Number of bytes written differ from bytes read
  * @retval -4          - Invalid PWORD size(dev->pword!=8,16,32)
  *
  * @remark     Normally you should call this routine with patterns
  *             000000 or 0x55555555(0xFFFFFFFF or 0xAAAAAAAA)
  *
  * @pre        The hardware must be placed in ECP_TEST mode with
  *             Service Interrupts disabled prior to calling this routine.
  *             Also the FIFO must be empty.
  *
  * @post       The hardware is left in ECP_TEST mode with
  *             Service Interrupts disabled and empty FIFO
  */
int
fifo_falling_blind( ecp_dev* dev,__u32 pattern)
{

        __u8 nbytes_write=0;
        __u8 nbytes_read=0;
        __u32 mask;

        switch(dev->pword){
                case 8  :       mask = 0xff     ; break;
                case 16 :       mask = 0xffff   ; break;
                case 32 :       mask = 0xffffffff;break;
                default :       return -4 ;
        };
/** Write a pword to the FIFO until FIFO_FULL bit toggles
  */
        while(!(fifo_full(dev))) {

                if(nbytes_write & 1)
                        fifo_write_pword(dev,pattern);
                else
                        fifo_write_pword(dev,~pattern);

                if(!(++nbytes_write))           /** to many pwords were written  */
	                return  0;
        };

/** if FIFO_FULL & FIFO_EMPTY are both set =>  error condition
  */
        if(fifo_empty(dev))
                return -1;

/** Read a pword from the FIFO until FIFO_EMPTY bit toggles
  */
        while(!fifo_empty(dev)) {

                if(nbytes_read & 1){
                        if((fifo_read_pword(dev))!=( pattern & mask ) )
	                        return -2;
                }
                else
		        if((fifo_read_pword(dev))!=( ( ~pattern ) & mask ) )
	                        return -2;

                if(!(++nbytes_read))    /* to many pwords were read */
                        return 0;
        };

/** if FIFO_FULL & FIFO_EMPTY are both set =>  error condition
  */
        if(fifo_full(dev))
                return -1;

/** Number of pwords writen should be same as number of bytes read from the FIFO
  */
        if( nbytes_write!=nbytes_read )
                return -3;

  return nbytes_read;
}
#endif
/*--------------------------------------------------------------------------*/
/** @func       ecp_probe( ecp_dev* dev )
  * @brief      Probes the ECP HWD properties and capabilities
  * @param      dev     A pointer to ecp_dev structure.dev->iobase should be set to
  *                     the base io address and dev->ioextent to the base io address
  *                     of the ECP extended register set
  * @return     int     An Error Code
  * @retval      0      Success
  * @retval     !0      Probing failed at some point.The LSB byte is a facility code
  *                     For some facilities codes there is an extended error code in
  *                     the MSB 3 bytes. See @link facility_error_codes.
  */            
int
ecp_probe ( ecp_dev *dev )
{
        __u8 configA ;
        int  error = EOK ;

/** Switch to ECP_CONFIG mode
  */
	if( set_mode_config( dev )!=0 )
	        return EMODE_CFG ;

/** Read CONFIGURATION REGISTER A
  */
        configA = in(CONFIGA( dev )) ;

/** Check IRQ Trigger scheme
  */
        if( configA & IRQ_TRIGGER )
                dev->flags |= ECPDEV_IRQ_TRIGGER ;

        dev->pword = getFIFO_DBUS_WIDTH(configA);

/** Check if Transmitter byte affects FIFO_FULL/FIFO_EMPTY bits
  */
        if( configA & HRECOVER_PIPE )
                dev->flags |= ECPDEV_TX_PIPE ;

/** Check if RLE commpression is supported by this HWD
  */
        if( RLE_supported( dev ) )
                dev->flags |= ECPDEV_RLE_SUPPORTED ;

/** Try to get IRQ through CFGB register
  */
        dev->irq=getIRQ( dev ) ;

/** Try to get DMA channel trough CFGB register
  */
        dev->dma=getDMA( dev ) ;

/** Test FIFO depth and FIFO is functional
  */
        while( 1 ) {

                int bread;
                int bwritten;

	        set_mode_test( dev );

                if( ( bwritten = wgetFIFO_DEPTH( dev , 0x55555555 ) ) > 0 )
                        dev->fifo_depth = (__u16)bwritten ;
                else {
                        error =  MK_FACILITY_ERR(EFIFO_WRITE,bwritten);
                        break ;
                }
	        if( ( bread = rgetFIFO_DEPTH( dev , 0x55555555 ) ) > 0 ) {
                        if ( bread != bwritten ) {
                                error = MK_FACILITY_ERR(EFIFO_READ,bread);
                                break ;
                        }
                }
                else{
                        error =  EFIFO_WR_DIFF ;
                        break ;
                }

/** Test WRITE_INTR_THRESHOLD, READ_INTR_THRESHOLD
  * Normaly if FIFO_DBUS_WIDTH is 1 byte they are same as FIFO_DEPTH
  */
                
                bwritten = getWRITE_INTR( dev );
                if( bwritten ){
                        if ( bwritten < 0 ) {
                                dev->flags |= ECPDEV_ECR_BUGGY ;
                                dev->write_intr_thr= ( __u16 )(-bwritten ) ;
                        }
                        else {
                                dev->write_intr_thr= ( __u16 ) bwritten ;
                        }
                }
                else {
                        error = MK_FACILITY_ERR(EWRITE_INTR,bwritten);
                        break;
                }

                write_econtrol( dev , ( MODE_PS2|IRQ_DISABLE|SRVC_INTR ) );
                if( ( read_econtrol( dev ) & ( MODE_ALL|IRQ_DISABLE|SRVC_INTR ) ) !=(MODE_PS2|IRQ_DISABLE|SRVC_INTR) ) {
                        error = EMODE_PS2R;
                        break ;
                }
                frob_control_hw( dev , CR_DIR,CR_DIR );

                if(set_mode_test( dev )!=0 ) {
                        error = EMODE_TEST;
                        break ;
                }

                bread = getREAD_INTR( dev ) ;
                if( bread ) {
                        if( bread < 0 ) {
                                dev->flags |= ECPDEV_ECR_BUGGY ;
                                dev->read_intr_thr = ( __u16 ) ( -bread )  ;
                        }
                        else {
                                dev->read_intr_thr = ( __u16 ) bread  ;
                        }
                }
                else {
                        error = MK_FACILITY_ERR(EREAD_INTR,bread);
                        break ;
                }

                break;
        };

        ECP_reset( dev->iobase );

        return error ;
}
/*--------------------------------------------------------------------------*/
/** Autoprobes the ECP IRQ
  * @retval 0           No IRQ detected
  * @retval Negative    Multiple IRQs detected
  * @retval Positive    The IRQ Line detected
  */
int
irq_autoprobe( __u16 irqs,ecp_dev* dev )
{
       int irq=0;
       unsigned long irq_flags;

       local_irq_enable();

       irq_flags = probe_irq_on();

/** Forse ECP HWD to trigger an irq
  */
        ECP_reset( dev->iobase );

        set_mode_test( dev );
        enable_srvc_intr( dev );
        
        while(!fifo_full( dev ) ){
                fifo_write_pword( dev , 0 );
        }

        udelay( __IRQ_LATENCY__ );

/** read back actually generated irqs and mask irqs unmasked by autoirq_setup()
  */
        irq = probe_irq_off( irq_flags );

        ECP_reset( dev->iobase );

        return irq;
};


/** Autoprobes the Slave DMA channel used for ECP FIFO DMA trans
  * @retval -1          No DMA detected
  * @retval -2          Multiple DMAs
  * @retval Positive    The DMA channel detected
  */
int
dma_autoprobe( __u16 dmas , ecp_dev* dev )
{

        short dma_status, new_dma_status ,i , dma;

        local_irq_enable();

/** Reset just in case  the FIFO is not empty
  */
        ECP_reset( dev->iobase );


/** Read the DMA channel status registers.
  */
	dma_status = ((in(DMA1_STAT_REG) >> 4) & 0x0f) | (in(DMA2_STAT_REG) & 0xf0);

/** Trigger a DMA request, perhaps pause a bit.
  */
        read_econtrol   ( dev );
        enable_ecp_dma  ( dev );
        clear_srvc_intr ( dev );

        udelay ( __DMA_LATENCY__ );

/** Re-read the DMA status registers.
  */
        new_dma_status = ((in(DMA1_STAT_REG) >> 4) & 0x0f) | (in(DMA2_STAT_REG) & 0xf0);

/**
  * Eliminate the old and floating requests,
  * and DMA4 the cascade.
  * Also ignore channels we're not interested in.
  */
        new_dma_status ^= dma_status;
	new_dma_status &= ~0x10;
        new_dma_status &= dmas ;
        dma = -1 ;

	for (i = 0; i < 8; i++) {
	        if ( new_dma_status & 1 ) {
                        if(dma!=-1) {
                                dma = -2 ; /* Multiple DMAs Detected */
                                break ;
                        }
                        else
	                        dma = i;
	        }
                new_dma_status >>=1;
        }

        set_srvc_intr   ( dev );
        disable_ecp_dma ( dev );

        ECP_reset( dev->iobase );

        return dma;
}
/*--------------------------------------------------------------------------*/
__inline __u8
read_control_hw( ecp_dev *dev )
{
        dev->hwd.control = in(dev->iobase+CR) ;
        return dev->hwd.control ;
}
__inline __u8
read_control( ecp_dev *dev )
{
        return dev->hwd.control ;
}
__inline __u8
read_econtrol( ecp_dev *dev )
{
        dev->hwd.econtrol = in(dev->ioextent + PCI_ECR );
        return dev->hwd.econtrol ;
};
__inline void
write_control( ecp_dev *dev , __u8 val )
{
        dev->hwd.control = val ;
        out( val , dev->iobase+CR );
};
__inline void
write_econtrol( ecp_dev *dev , __u8 val )
{
        dev->hwd.econtrol = val ;
        out( val , dev->ioextent + PCI_ECR ) ;
};
__inline __u8
frob_control( ecp_dev *dev , __u8 mask , __u8 val )
{
        dev->hwd.control = ( dev->hwd.control & ~mask )^val ;
        out( dev->hwd.control , dev->iobase+CR );
        return dev->hwd.control;
};
__inline __u8
frob_econtrol( ecp_dev *dev , __u8 mask , __u8 val )
{
        dev->hwd.econtrol = ( dev->hwd.econtrol & ~mask ) ^val ;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
        return dev->hwd.econtrol;
};
__inline __u8
frob_control_hw( ecp_dev *dev , __u8 mask , __u8 val )
{
        dev->hwd.control = in(dev->iobase+CR) ;
        dev->hwd.control = ( dev->hwd.control & ~mask )^val ;
        out( dev->hwd.control , dev->iobase+CR );
        return dev->hwd.control;
}
__inline __u8
frob_econtrol_hw( ecp_dev *dev , __u8 mask , __u8 val )
{
        dev->hwd.econtrol = in( dev->ioextent + PCI_ECR );
        dev->hwd.econtrol = ( dev->hwd.econtrol & ~mask ) ^val ;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
        return dev->hwd.econtrol;
};
__inline void
enable_interrupt( ecp_dev *dev )
{
        if( !( dev->hwd.control & CR_IRQ ) ) {
                dev->hwd.control |= CR_IRQ ;
                out( dev->hwd.control , dev->iobase+CR );
        }
}
__inline void
disable_interrupt( ecp_dev *dev )
{
        if( dev->hwd.control & CR_IRQ ) {
                dev->hwd.control &=~CR_IRQ;
                out( dev->hwd.control , dev->iobase+CR );
        }
}
__inline void
enable_ecp_interrupt( ecp_dev *dev )
{
        dev->hwd.econtrol &= ~IRQ_ENABLE;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
}
__inline void
disable_ecp_interrupt( ecp_dev *dev )
{
        dev->hwd.econtrol |= IRQ_ENABLE;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
}
__inline void
enable_ecp_dma( ecp_dev *dev )
{
        dev->hwd.econtrol |= DMA_ENABLE ;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
}
__inline void
disable_ecp_dma( ecp_dev *dev )
{
        dev->hwd.econtrol &= ~DMA_ENABLE ;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
}
__inline void
clear_srvc_intr( ecp_dev * dev )
{
        dev->hwd.econtrol &=~SRVC_INTR ;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
}
__inline void
set_srvc_intr( ecp_dev * dev )
{
        dev->hwd.econtrol |=SRVC_INTR ;
        out( dev->hwd.econtrol , dev->ioextent + PCI_ECR );
}

__inline void
enable_srvc_intr( ecp_dev *dev )
{
        set_srvc_intr( dev ) ;
        clear_srvc_intr( dev );
}
__inline void
disable_srvc_intr( ecp_dev *dev )
{
        set_srvc_intr( dev ) ;
}
__inline void
set_Xflag( ecp_dev *dev )
{
        frob_control( dev , CR_SLCT , 0 );
}
__inline void
clear_Xflag( ecp_dev *dev )
{
        frob_control( dev , CR_SLCT , CR_SLCT );
}
__inline void
ecp_open( ecp_dev *dev )
{
        read_control_hw( dev );
        write_econtrol( dev , MODE_PS2 | IRQ_DISABLE | SRVC_INTR );

/*                           read     nRevReq            nSTB     nXflag    nBusy                         */
        frob_control ( dev , CR_DIR | CR_INIT | CR_IRQ | CR_STB | CR_SLCT | CR_AUTOFD, CR_DIR | CR_INIT | 0 | 0 | 0 |0 );
}
__inline __u8
set_mode_idle( ecp_dev *dev )
{
        write_econtrol( dev , MODE_PS2 | IRQ_DISABLE | SRVC_INTR );
/*                           read     nRevReq            nSTB     nXflag     nBusy                           */
        frob_control( dev , CR_DIR | CR_INIT | CR_IRQ | CR_STB | CR_SLCT | CR_AUTOFD, CR_DIR | CR_INIT | CR_IRQ | 0 | CR_SLCT |0 );

        return 0 ;
}

__inline void
ecp_close( ecp_dev *dev )
{
        write_econtrol( dev , MODE_PS2 );
        frob_control  ( dev , CR_DIR | CR_INIT | CR_IRQ | CR_STB | CR_SLCT , CR_DIR | CR_INIT | 0 | 0 | 0 );

}
__inline __u8
set_mode_test( ecp_dev *dev )
{
        write_econtrol( dev , MODE_FIFO_TEST|IRQ_DISABLE|SRVC_INTR );
        return 0 ;
}
__inline __u8
set_mode_config( ecp_dev *dev )
{
        write_econtrol( dev , MODE_CONFIG|SRVC_INTR|IRQ_DISABLE );
        return 0 ;
}
__inline __u8
fifo_read_byte( ecp_dev *dev )
{
        return in( dev->ioextent + PCI_DFIFO );
}
__inline __u16
fifo_read_word( ecp_dev *dev )
{
        return inw( dev->ioextent + PCI_DFIFO );
}
__inline __u32
fifo_read_long( ecp_dev *dev )
{
        return inl( dev->ioextent + PCI_DFIFO );
}

__inline void
fifo_write_byte( ecp_dev *dev , __u8 val )
{
        out( val , dev->ioextent + PCI_DFIFO );
}
__inline void
fifo_write_word( ecp_dev *dev , __u16 val )
{
        outw( val , dev->ioextent + PCI_DFIFO );
}

__inline void
fifo_write_long( ecp_dev *dev , __u32 val )
{
        outl( val , dev->ioextent + PCI_DFIFO );
}
/*--------------------------------------------------------------------------*/
__inline void
fifo_write_pword( ecp_dev* dev, __u32 pword )
{
        switch( dev->pword ) {
                case 8  :       fifo_write_byte(dev,((__u8)(pword & 0xff)));
                                break;
                case 16 :       fifo_write_word(dev,((__u16)(pword & 0xffff)));
                                break;
                case 32 :       fifo_write_long(dev, pword);
                                break;
                default :       break;
        };
};
/*--------------------------------------------------------------------------*/
__inline __u32
fifo_read_pword( ecp_dev* dev )
{
        switch( dev->pword ) {
                case 8  :       return fifo_read_byte( dev );
                case 16 :       return fifo_read_word( dev );
                case 32 :       return fifo_read_long( dev );
                default :       return 0xffffffff;
        }
}
__inline __u8
fifo_empty( ecp_dev *dev )
{
        return ( in( dev->ioextent + PCI_ECR ) & FIFO_EMPTY );
}
__inline __u8
fifo_full( ecp_dev *dev )
{
        return ( in( dev->ioextent + PCI_ECR ) & FIFO_FULL );
}

/*--------------------------------------------------------------------------*/
/*
 * Checks for port existence, all ports support SPP MODE
 * Returns:
 *         0            :  No parallel port at this adress
 *         1            :  SPP port detected
 *                        (if the user specified an ioport himself,
 *                         this shall always be the case!)
 *
 */
__inline __u8
SPP_supported( __u16 iobase )
{
	unsigned char r, w;

        /* Do a simple read-write test to make sure the port exists. */
	w = 0xc;
        WRITE_CONTROL(iobase,w);
	
	/* Is there a control register that we can read from?  Some
	 * ports don't allow reads.In addition, some bits aren't
	 * writable. */

        r = READ_CONTROL( iobase );
        if ((r & 0xf) == w) {
		w = 0xe;
		WRITE_CONTROL(iobase ,w );
                r =READ_CONTROL(iobase);
                WRITE_CONTROL(iobase,0xc);
		if ((r & 0xf) == w)
			return 1 ;
	}

        return 0;
}

/* Detect PS/2 support.
 *
 * Bit 5 (0x20) sets the PS/2 data direction; setting this high
 * allows us to read data from the data lines.  In theory we would get back
 * 0xff but any peripheral attached to the port may drag some or all of the
 * lines down to zero.  So if we get back anything that isn't the contents
 * of the data register we deem PS/2 support to be present. 
 *
 * Some SPP ports have "half PS/2" ability - you can't turn off the line
 * drivers, but an external peripheral with sufficiently beefy drivers of
 * its own can overpower them and assert its own levels onto the bus, from
 * where they can then be read back as normal.  Ports with this property
 * and the right type of device attached are likely to fail the SPP test,
 * (as they will appear to have stuck bits) and so the fact that they might
 * be misdetected here is rather academic. 
 */

__inline __u8 
PS2_supported( __u16 iobase )
{
	__u8 ok = 0;
  
	/* try to tri-state the buffer */
	WRITE_CONTROL( iobase , READ_CONTROL(iobase)|CR_DIR);
        if( READ_CONTROL(iobase) & CR_DIR ){

	        WRITE_DATA(iobase , 0x55);

	        if ( READ_DATA(iobase)!= 0x55) ok++;

	        WRITE_DATA(iobase , 0xaa);
	        if (READ_DATA(iobase) != 0xaa) ok++;

	        /* cancel input mode */
	        WRITE_CONTROL( iobase , ( READ_CONTROL(iobase) & (~CR_DIR ) ) );
        }

	return ok;
}

__inline __u8
ECPPS2_supported( ecp_dev* dev )
{
	__u8 result;
	unsigned char ecr;

	ecr = read_econtrol( dev );
        write_econtrol( dev , (MODE_PS2|IRQ_DISABLE|SRVC_INTR) );
        frob_control_hw( dev , CR_DIR , CR_DIR );

	result = PS2_supported( dev->iobase );

	write_econtrol( dev, ecr );
	return result;
}

/*
 * Clear TIMEOUT BIT in EPP MODE
 *
 * This is also used in SPP detection.
 */
static int clear_epp_timeout( __u16 iobase )
{
	unsigned char r;

	if (!(READ_STATUS(iobase) & 0x01))
		return 1;

	/* To clear timeout some chips require double read */
	READ_STATUS(iobase);
	r = READ_STATUS(iobase);
        /* Some reset by writing 1 */
        WRITE_STATUS(iobase , (r | 0x01) ); 
	/* Others by writing 0 */
        WRITE_STATUS(iobase , (r & 0xfe) );
	r = READ_STATUS(iobase);

	return !(r & 0x01);
}

/* EPP mode detection  */
__inline __u8
EPP_supported( __u16 iobase )
{
	/*
	 * Theory:
	 *	Bit 0 of STR is the EPP timeout bit, this bit is 0
	 *	when EPP is possible and is set high when an EPP timeout
	 *	occurs (EPP uses the HALT line to stop the CPU while it does
	 *	the byte transfer, an EPP timeout occurs if the attached
	 *	device fails to respond after 10 micro seconds).
	 *
	 *	This bit is cleared by either reading it (National Semi)
	 *	or writing a 1 to the bit (SMC, UMC, WinBond), others ???
	 *	This bit is always high in non EPP modes.
	 */

	/* If EPP timeout bit clear then EPP available */
	if ( !clear_epp_timeout( iobase ) )
		return 0;  /* No way to clear timeout */

	return 1;
}

__inline __u8
ECPEPP_supported( ecp_dev *dev )
{
	__u8 result;
	unsigned char ecr;

	ecr = read_econtrol( dev );

	/* Search for SMC style EPP+ECP mode */
	write_econtrol( dev , 0x80 );
        WRITE_CONTROL( dev->iobase,0x4);
	result = EPP_supported( dev->iobase );

        write_econtrol( dev , ecr );
	return result;
}
/* Determine Port Capabilities( Supported modes )
 */
void
getPCAPS( ecp_dev* dev )
{
        dev->flags &= ~ECPDEV_SMODE_ALL;

        if( SPP_supported( dev->iobase ) ){

                dev->flags |= ECPDEV_SMODE_SPP ;

                if( is_IEEE1284( dev ) ) {
        
                        dev->flags |= ECPDEV_SMODE_ECP;
                        if( ECPPS2_supported( dev ) )
                                dev->flags|=ECPDEV_SMODE_PS2;
                        if( ECPEPP_supported( dev ) )
                                dev->flags|=ECPDEV_SMODE_EPP;
                }
                else {
                        if( PS2_supported( dev->iobase ) )
                                dev->flags |= ECPDEV_SMODE_PS2;
                /* EPP is never present if BASE IO is 0x3BC
                 * since Extended EPP registers will occupy IO range
                 * reserved for STD VGA controlers
                 */
                        if( (dev->iobase !=0x3BC) && EPP_supported( dev->iobase ) )
                                dev->flags |= ECPDEV_SMODE_EPP;
                }
        }

        ECP_reset( dev->iobase );
}

