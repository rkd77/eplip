
                                /* The necessary header files   */
//#include <linux/autoconf.h>       /* system name and global items */
                                /* Standard in kernel modules   */
#include <linux/module.h>
#include <linux/kernel.h>

/*
 * Then include whatever header you need.
 * Most likely you need the following:
 */
#include <linux/sched.h>        /* current, task_struct, other goodies */
#include <linux/types.h>        /* ulong and friends            */
#include <linux/fcntl.h>        /* O_NONBLOCK etc.              */
#include <linux/interrupt.h>
#include <linux/ptrace.h>
#include <linux/ioport.h>       /* request_region()             */
#include <linux/in.h>           /* inet stuff                   */
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/spinlock.h>
#include <linux/errno.h>        /* return values */
#include <linux/init.h>
#include <linux/delay.h>        /* udelay()                     */
//#include <asm/system.h>         /* cli() sti()                  */
#include <asm/bitops.h>
#include <asm/io.h>             /* inb() inw() outb() ...       */
#include <asm/dma.h>            /* PC DMAC releated stuff       */
#include <asm/irq.h>            /* unreadable, but useful       */

                                /* NET headers                  */
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/inetdevice.h>
#include <linux/skbuff.h>
#include <linux/if_plip.h>
#include <net/neighbour.h>

                                /* Our Driver headers           */
#include "sysdep.h"
#include "ioport.h"
#include "ecp.c"

#ifndef MAX_DMA_PHYS_ADDRESS
#  define MAX_DMA_PHYS_ADDRESS 0x1000000
#endif

static const char driver [] 	="eplip";
static const char version[] 	="0.5.6";
static const char date   []     ="05/16/02";
static const char author []     ="Angel Valkov";
static const char email  []     ="whitefang@dir.bg";


/* Maximum number of devices to support. */
#define EPLIP_MAX  3

/* Use 0 for production, 1 for verification, >2 for debug */
#ifndef NET_DEBUG
#define NET_DEBUG 0
#endif

#ifndef EPLIP_DEBUG
#define EPLIP_DEBUG 0
#endif

#define EPLIP_MTU    (16360+2)
#define PKT_BUF_SZ (EPLIP_MTU+ETH_HLEN)

#define EPLIP_MRU (32740+2)
#define PKT_RCVBUF_SZ (EPLIP_MRU+ETH_HLEN)

#if NET_DEBUG >= 3
#ifndef EPLIP_DEBUG
#define EPLIP_DEBUG 3
#endif
#endif

/* In micro second */
#define EPLIP_DELAY_UNIT        1

/* Connection time out = EPLIP_TRIGGER_WAIT * EPLIP_DELAY_UNIT usec */
#define EPLIP_TRIGGER_WAIT      500

#define EPLIP_FIFO_WAIT        5000*16

#define EPLIP_BYTE_WAIT        5000


#define IO_EXTENT              3
#define IO_EXTENT_BASE         IO_EXTENT
#define IO_EXTENT_ECP          3


#define EPLIP_MODE_NIBBLE       0x0     /** LapLink cabble mode                         */
#define EPLIP_MODE_ECP_BYTE     0x1     /** Fast ECP cable on ECP HWD using BiDi SPP    */
#define EPLIP_MODE_ECP_SPP      0x2     /** Fast ECP cable on ECP HWD using FIFO SPP    */
#define EPLIP_MODE_ECP_FIFO     0x4     /** Fast ECP cable on ECP HWD using FIFO ECP    */
#define EPLIP_MODE_ECP_DMA      0x8     /** Fast ECP cable on ECP HWD using FIFO+DMA ECP*/
#define EPLIP_MODE_TEST         0x10    /** No cable on any HWD - caps and params test  */


#define OK              0
#define TIMEOUT         1
#define ERROR           2
#define HS_TIMEOUT      3

/*****************************************************************************/
/** Device Initialization/Deinitialization Stuff
  */

/** Probes for ECP hardware @iobase dev->iobase
  * Fills up the *dev structure
  * should be OK
  */
static __inline int
simple_probe( ecp_dev  *dev )
{
        int error;
        __u8 dma_mask = ALL_DMAS ;

        if( ( error = ecp_probe( dev ) ) == EOK ) {

#if EPLIP_DEBUG < 3
                if ( dev->mode == EPLIP_MODE_TEST )
#endif
                {
                        printk(KERN_INFO"IRQ           : %u\n",dev->irq           );
                        printk(KERN_INFO"DMA           : %u\n",dev->dma           );
                        printk(KERN_INFO"PWORD         : %u\n",dev->pword         );
                        printk(KERN_INFO"FIFO_DEPTH    : %u\n",dev->fifo_depth    );
                        printk(KERN_INFO"FIFO_TX_PIPE  : %s\n",(dev->flags & ECPDEV_TX_PIPE)      ? GEN_YES:GEN_NO                );
                        printk(KERN_INFO"IRQ_TRIGGER   : %s\n",(dev->flags & ECPDEV_IRQ_TRIGGER)  ? IRQ_TRIG_LEVEL:IRQ_TRIG_EDGE  );
                        printk(KERN_INFO"RLE_SUPPORTED : %s\n",(dev->flags & ECPDEV_RLE_SUPPORTED)? GEN_YES:GEN_NO                );
                        printk(KERN_INFO"WRITE_INTR_THR: %u\n",dev->write_intr_thr);
                        printk(KERN_INFO"READ_INTR_THR : %u\n",dev->read_intr_thr );
#if HWD_DEBUG > 0
                        printk(KERN_INFO"ECR_BUGGY     : %s\n",(dev->flags & ECPDEV_ECR_BUGGY)    ? GEN_YES:GEN_NO                );
#endif
                }

/** If IRQ was not detected through CFGB register
  * or HWD_DEBUG >= 3
  * Autodetect the IRQ
  */

#if HWD_DEBUG < 3
                if( (!dev->irq) || ( dev->mode == EPLIP_MODE_TEST ) )
#endif
                {
                        int irq ;
                        irq = irq_autoprobe( ALL_IRQS, dev ) ;
#if EPLIP_DEBUG < 3
                        if ( dev->mode == EPLIP_MODE_TEST )
#endif
                        {
                                if( irq > 0 ) {
                                        printk(KERN_INFO"AutoIRQ probed: %u\n",irq);
                                        dev->flags|=ECPDEV_IRQ_AUTOPROBED;
                                }

                        }
                        if( irq ) {
                                if( dev->irq && ( dev->irq!=irq ) && (irq!=-1) ){
                                        /** Different AutoIRQ and CFGB IRQ */
                                        printk(KERN_INFO "AutoIRQ probed: %u NOT SAME as ConfigB IRQ(%u).Ignoring it\n",irq,dev->irq);
                                        dev->flags|=ECPDEV_IRQ_MISMATCH;
                                }
                                else
                                        if(irq > 0 )
                                                dev->irq=(__u8)irq ;
                                        else{
                                        /* Multiple IRQs detected */
                                                printk(KERN_INFO "AutoIRQ probed: %s\n","Multiple IRQs Detected");
                                        }
                        }
                        else{
                                /* No IRQs AutoIRQ detected */
                                printk(KERN_INFO "AutoIRQ probed: %s\n","No IRQs Detected");
                        }
                }
dma_paranoid:
#if HWD_DEBUG < 3
                if( (!dev->dma) || ( dev->mode == EPLIP_MODE_TEST ) )
#endif
                {
                        int dma;
                        dma = dma_autoprobe( dma_mask, dev ) ;
#if EPLIP_DEBUG < 3
                        if ( dev->mode == EPLIP_MODE_TEST )
#endif
                        {
                                if( dma > 0 ) {
                                        printk(KERN_INFO "AutoDMA probed: %u\n",dma);
                                        dev->flags|=ECPDEV_DMA_AUTOPROBED ;
                                }
                        }
                        if( dma!=-1 ) {
                                if( (dma!=-2) && dev->dma && ( dev->dma!=dma ) ){
                                        /** Different AutoDMA and CFGB DMA */
                                        printk(KERN_INFO "AutoDMA probed: %u NOT SAME as ConfigB DMA(%u).Ignoring it\n",dma,dev->dma);
                                        dev->flags|=ECPDEV_DMA_MISMATCH;
                                }
                                else
                                        if( dma > -1 )
                                                dev->dma=(__u8)dma;
                                        else {
                                        /* Multiple DMAs detected */
                                                if( dma_mask == ALL_DMAS ) {
                                                        printk(KERN_INFO "AutoDMA probed: %s\n","Multiple DMAs Detected.");
                                                        dma_mask = ECP_DMAS;
                                                        goto dma_paranoid;
                                                }
                                                else
                                                    printk(KERN_INFO "AutoDMA probed: %s\n","Multiple DMAs Detected again.Giving Up");
                                        }
                        }
                        else{
                                /* No DMAs AutoDMA detected */
                                 printk(KERN_INFO "AutoDMA probed: %s\n","No DMAs Detected");
                        }
                }

        }
        else {
#if NET_DEBUG > 0
                printk(KERN_INFO"%s simple_probe() failed wt ecode:%u\n",driver,error );
#endif
        }


        return error ;

};

/*****************************************************************************/
/*      NET related stuff
 */

/** NOTE: As version 0.5.5 EPLIP  is
  *     incompatible with earlier versions at either
  *     the physical and link layers.
  *     The EPILP "HARDWARE" packet header has new
  *     format:
  *             16 bits( 2 octets ) length
  *             16 bits( 2 octets ) flags
  *             32 bits( 4 octets ) checksum
  *
  *     All octets are sent in x86 PCs' native byte order
  *     ( That is LSB first ). The number of octets in
  *     the header is 4 byte alligned, so is the number of
  *     octets in the packets data block. This is to ensure
  *     that if the underlaying ECP hardware has a 32(16) bit
  *     fifo all header and packet data octets could be send
  *     in ECP FIFO mode( or ECP DMA mode for the packet data )
  *     and all bytes sent could be accepted in ECP FIFO(DMA) mode
  *     at the  remote end(if it also is 16(32) bit).
  *     (The ISA ECP specification defines that if odd number of bytes
  *     have to be transmited over an ECP link on ECP ports that have 16/32 bit
  *     FIFO, the software driver should simulate ECP handshaking in either COMPATIBILITY
  *     or BYTE mode to send/receive single bytes , which is not difficult to implement,
  *     but poses a performance penalty, since the transfer mode should be re-negotiatated
  *     - which is entirely software operation[ on ISA Bus all io r/w instructions take at least
  *     1us to execute- if the Bus runs at standard speeds - 8-8.33 Mhz]. This operation requires
  *     much more time that time requires to send/receive additional 1 to 3 bytes
  *     on 8 bit FIFOs ).
  *
  *     The byte order was chosen to be LSB instead of NETWORK( That's MSB first )
  *     since the ECP standard defines that the PWord written to the FIFO is
  *     sent through the 8 lines data interface in LSB order
  *     ( That's if you issue an "outw" instruction on a x86 based machine
  *     to the ECP FIFO the first octet that will apper on data lines will be the low
  *     byte of the word written.).
  *     If the packets data is not 4 byte alligned it is padded with null octets
  *     to a four byte alligned quantity.( So you should assure that on MSB first
  *     platforms that have to read/write 16/32 bit PWords, last data byte(s) are
  *     properly alligned(swapped) with zero padding byte(s). On the x86 PC
  *     the trayling 32 bits of packet data( including padding bytes ) does not
  *     need to be re-ordered.
  *     NOTE that the header field "length" shows the packets data length
  *     not counting any padding octets. So the receiver must allocated
  *     buffer for packet data reception that is (length+3)/4, and
  *     discard ( length+3)%4 octets at the end of this buffer( swap octets if native byte order
  *     is not LSB before discarding trayling padding octets ) before
  *     indicating the packet to the upper network layer.
  *
  *     Also if MSB first platform has to receive/send using 16/32 bit ECP Hardware
  *     it should reorder all packet data ( if it needs to do so ) so it is sent
  *     through the 8bit physical interface in the packet's native order( which is
  *     again always true for x86 PCs )
  *
  */
#if defined(__LITTLE_ENDIAN)
#define EPLIP_HLEN 8
#define EPLIP_HHFLAGS_PADDING           0x3
#define EPLIP_HHFLAGS_ADLER32CRC        0x80
union   eplip_hh {
        struct {
                __u16 length;   /* packet length        */
                __u16 flags;    /* currently only LS 2 bits are used( number of padding bytes in the packet     */
                __u32 checksum; /* packet data checksum */
        } h;
        __u8  raw[EPLIP_HLEN];
};
#else
#  error "Please FIX the endianness issues"
#endif


enum eplip_connection_state {
	EPLIP_CN_NONE=0,
	EPLIP_CN_RECEIVE,
	EPLIP_CN_SEND,
};

enum eplip_packet_state {
	EPLIP_PK_DONE=0,
	EPLIP_PK_TRIGGER,
        EPLIP_PK_HEADER,
	EPLIP_PK_DATA,
        EPLIP_PK_CHECKSUM,
        EPLIP_PK_TERM
};

enum eplip_dma_state {
        EPLIP_DMA_NONE=0,
        EPLIP_DMA_RECEIVE,
        EPLIP_DMA_SEND,
        EPLIP_DMA_TERM,
        EPLIP_DMA_ERROR,
};

struct eplip_local {
	enum    eplip_packet_state state;
        union   eplip_hh hh ;
        struct  sk_buff *skb;
} ;

/* Information that need to be kept for each board. */
struct net_local {

	struct net_device_stats enet_stats;
	struct net_device *ndev;

        struct work_struct immediate;                     /** for immediate Bottom Half handler           */
        struct delayed_work deferred;                      /** for deferred  Bottom Half handler           */

        struct eplip_local snd_data;
	struct eplip_local rcv_data;

        unsigned short timeout_count;                   /** timeout counter                             */
        unsigned long  trigger_timeout;                 /** Transfer negotiation timeout interval       */
        unsigned long  fifo_timeout;                    /** fifo burst timeout interval                 */

        enum    eplip_connection_state  connection;     /** for the state machine state tracking        */

        enum    eplip_dma_state         dma_state;      /** for the DMA trans state machine             */

        char*   bounce_buff;/** for the bouncing buffer used for ISA safe DMAing from/to the ECP FIFO   */
        __u32   bounce_buff_phys;
        __u32   phys_addr;  /** physical(aka bus address ) of the buffer for DMAing from/to             */

        ecp_dev *dev ;                                  /** for the ECP HWD                             */

#ifdef  EPLIP_16bit_FIFO_SUPPORT
        __u8    (*send_hard_header)     ( struct eplip_local *snd , ecp_dev *dev , __u32 timeout );
        __u8    (*receive_hard_header)  ( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout );
        __u8    (*send_data)            ( struct eplip_local *snd , ecp_dev *dev );
        __u8    (*receive_data)         ( struct eplip_local *rcv , ecp_dev *dev );
#endif

        int     is_deferred;
#ifdef NOARP
        int (*orig_hard_header)(struct sk_buff *skb, struct net_device *dev,
	                        unsigned short type, void *daddr,
	                        void *saddr, unsigned len);

	int (*orig_hard_header_cache)(struct neighbour *neigh,
	                              struct hh_cache *hh);
#endif
	/* Tx control lock.  This protects the transmit buffer ring
	 * state along with the "tx full" state of the driver.  This
	 * means all netif_queue flow control actions are protected
	 * by this lock as well.
	 */
	spinlock_t lock;

        struct timer_list       timer;
        unsigned                mru ;
};


/* Index to functions, as function prototypes. */

/* Bottom halves */
static void eplip_kick_bh(struct work_struct *work);
static void eplip_bh(struct work_struct *work);

/* Functions for DEV methods */
static int eplip_tx_packet(struct sk_buff *skb, struct net_device *dev);
#ifdef NOARP
static int eplip_hard_header(struct sk_buff *skb, struct net_device *dev,
                            unsigned short type, void *daddr,
                            void *saddr, unsigned len);
static int eplip_hard_header_cache(struct neighbour *neigh,
                                  struct hh_cache *hh);
#endif
static int	eplip_open(struct net_device *dev);
static irqreturn_t eplip_interrupt(int, void *);
static int	eplip_close(struct net_device *dev);
static struct	net_device_stats *eplip_get_stats(struct net_device *dev);
static int      eplip_change_mtu(struct net_device *dev, int new_mtu);
//static void	set_multicast_list(struct net_device *dev);
#ifdef EPLIP_16bit_FIFO_SUPPORT
static int      eplip_ioctl(struct net_device *dev, struct ifreq *rq, int cmd);
#endif



/** Low Level Transmit routines */
__inline __u32
mk_chk( __u8 *buff , __u16 nbytes ) ;

static __inline __u8
eplip_send_hard_header( struct eplip_local *snd , ecp_dev *dev , __u32 timeout );

static __inline __u8
eplip_receive_hard_header( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout );

static __inline __u8
eplip_send_data(  struct eplip_local *snd ,ecp_dev *dev );

static __inline __u8
eplip_receive_data( struct eplip_local *rcv , ecp_dev *dev );

#ifdef EPLIP_16bit_FIFO_SUPPORT

static __inline __u8
eplip_send_hard_header2( struct eplip_local *snd , ecp_dev *dev , __u32 timeout );

static __inline __u8
eplip_receive_hard_header2( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout );

static __inline __u8
eplip_send_data2(  struct eplip_local *snd ,ecp_dev *dev );

static __inline __u8
eplip_receive_data2( struct eplip_local *rcv , ecp_dev *dev );

static __inline __u8
eplip_receive_data4( struct eplip_local *rcv , ecp_dev *dev ) ;

static __inline __u8
eplip_send_data4(  struct eplip_local *snd ,ecp_dev *dev );

static __inline __u8
eplip_send_hard_header4( struct eplip_local *snd , ecp_dev *dev , __u32 timeout );

static __inline __u8
eplip_receive_hard_header4( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout );

#endif

static void dma_timeout_routine( struct timer_list *t );

static const struct net_device_ops eplip_netdev_ops = {
	.ndo_open	= eplip_open,
	.ndo_stop	= eplip_close,
	.ndo_start_xmit	= eplip_tx_packet,
	.ndo_get_stats	= eplip_get_stats,
	.ndo_change_mtu	= eplip_change_mtu,
	/*.ndo_do_ioctl	= eplip_ioctl;*/
};


int
eplip_init_dev(struct net_device *dev, ecp_dev *ecpdev,unsigned long hwaddr)
{
	struct  net_local *nl;
        int     i  ;

	SET_MODULE_OWNER(dev);

	dev->irq = ecpdev->irq;
	dev->base_addr = ecpdev->iobase;
        if((ecpdev->dma > 0 ) && ( ecpdev->dma < 8 ) && ( ecpdev->mode == EPLIP_MODE_ECP_DMA ) )
                dev->dma = ecpdev->dma;
        else
                dev->dma=0;

        /* Fill in the generic fields of the device structure. */
	ether_setup(dev);

	/* Then, override parts of it */
	dev->netdev_ops =  &eplip_netdev_ops;
	/*
	dev->hard_start_xmit	 = eplip_tx_packet;
	dev->open		 = eplip_open;
	dev->stop		 = eplip_close;
	dev->get_stats 		 = eplip_get_stats;
        dev->change_mtu          = eplip_change_mtu;
       	//dev->do_ioctl		 = eplip_ioctl;
       	*/

	dev->tx_queue_len 	 = 10;
	//dev->flags	        |= IFF_POINTOPOINT|IFF_NOARP;
	dev->min_mtu                 = 68;
	dev->max_mtu                 = EPLIP_MRU;
        dev->mtu                 = EPLIP_MTU;

        /** set the upper byte of MAC Addr */
	memset(dev->dev_addr, 0xfc, ETH_ALEN);
      //  if(hwaddr){
                for( i=0; i<4; i++ ){
                        dev->dev_addr[5-i]=hwaddr &0xff;
                        hwaddr>>=8;
                }
               
      //  }
	/* Set the private structure */

/*
	dev->priv = kmalloc(sizeof (struct net_local), GFP_KERNEL);
	if (!dev->priv) {
		printk(KERN_ERR "%s: out of memory\n", dev->name);
		return -ENOMEM;
	}
	memset(dev->priv, 0, sizeof(struct net_local));
*/
	nl = (struct net_local *)netdev_priv(dev);

        nl->mru = EPLIP_MRU ;
	nl->ndev = dev;

        i=3;
        while( (dev->dma > 0 )) {

                nl->bounce_buff = kmalloc(PKT_RCVBUF_SZ,GFP_DMA | GFP_KERNEL);

                if((!nl->bounce_buff) ||   ( (u32)virt_to_bus(nl->bounce_buff) + PKT_RCVBUF_SZ > MAX_DMA_PHYS_ADDRESS)  ){

                        if(nl->bounce_buff) {
                                kfree(nl->bounce_buff);
                                nl->bounce_buff=NULL;
#if NET_DEBUG > 3
                                printk(KERN_DEBUG "%s:Bounce Buffer Allocated __above__ 16MB\n",dev->name);
#endif
                        }
                        if( i-- )
                                continue;

                        printk(KERN_ERR "%s:Out of ISA DMA safe memory.Falling to FIFO PIO mode\n",dev->name);
                        dev->dma = 0;
            }
                else {
                        nl->bounce_buff_phys = (u32)virt_to_bus(nl->bounce_buff);
                }
                break ;
        }

#ifdef NOARP
	nl->orig_hard_header    = dev->hard_header;
	dev->hard_header        = eplip_hard_header;

	nl->orig_hard_header_cache = dev->hard_header_cache;
	dev->hard_header_cache     = eplip_hard_header_cache;
#endif
	nl->dev = ecpdev;


#ifdef EPLIP_16bit_FIFO_SUPPORT
        switch( ecpdev->pword ){
        case 8: nl->send_hard_header   = eplip_send_hard_header;
                nl->receive_hard_header= eplip_receive_hard_header;
                nl->send_data          = eplip_send_data;
                nl->receive_data       = eplip_receive_data;
                break;
        case 16:nl->send_hard_header   = eplip_send_hard_header2;
                nl->receive_hard_header= eplip_receive_hard_header2;
                nl->send_data          = eplip_send_data2;
                nl->receive_data       = eplip_receive_data2;
                break;
        case 32:nl->send_hard_header   = eplip_send_hard_header4;
                nl->receive_hard_header= eplip_receive_hard_header4;
                nl->send_data          = eplip_send_data4;
                nl->receive_data       = eplip_receive_data4;
                break;
        }
#endif

	/* Initialize constants */
	nl->trigger_timeout	= EPLIP_TRIGGER_WAIT;
	nl->fifo_timeout	= EPLIP_BYTE_WAIT* nl->dev->fifo_depth;

	/* Initialize task queue structures */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0)
        INIT_WORK(&nl->immediate, eplip_bh);
        INIT_DELAYED_WORK(&nl->deferred, eplip_kick_bh);

#else
        nl->immediate.next = NULL;
        nl->deferred.next  = NULL;
#endif

	spin_lock_init(&nl->lock);

	return 0;
}

/* Forward declarations of internal routines */

static int eplip_none(struct net_device *, struct net_local *,
		     struct eplip_local *, struct eplip_local *);
static int eplip_receive_packet(struct net_device *, struct net_local *,
			       struct eplip_local *, struct eplip_local *);
static int eplip_send_packet(struct net_device *, struct net_local *,
			    struct eplip_local *, struct eplip_local *);
static int eplip_bh_timeout_error(struct net_device *dev, struct net_local *nl,
				 struct eplip_local *snd,
				 struct eplip_local *rcv,
				 int error);
typedef int (*eplip_func)(struct net_device *dev, struct net_local *nl,
			 struct eplip_local *snd, struct eplip_local *rcv);

static eplip_func connection_state_table[] =
{
	eplip_none,
	eplip_receive_packet,
	eplip_send_packet
};

/* Bottom half handler for the delayed request.
   This routine is kicked by do_timer().
   Request `eplip_bh' to be invoked. */

static void
eplip_kick_bh(struct work_struct *work)

{
	struct net_local *nl = container_of(work, struct net_local, deferred.work);
	unsigned long flags;

        spin_lock_irqsave( &nl->lock, flags );
	if ( nl->is_deferred ) {

                //if( nl->connection !=EPLIP_CN_RECEIVE )

                {
#if NET_DEBUG >3
                        printk(KERN_DEBUG"%s: Scheduling this task to the IMMEDIATE_BH queue\n",dev->name);
#endif
                        //nl->is_deferred = 0 ;
		        schedule_work(&nl->immediate);

                }

                //else {
                        //queue_task(&nl->deferred,&tq_timer);
                //}

	}
        spin_unlock_irqrestore( &nl->lock, flags );
}

/* Bottom half handler of EPLIP. */
static void
eplip_bh(struct work_struct *work)
{
	struct net_local *nl = container_of(work, struct net_local, immediate);

	struct eplip_local *snd = &nl->snd_data;
	struct eplip_local *rcv = &nl->rcv_data;
	eplip_func f;
	int r;

                nl->is_deferred = 0;
                f = connection_state_table[nl->connection];
                if ((r = (*f)(nl->ndev, nl, snd, rcv)) != OK
                && (r = eplip_bh_timeout_error(nl->ndev, nl, snd, rcv, r)) != OK) {
	                nl->is_deferred = 1;
	                schedule_delayed_work(&nl->deferred, 1);
                }

}
static int
eplip_none(struct net_device *dev, struct net_local *nl,
	  struct eplip_local *snd, struct eplip_local *rcv)
{
	return OK;
}

static void
eplip_schedule_dma_timeout( struct net_local* nl, unsigned long timeout)
{
	timer_setup(&nl->timer, dma_timeout_routine, 0);
	mod_timer(&nl->timer, jiffies + timeout);
}

/*
 *	Determine the packet's protocol ID. The rule here is that we
 *	assume 802.3 if the type field is short enough to be a length.
 *	This is normal practice and works for any 'now in use' protocol.
 *
 *	EPLIP is ethernet ish but the daddr might not be valid if unicast.
 *	EPLIP fortunately has no bus architecture (its Point-to-point).
 *
 *	We can't fix the daddr thing as that quirk (more bug) is embedded
 *	in far too many old systems not all even running Linux.
 */

static unsigned short eplip_type_trans(struct sk_buff *skb, struct net_device *dev)
{
	struct ethhdr *eth;
	unsigned char *rawp;

	skb_reset_mac_header(skb);
	skb_pull(skb,dev->hard_header_len);
	eth = eth_hdr(skb);

	if(*eth->h_dest&1)
	{
		if(memcmp(eth->h_dest,dev->broadcast, ETH_ALEN)==0)
			skb->pkt_type=PACKET_BROADCAST;
		else
			skb->pkt_type=PACKET_MULTICAST;
	}

	if (ntohs(eth->h_proto) >= 1536)
        {
                return eth->h_proto;
        }

	rawp = skb->data;

	/*
	 *	This is a magic hack to spot IPX packets. Older Novell breaks
	 *	the protocol design and runs IPX over 802.3 without an 802.2 LLC
	 *	layer. We look for FFFF which isn't a used 802.2 SSAP/DSAP. This
	 *	won't work for fault tolerant netware but does for the rest.
	 */
	if (*(unsigned short *)rawp == 0xFFFF)
		return htons(ETH_P_802_3);

	/*
	 *	Real 802.2 LLC
	 */
	return htons(ETH_P_802_2);
}

#ifdef NOARP

static void
eplip_rewrite_address(struct net_device *dev, struct ethhdr *eth)
{
        struct in_device *in_dev;

	if ((in_dev=dev->ip_ptr) != NULL) {
		/* Any address will do - we take the first */
		struct in_ifaddr *ifa=in_dev->ifa_list;
		if (ifa != NULL) {
			memcpy(eth->h_source, dev->dev_addr, 6);
			memset(eth->h_dest, 0xfc, 2);
			memcpy(eth->h_dest+2, &ifa->ifa_address, 4);
		}
	}
}

static int
eplip_hard_header(struct sk_buff *skb, struct net_device *dev,
                 unsigned short type, void *daddr,
	         void *saddr, unsigned len)
{
	struct net_local *nl = netdev_priv(dev);
	int ret;

	if ((ret = nl->orig_hard_header(skb, dev, type, daddr, saddr, len)) >= 0)
		eplip_rewrite_address (dev, (struct ethhdr *)skb->data);

	return ret;
}

int
eplip_hard_header_cache(struct neighbour *neigh,
                           struct hh_cache *hh)
{
	struct net_local *nl = netdev_priv(neigh->dev);
	int ret;

	if ((ret = nl->orig_hard_header_cache(neigh, hh)) == 0)
	{
		struct ethhdr *eth = (struct ethhdr*)(((u8*)hh->hh_data) + 2);
		eplip_rewrite_address (neigh->dev, eth);
	}

	return ret;
}

#endif

static int
eplip_bh_timeout_error(struct net_device *dev, struct net_local *nl,
		      struct eplip_local *snd, struct eplip_local *rcv,
		      int error)
{
	unsigned long flags;

	spin_lock_irqsave(&nl->lock, flags);
	if (nl->connection == EPLIP_CN_SEND) {

		if ( error == HS_TIMEOUT ) { /* Timeout */
			nl->timeout_count++;
			if ( nl->timeout_count <= 10) {
				spin_unlock_irqrestore(&nl->lock, flags);
				/* Try again later */
				return TIMEOUT;
			}

		}

		nl->enet_stats.tx_errors++;
                if( error == HS_TIMEOUT )
                {
                        nl->enet_stats.tx_carrier_errors++;
                }
                else {
                        nl->enet_stats.tx_aborted_errors++;
                }

                if (snd->skb) {
		        dev_kfree_skb(snd->skb);
		        snd->skb = NULL;
	        }
                snd->state = EPLIP_PK_DONE ;
                nl->connection = EPLIP_CN_NONE ;
                nl->dma_state =  EPLIP_DMA_NONE ;

                set_mode_idle( nl->dev );

                netif_wake_queue( dev );

                spin_unlock_irqrestore(&nl->lock, flags);
                return OK ;

	}
        else
                if ( nl->connection == EPLIP_CN_RECEIVE) {
		        if (rcv->state == EPLIP_PK_TRIGGER) {
			/* Transmission Start was interrupted. */

                        //printk(KERN_DEBUG"%s: inside eplip_bh_timeout_error()\n",dev->name);
                        //nl->is_deferred = 0 ;
                        //queue_task( &nl->immediate , &tq_immediate );
                        //mark_bh( IMMEDIATE_BH );
												schedule_work(&nl->immediate);
			spin_unlock_irqrestore(&nl->lock, flags);
			return OK;
		        }

	        }
        else
                printk( KERN_ERR "%s: inside eplip_bh_timeout() in neither SND/SCV state",dev->name);

        //printk(KERN_DEBUG"%s: CN_STATE:%x RCV_STATE:%x SND_STATE:%x\n",dev->name,nl->connection,rcv->state,snd->state);

        rcv->state = EPLIP_PK_DONE ;
        nl->dma_state = EPLIP_DMA_NONE;

	if (rcv->skb) {
                //printk(KERN_DEBUG"%s: receive skb left allocated\n" , dev->name);
		kfree_skb(rcv->skb);
		rcv->skb = NULL;
	}

        if( snd->state != EPLIP_PK_DONE ) {
                if( snd->state == EPLIP_PK_TRIGGER ) {
                        nl->connection = EPLIP_CN_SEND ;
                        set_mode_idle( nl->dev );
                        spin_unlock_irqrestore( &nl->lock, flags );
                        return TIMEOUT ;

                } else {

	                if (snd->skb) {
                                printk(KERN_ERR "%s: send skb left allocated\n" , dev->name);
		                dev_kfree_skb(snd->skb);
		                snd->skb = NULL;
	                }
                        snd->state = EPLIP_PK_DONE ;
                        if( netif_queue_stopped( dev ) ) {
                                netif_wake_queue( dev );
                        }
                        else
                                printk(KERN_ERR "%s: I've hit a BUG in the STATE machine( Queue running in SND state)",dev->name);
                        nl->enet_stats.tx_errors++;
                        nl->enet_stats.tx_aborted_errors++;
                 }
        }

        nl->connection = EPLIP_CN_NONE;
        set_mode_idle( nl->dev );
	spin_unlock_irqrestore(&nl->lock, flags);

        return OK;
}

static int
eplip_tx_packet(struct sk_buff *skb, struct net_device *dev)
{
	struct net_local *nl = netdev_priv(dev);
	struct eplip_local *snd = &nl->snd_data;
	unsigned long flags;

	if (netif_queue_stopped(dev))
		return NET_XMIT_CN;

        netif_stop_queue(dev);

	if (skb->len > dev->mtu + dev->hard_header_len) {
                printk(KERN_WARNING "%s: packet too big, %d.\n", dev->name, (int)skb->len);
                netif_start_queue (dev);
		return NET_XMIT_DROP;
	}
#if NET_DEBUG > 2
        printk(KERN_DEBUG "%s: send request\n", dev->name);
#endif

        //snd->hh.h.flags  = 0;//nl->uses_adler32crc?EPLIP_HHFLAGS_ADLER32CRC:0;
#ifdef OLD_PAD_METHOD
        if( skb->len % 4 ) {
                unsigned int pad_len;
#if NET_DEBUG > 3
                printk(KERN_INFO"%s: Send SKB data is not 4 bytes alligned\n",dev->name);
#endif
                pad_len = 4 - ( skb->len % 4 );
                if(skb_tailroom( skb ) < pad_len ) {
                        struct sk_buff *tmp_skb ;
#if NET_DEBUG > 3
                        printk(KERN_INFO"%s: Not Enough free tail room:%u needed:%u\n",dev->name,skb_tailroom(skb),pad_len);
#endif
                        tmp_skb = __dev_alloc_skb( skb->len+pad_len, GFP_ATOMIC|GFP_DMA );
                        if( tmp_skb == NULL )
                                tmp_skb = dev_alloc_skb( skb->len+pad_len );

		        if (tmp_skb == NULL) {
			        printk(KERN_ERR "%s: Memory squeeze.\n", dev->name);
                                netif_start_queue(dev);
			        return NET_XMIT_DROP;
		        }

                        tmp_skb->dev = dev ;
                        tmp_skb->protocol = skb->protocol ;
                        memcpy( skb_put( tmp_skb,skb->len+pad_len ) , skb->data , skb->len );
                        memset( tmp_skb->data+skb->len,0,pad_len);
                        dev_kfree_skb(skb);
                        skb=tmp_skb;
                }
                else {
                        memset(skb_put( skb , pad_len ), 0 , pad_len );
                }
                snd->hh.h.flags |= pad_len ;
        }
#endif
        {       unsigned int pad_len = skb->len % 4 ;
                if( pad_len ) {
                        snd->hh.h.flags = 4 - ( pad_len ) ;
                }
                else{
                        snd->hh.h.flags = 0 ;
                }
                snd->hh.h.length   = skb->len+snd->hh.h.flags ;
        }
        dev->_tx->trans_start   = jiffies;
        snd->skb = skb;

        snd->hh.h.checksum = mk_chk( skb->data , skb->len );

	snd->state = EPLIP_PK_TRIGGER;

        spin_lock_irqsave(&nl->lock, flags);
        nl->timeout_count = 0;
	if (nl->connection == EPLIP_CN_NONE) {

		nl->connection = EPLIP_CN_SEND;
                schedule_work(&nl->immediate);
	}

        else
        {
#if NET_DEBUG > 3
                printk(KERN_DEBUG"%s:RCV DMA in progress.Deferring this SND request immediately\n",dev->name);
#endif
                nl->is_deferred = 1;
                schedule_delayed_work(&nl->deferred, 1);
        }
	spin_unlock_irqrestore(&nl->lock, flags);

	return NET_XMIT_SUCCESS;
}

/** @func   eplip_dma_close( struct net_device* dev )
  * @brief  Closes a DMA transfer session.
  * @return Number of bytes left to be transfered
  * @retval 0 - All bytes were transfered
  */
static int
eplip_dma_close( struct net_device* dev )
{
        struct net_local *nl = netdev_priv(dev);
        unsigned long flags;
        int bytes_left;
#ifdef PARANOID
        read_econtrol   ( nl->dev );
#endif
        set_srvc_intr   ( nl->dev );
        disable_ecp_dma ( nl->dev );

        flags = claim_dma_lock();
        disable_dma( dev->dma );
        bytes_left=get_dma_residue(dev->dma);
        release_dma_lock(flags);

        return bytes_left;
};

static void
dma_timeout_routine( struct timer_list *t )
{
	struct net_local* nl = from_timer(nl, t, timer);
	unsigned long flags;

	spin_lock_irqsave(&nl->lock, flags);
	//printk(KERN_ERR"inside dma_timeout_routine() \n");
	if (nl->dma_state != EPLIP_DMA_TERM ) {
		nl->dma_state = EPLIP_DMA_ERROR ;
		schedule_work(&nl->immediate);
	}
	spin_unlock_irqrestore(&nl->lock, flags);
};

static int
eplip_dma_send_data( struct net_device* dev )
{
        struct net_local        *nl  = netdev_priv(dev);
        struct eplip_local      *snd = &nl->snd_data;
        unsigned long           flags;
        u32                     phys_addr;

/** Check the send skb is ISA DMA safe
  * if __NOT__ use the bounce buffer
  */
        phys_addr=(u32)virt_to_bus(snd->skb->data );
        if(   ( phys_addr + snd->hh.h.length ) > MAX_DMA_PHYS_ADDRESS  ){

#if NET_DEBUG > 3
                printk(KERN_DEBUG "%s: Using bouncing buffer to send data\n",dev->name);
#endif
                memcpy(nl->bounce_buff, snd->skb->data , snd->hh.h.length );
                phys_addr = nl->bounce_buff_phys;

        }

        flags = claim_dma_lock();
/** Program the DMAC with op-mode, address and count
  */
        clear_dma_ff( dev->dma );
        set_dma_mode( dev->dma , DMA_MODE_WRITE );
        set_dma_addr( dev->dma , phys_addr );
        set_dma_count(dev->dma , snd->hh.h.length);

/** Enable DMA operation on the ECP HWD
  * Enable SrvcIntr's
  */
#ifdef PARANOID
        read_econtrol ( nl->dev );
#endif
        enable_ecp_dma( nl->dev );
        clear_srvc_intr(nl->dev );

/** Now we are ready to enable the DMA on the DMAC
  */
        nl->dma_state = EPLIP_DMA_SEND ;

        enable_dma(dev->dma);
        release_dma_lock( flags );

        return OK ;
}


static int
eplip_dma_receive_data( struct net_device* dev)
{
        struct net_local        *nl  = netdev_priv(dev);
        struct eplip_local      *rcv = &nl->rcv_data;
        unsigned long           flags;


/** Check the send skb is ISA DMA safe
  * if __NOT__ use the bounce buffer
  */
        nl->phys_addr=(u32)virt_to_bus(rcv->skb->data );
        if(   ( nl->phys_addr + rcv->hh.h.length ) > MAX_DMA_PHYS_ADDRESS  ){

#if NET_DEBUG > 3
                printk(KERN_DEBUG "%s: Using bouncing buffer to receive data\n",dev->name);
#endif
                nl->phys_addr = nl->bounce_buff_phys;//(u32)virt_to_bus(nl->bounce_buff);

        }

        flags = claim_dma_lock();
/** Program the DMAC with op-mode, address and count
  */
        clear_dma_ff( dev->dma );
        set_dma_mode( dev->dma , DMA_MODE_READ );
        set_dma_addr( dev->dma , nl->phys_addr );
        set_dma_count(dev->dma , rcv->hh.h.length );

/** Enable DMA operation on the ECP HWD
  * Enable SrvcIntr's
  */
#ifdef PARANOID
        read_econtrol ( nl->dev );
#endif
        enable_ecp_dma( nl->dev );
        clear_srvc_intr(nl->dev );

/** Now we are ready to enable the DMA on the DMAC
  */
        nl->dma_state = EPLIP_DMA_RECEIVE ;

        enable_dma(dev->dma);
        release_dma_lock( flags );

        return OK ;
}

/* PLIP_SEND_PACKET --- send a packet */
static int
eplip_send_packet(struct net_device *dev,  struct net_local *nl,
		  struct eplip_local *snd, struct eplip_local *rcv)
{
	ecp_dev*        ecpdev          = nl->dev ;
	unsigned long flags;

#ifdef PARANOID

        if (snd->skb == NULL ||  snd->skb->data == NULL) {
		printk(KERN_DEBUG "%s: send skb lost\n", dev->name);
		snd->state = EPLIP_PK_DONE;
		snd->skb = NULL;
		return ERROR;
	}
#endif
        switch (snd->state) {

	case EPLIP_PK_TRIGGER:
		spin_lock_irqsave( &nl->lock, flags );
                if ( nl->connection == EPLIP_CN_RECEIVE) {
                        /* Interrupted. */
                        nl->enet_stats.collisions++;
                        schedule_work(&nl->immediate);
                        
		        spin_unlock_irqrestore(&nl->lock, flags);
			return OK;
		}

                switch( aquire_ecp_bus( ecpdev ) ) {
                        case E_MODE_OK:
                                        break;
                        case E_MODE_RQ_PADDING  :
#if NET_DEBUG >1
                                printk(KERN_ERR "%s: RQ_PADDING\n", dev->name);
#endif
                                spin_unlock_irqrestore( &nl->lock, flags );
                                return HS_TIMEOUT ;

                        case E_MODE_RQ_COLLISION:
#if NET_DEBUG >1
                                printk(KERN_ERR "%s: RQ_COLLISION\n", dev->name);
#endif
                                        spin_unlock_irqrestore( &nl->lock, flags );
                                        return HS_TIMEOUT;
                        case E_MODE_REMOTE_DOWN :
#if NET_DEBUG >1
                                printk(KERN_ERR "%s: REMOTE_DOWN(SR=%x)\n", dev->name,READ_STATUS(nl->dev->iobase));
#endif
                                        spin_unlock_irqrestore( &nl->lock, flags );
                                        return HS_TIMEOUT;

                }

                switch( set_mode_fdata( ecpdev , nl->trigger_timeout ) ){
                        case E_MODE_OK :
                                        break;
                        case E_MODE_RQ_COLLISION:
                                        nl->enet_stats.collisions++;

                        case E_MODE_RQ_TIMEOUT:
#if NET_DEBUG >1
                                printk(KERN_ERR "%s: Timeout Negotiating RIDLE2F mode\n", dev->name);
#endif
                                        enable_interrupt( ecpdev );
                                        frob_control( ecpdev , CR_INIT , CR_INIT );

                                        spin_unlock_irqrestore( &nl->lock, flags );
                                        return HS_TIMEOUT;
                }
                spin_unlock_irqrestore( &nl->lock, flags );

#if NET_DEBUG > 2
	        printk(KERN_DEBUG "%s: send start\n", dev->name);
#endif
		snd->state = EPLIP_PK_HEADER;

        case EPLIP_PK_HEADER:
        case EPLIP_PK_CHECKSUM:

/** Send Header - LENGTH , CHECHSUM
  */
#ifdef EPLIP_16bit_FIFO_SUPPORT
                if( nl->send_hard_header( snd , ecpdev , nl->fifo_timeout )!=OK ) {
#else
                if(eplip_send_hard_header( snd , ecpdev , nl->fifo_timeout )!=OK ) {
#endif

#if NET_DEBUG > 1
                        printk(KERN_ERR "%s: Timeout sending HEADER\n", dev->name);
#endif
			return TIMEOUT;
                }
		snd->state      = EPLIP_PK_DATA;

	case EPLIP_PK_DATA:
 /** Send Data Block
   */
                if( dev->dma ){

                        if(nl->dma_state == EPLIP_DMA_NONE ) {
#if NET_DEBUG >3
                                printk(KERN_DEBUG "%s:Using DMA to Send DATA\n",dev->name);
#endif
                                eplip_dma_send_data( dev );
                                spin_lock_irqsave(&nl->lock, flags);
                                if(nl->dma_state == EPLIP_DMA_SEND ) {
                                        eplip_schedule_dma_timeout( nl , (HZ) );
                                        spin_unlock_irqrestore(&nl->lock, flags);
                                        return OK ;
                                }
                                spin_unlock_irqrestore(&nl->lock, flags);
                        }

                        spin_lock_irqsave(&nl->lock, flags);
#if NET_DEBUG > 3
                        if(!del_timer(&nl->timer)) {
                                printk(KERN_DEBUG"%s: DMA timeout timer has expired\n",dev->name);
                        };
#else
                        del_timer(&nl->timer);
#endif
                        if( nl->dma_state!=EPLIP_DMA_TERM ){
                                unsigned int residue ;
                                residue         = eplip_dma_close( dev );
                                nl->dma_state   = EPLIP_DMA_ERROR ;
#if NET_DEBUG > 1
                                printk(KERN_ERR "%s DMA Timeout sending DATA.Bytes Left:%u of:%u\n",dev->name,residue,snd->hh.h.length);
#endif
                                spin_unlock_irqrestore(&nl->lock, flags);
                                return TIMEOUT ;
                        }

                        {
                                int residue = eplip_dma_close( dev );
                                if(residue){
#if NET_DEBUG >2
                                        printk(KERN_DEBUG "%s: DMA Send __UNFINISHED__.%i bytes left.\n",dev->name,residue);
#endif
                                }

                        }
                        nl->dma_state = EPLIP_DMA_NONE ;
                        spin_unlock_irqrestore(&nl->lock, flags);

                }
                else
                {
#ifdef EPLIP_16bit_FIFO_SUPPORT
                        if( (  nl->send_data( snd , ecpdev ) ) !=OK ){
#else
                        if( (  eplip_send_data( snd , ecpdev ) ) !=OK ){
#endif
#if NET_DEBUG > 1
                                printk(KERN_ERR "%s: Timeout sending DATA\n", dev->name);
#endif

                                return TIMEOUT ;
                        }

                }

                snd->state = EPLIP_PK_TERM;

	case EPLIP_PK_TERM:

/** Wait untill all bytes in the FIFO have been transmited or timeout
  */
                switch( fifo_send_complete( ecpdev , nl->fifo_timeout ) ){
                case E_MODE_OK: break;
                case 1:         printk(KERN_INFO"%s: Timeout waiting HWD ACKed last byte\n",dev->name);break;
                case 2:
#if NET_DEBUG > 1
                        printk(KERN_ERR "%s: Timeout waiting transfer TERMINATION\n", dev->name);
#endif
                        return TIMEOUT ;
                }


/** Update NET_DEVICE stats
  */
		nl->enet_stats.tx_bytes += snd->skb->len;
		nl->enet_stats.tx_packets++;

		snd->state = EPLIP_PK_DONE;

	case EPLIP_PK_DONE:
/** Terminate Transfer - Swith HWD back to REVERSE_IDLE state
  */            spin_lock_irqsave(&nl->lock, flags);

                dev_kfree_skb(snd->skb);
		snd->skb = NULL;

                nl->connection = EPLIP_CN_NONE;
                set_mode_idle( ecpdev );
                netif_wake_queue(dev);

#if NET_DEBUG > 2
                printk(KERN_DEBUG "%s: send end\n", dev->name);
#endif
                spin_unlock_irqrestore(&nl->lock, flags);

		return OK;
	}

	return OK;
}

/* PLIP_RECEIVE_PACKET --- receive a packet */
static int
eplip_receive_packet(struct net_device *dev, struct net_local *nl,
		    struct eplip_local *snd, struct eplip_local *rcv)
{
	ecp_dev         *ecpdev      = nl->dev;
	unsigned long flags;

	switch (rcv->state) {
	case EPLIP_PK_TRIGGER:
		/* swithch to RIDLER_mode() */
                spin_lock_irqsave(&nl->lock, flags);
                switch( set_mode_rdata( ecpdev , nl->trigger_timeout) ) {
                        case E_MODE_OK: break;
                        case E_MODE_RQ_TIMEOUT:
                                        clear_Xflag( ecpdev );
                        case E_MODE_RQ_NACK:
#if NET_DEBUG > 3
                                        printk(KERN_DEBUG"%s: RECEIVE RQ not ACKed(SR:%x)\n",dev->name,READ_STATUS(nl->dev->iobase));
#endif
                                        rcv->state = EPLIP_PK_DONE ;
                                        if( snd->state != EPLIP_PK_DONE ) {

                                                nl->connection = EPLIP_CN_SEND;
                                                schedule_work(&nl->immediate);
                                        }
                                        else {

                                                nl->connection = EPLIP_CN_NONE ;
                                        }
                                        spin_unlock_irqrestore(&nl->lock, flags);
                                        return OK;
                }

                spin_unlock_irqrestore(&nl->lock, flags);
#if NET_DEBUG > 2
                printk(KERN_DEBUG "%s: receive start\n", dev->name);
#endif
		rcv->state = EPLIP_PK_HEADER;


	case EPLIP_PK_HEADER:

#ifdef  EPLIP_16bit_FIFO_SUPPORT
                if( nl->receive_hard_header( rcv , ecpdev , nl->fifo_timeout )  !=OK ){
#else
                if( eplip_receive_hard_header( rcv , ecpdev , nl->fifo_timeout )  !=OK ){
#endif
#if NET_DEBUG >1
                printk(KERN_ERR "%s:Timeout Header RCV after %lu useconds\n", dev->name,nl->fifo_timeout);
#endif
			return TIMEOUT;
                }
		if (rcv->hh.h.length > nl->mru + dev->hard_header_len
		    || rcv->hh.h.length < 8) {
#if NET_DEBUG > 1
			printk(KERN_WARNING "%s: bogus packet size %d.\n", dev->name, rcv->hh.h.length);
#endif
                        return ERROR;
		}

		/* Malloc up new buffer. */
                rcv->skb = __dev_alloc_skb(rcv->hh.h.length + 2,GFP_DMA|GFP_ATOMIC);
                if( rcv->skb == NULL )
                        rcv->skb = dev_alloc_skb(rcv->hh.h.length + 2 );

		if (rcv->skb == NULL) {
			printk(KERN_ERR "%s: Memory squeeze.\n", dev->name);
			return ERROR;
		}

		skb_reserve(rcv->skb, 2);	/* Align IP on 16 byte boundaries */
		skb_put(rcv->skb,rcv->hh.h.length);
		rcv->skb->dev = dev;
		rcv->state = EPLIP_PK_DATA;

	case EPLIP_PK_DATA:
                if( dev->dma ) {

                        if( nl->dma_state == EPLIP_DMA_NONE ) {
#if NET_DEBUG >3
                                printk(KERN_DEBUG "%s:Using DMA to Receive DATA\n",dev->name);
#endif

                                eplip_dma_receive_data( dev );
                                spin_lock_irqsave(&nl->lock, flags);

                                if(nl->dma_state == EPLIP_DMA_RECEIVE ) {
                                        eplip_schedule_dma_timeout( nl , (HZ) );
                                        spin_unlock_irqrestore(&nl->lock, flags);
                                        return OK ;
                                }

                                spin_unlock_irqrestore(&nl->lock, flags);
                        }

                        if( nl->dma_state == EPLIP_DMA_RECEIVE )
                                return OK ;

                        spin_lock_irqsave(&nl->lock, flags);

#if NET_DEBUG > 3
                        if(!del_timer(&nl->timer)) {
                                printk(KERN_DEBUG"%s: DMA timeout timer has expired\n",dev->name);
                        };
#else
                        del_timer(&nl->timer);
#endif
                        if( nl->dma_state!=EPLIP_DMA_TERM ){
                                unsigned int residue ;
                                residue = eplip_dma_close( dev );

#if NET_DEBUG > 1
                                printk(KERN_ERR  "%s: DMA Timeout Receiving DATA.Bytes Left:%u of:%u\n",dev->name,residue,rcv->hh.h.length);
                                printk(KERN_DEBUG"%s: dma_state=%u snd->state=%u rcv->state=%u connection=%u\n",dev->name,nl->dma_state,snd->state,rcv->state,nl->connection);
#endif
                                nl->dma_state =EPLIP_DMA_ERROR ;
                                spin_unlock_irqrestore(&nl->lock, flags);
                                return TIMEOUT ;
                        }

                        {
                                int residue = eplip_dma_close( dev );
                                if( residue ){
#if NET_DEBUG >1
                                        printk(KERN_DEBUG "%s: DMA Receive __UNFINISHED__.%i bytes left.\n",dev->name,residue);
#endif
                                }
                        }

                        nl->dma_state = EPLIP_DMA_NONE ;
                        spin_unlock_irqrestore(&nl->lock, flags);

                        if(  nl->bounce_buff_phys == nl->phys_addr ) {
#if NET_DEBUG >3
                                printk(KERN_DEBUG "%s Copying UP from the bounce buffer to the receive SKB\n",dev->name);

#endif
                                memcpy(rcv->skb->data,nl->bounce_buff,rcv->hh.h.length);
                        }
                }
                else
                {
#ifdef  EPLIP_16bit_FIFO_SUPPORT
                        if( nl->receive_data( rcv , ecpdev )  != OK ){
#else
                        if( eplip_receive_data( rcv , ecpdev )  != OK ){
#endif
#if NET_DEBUG > 1
                                printk(KERN_ERR "%s:Timeout DATA RCV\n", dev->name);
#endif
                                return TIMEOUT;
                        }
                }

		rcv->state = EPLIP_PK_CHECKSUM;


	case EPLIP_PK_CHECKSUM:{
                __u32 checksum ;

                 if( rcv->hh.h.flags & EPLIP_HHFLAGS_PADDING ) {

#if NET_DEBUG > 3
                        printk(KERN_INFO"%s: RCV Packet was padded with %u bytes\n",dev->name, (rcv->hh.h.flags & EPLIP_HHFLAGS_PADDING));
#endif
                        skb_trim( rcv->skb , rcv->hh.h.length - ( rcv->hh.h.flags & EPLIP_HHFLAGS_PADDING ) );
                };

                //if( rcv->hh.h.flags & EPLIP_HHFLAGS_ADLER32CRC ){
               //         checksum = mk_crc( rcv->skb->data,rcv->skb->len);
               // }
               // else {
                        checksum = mk_chk( rcv->skb->data, rcv->skb->len );
               // }

		if ( checksum != rcv->hh.h.checksum) {
			nl->enet_stats.rx_crc_errors++;
                        nl->enet_stats.rx_errors++;
#if NET_DEBUG > 1
                               // if(rcv->hh.h.flags & EPLIP_HHFLAGS_ADLER32CRC)
                               //         printk(KERN_DEBUG "%s: crc error->HCRC: %#8x CCRC: %#8x:\n", dev->name,rcv->hh.h.checksum,checksum);
                               // else
				        printk(KERN_DEBUG "%s: checksum error->HCRC: %#8x CCRC: %#8x:\n", dev->name,rcv->hh.h.checksum,checksum);
#endif
			return ERROR;
		}
		rcv->state = EPLIP_PK_DONE;
        }
        case EPLIP_PK_TERM:
	case EPLIP_PK_DONE:
		/* Inform the upper layer for the arrival of a packet.  */
                /* Remove the packet data padding bytes                 */

	        rcv->skb->protocol=eplip_type_trans(rcv->skb, dev);
		netif_rx(rcv->skb);
		nl->enet_stats.rx_bytes += rcv->hh.h.length;
		nl->enet_stats.rx_packets++;
		rcv->skb = NULL;
#if NET_DEBUG > 2
			printk(KERN_DEBUG "%s: received all bytes\n", dev->name);
#endif
#ifdef PARANOID
                if( wait_fdata_termination( ecpdev , nl->fifo_timeout )!=E_MODE_OK ){
#if NET_DEBUG > 1
                        printk(KERN_DEBUG "%s: Timeout waiting RevTrans termination\n", dev->name);
#endif
                }
#endif
                /* Close the connection. */
                spin_lock_irqsave(&nl->lock, flags);
                set_mode_idle( ecpdev );

#if NET_DEBUG > 2
			printk(KERN_DEBUG "%s: received end\n", dev->name);
#endif

		if (snd->state != EPLIP_PK_DONE) {
#if NET_DEBUG > 3
                        printk(KERN_DEBUG"%s: Fetching Deferred SND Request\n",dev->name);
#endif
                        nl->connection = EPLIP_CN_SEND;
                        schedule_work(&nl->immediate);
                        spin_unlock_irqrestore(&nl->lock, flags);
			return OK;
		} else {
			nl->connection = EPLIP_CN_NONE;
			spin_unlock_irqrestore(&nl->lock, flags);
			return OK;
		}
	}
	return OK;
}


/*--------------------------------------------------------------------------*/
/** Low Level TX/RX routines */
__inline __u32
mk_chk( __u8 *buff , __u16 nbytes )
{
        int i ;
        __u32 chk=0;

        for( i=0 ; i < nbytes ; i++ )
                chk+= buff[i];

        return chk ;
}

/*--------------------------------------------------------------------------*/
__inline __u8
eplip_send_hard_header( struct eplip_local *snd , ecp_dev *dev , __u32 timeout )
{
#if NET_DEBUG > 3
        printk(KERN_DEBUG "PKLEN:%u\n",(__u16)snd->hh.h.length);
        printk(KERN_DEBUG "HCRC:%x\n" ,(__u16)snd->hh.h.checksum);
#endif
        outs( (dev->ioextent + PCI_DFIFO) , (snd->hh.raw) , EPLIP_HLEN );

        while( !(fifo_empty(dev) ) ){
                if( !(timeout--) )
                        return TIMEOUT ;
                udelay(1);
        }
/** check the last byte send to the FIFO and moved to the
  * tranciever has been actualy sent.
  */
        while( (timeout) && !(in(dev->iobase+SR) & SR_BUSY))
                timeout--;

        return timeout?OK:TIMEOUT ;
}
/*--------------------------------------------------------------------------*/
#ifdef EPLIP_16bit_FIFO_SUPPORT

__inline __u8
eplip_send_hard_header2( struct eplip_local *snd , ecp_dev *dev , __u32 timeout )
{
#if NET_DEBUG > 3
        printk(KERN_DEBUG "PKLEN:%u\n",(__u16)snd->hh.h.length);
        printk(KERN_DEBUG "HCRC:%x\n" ,(__u16)snd->hh.h.checksum);
#endif
        outsw( (dev->ioextent + PCI_DFIFO) , (__u16*)(snd->hh.raw) , EPLIP_HLEN/2 );

        while( !(fifo_empty(dev) ) ){

                if( !(timeout--) )
                        return TIMEOUT ;
                udelay(1);
        }
/** check the last byte send to the FIFO and moved to the
  * tranciever has been actualy sent.
  */
        while( (timeout) && !(in(dev->iobase+SR) & SR_BUSY))
                timeout--;

        return timeout?OK:TIMEOUT ;

}
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_send_hard_header4( struct eplip_local *snd , ecp_dev *dev , __u32 timeout )
{
#if NET_DEBUG > 3
        printk(KERN_DEBUG "PKLEN:%u\n",(__u16)snd->hh.h.length);
        printk(KERN_DEBUG "HCRC:%x\n" ,(__u16)snd->hh.h.checksum);
#endif
        outsl( (dev->ioextent + PCI_DFIFO) , (__u32*)(snd->hh.raw) , EPLIP_HLEN/4 );

        while( !(fifo_empty(dev) ) ){

                if( !(timeout--) )
                        return TIMEOUT ;
                udelay(1);
        }
/** check the last byte send to the FIFO and moved to the
  * tranciever has been actualy sent.
  */
        while( (timeout) && !(in(dev->iobase+SR) & SR_BUSY))
                timeout--;

        return timeout?OK:TIMEOUT ;

}
#endif
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_receive_hard_header( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout )
{
register __u8* data   = rcv->hh.raw;
register __u8  i      = EPLIP_HLEN;

        do {

                while( fifo_empty ( dev ) ) {
                        if(!(timeout--) ){
                                return TIMEOUT ;
                        }
                        udelay(1);
                }

                *(data++) = fifo_read_byte( dev ) ;
        } while( --i);

#if NET_DEBUG > 3
        printk(KERN_DEBUG "PKLEN:%u\n",(__u16)rcv->hh.h.length);
        printk(KERN_DEBUG "HCRC:%x\n" ,(__u16)rcv->hh.h.checksum);
#endif
        return OK ;
}

/*--------------------------------------------------------------------------*/
#ifdef  EPLIP_16bit_FIFO_SUPPORT

__inline __u8
eplip_receive_hard_header2( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout )
{
register __u16* data   = (__u16*)rcv->hh.raw;
register __u8   i      = EPLIP_HLEN/2;

        do {

                while( fifo_empty ( dev ) ) {
                        if(!(timeout--) ){
                                return TIMEOUT ;
                        }
                        udelay(1);
                }

                *(data++) = fifo_read_word( dev ) ;
        } while( --i);

#if NET_DEBUG > 3
        printk(KERN_DEBUG "PKLEN:%u\n",(__u16)rcv->hh.h.length);
        printk(KERN_DEBUG "HCRC:%x\n" ,(__u16)rcv->hh.h.checksum);
#endif
        return OK ;
}
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_receive_hard_header4( struct eplip_local *rcv , ecp_dev *dev , __u32 timeout )
{
register __u32* data   = (__u32*)rcv->hh.raw;
register __u8   i      = EPLIP_HLEN/4;

        do {

                while( fifo_empty ( dev ) ) {
                        if(!(timeout--) ){
                                return TIMEOUT ;
                        }
                        udelay(1);
                }

                *(data++) = fifo_read_long( dev ) ;
        } while( --i);

#if NET_DEBUG > 3
        printk(KERN_DEBUG "PKLEN:%u\n",(__u16)rcv->hh.h.length);
        printk(KERN_DEBUG "HCRC:%x\n" ,(__u16)rcv->hh.h.checksum);
#endif
        return OK ;
}

#endif
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_send_data(  struct eplip_local *snd ,ecp_dev *dev )
{
        __u16 burst_size = dev->fifo_depth;
        __u32 burst_count= snd->hh.h.length / burst_size;
        __u16 ioextent   = dev->ioextent ;
        __u8  *lbuff     = snd->skb->data;
        __u32 cx ;


        while( burst_count--) {

/** Wait the FIFO to become empty
  */            cx = EPLIP_FIFO_WAIT ;
                while( !(fifo_empty(dev) )) {

                        if( !(--cx) )
                                return HS_TIMEOUT ;
                }
/** Wtite FIFO_DEPTH pwords
  */
                outs( (ioextent+PCI_DFIFO) , lbuff , burst_size );
                lbuff+=burst_size;

        } ;

        if( (burst_size= snd->hh.h.length % burst_size) ) {
/** Wait the FIFO to become empty
  */            cx = EPLIP_FIFO_WAIT ;
                while( !fifo_empty(dev) ) {

                        if( !(--cx) )
                                return HS_TIMEOUT ;
                }

                outs((ioextent+PCI_DFIFO) , lbuff , burst_size );
        }

        return OK ;
}
/*--------------------------------------------------------------------------*/
#ifdef  EPLIP_16bit_FIFO_SUPPORT

__inline __u8
eplip_send_data2(  struct eplip_local *snd ,ecp_dev *dev )
{
        __u16 burst_size = dev->fifo_depth;
        __u32 burst_count= snd->hh.h.length / (burst_size*2);
        __u16 ioextent   = dev->ioextent ;
        __u16 *lbuff     = (__u16*)snd->skb->data;
        __u32 cx ;


        while( burst_count--) {

/** Wait the FIFO to become empty
  */            cx = EPLIP_FIFO_WAIT ;
                while( !(fifo_empty(dev) )) {

                        if( !(--cx) )
                                return HS_TIMEOUT ;
                }
/** Wtite FIFO_DEPTH pwords
  */
                outsw( (ioextent+PCI_DFIFO) , lbuff , burst_size );
                lbuff+=burst_size;

        } ;

        if( (burst_size= ( snd->hh.h.length % (burst_size*2))/2) ) {
/** Wait the FIFO to become empty
  */            cx = EPLIP_FIFO_WAIT ;
                while( !fifo_empty(dev) ) {

                        if( !(--cx) )
                                return HS_TIMEOUT ;
                }

                outsw((ioextent+PCI_DFIFO) , lbuff , burst_size );
        }

        return OK ;
}
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_send_data4(  struct eplip_local *snd ,ecp_dev *dev )
{
        __u16 burst_size = dev->fifo_depth;
        __u32 burst_count= snd->hh.h.length / (burst_size*4);
        __u16 ioextent   = dev->ioextent ;
        __u32 *lbuff     = (__u32*)snd->skb->data;
        __u32 cx ;


        while( burst_count--) {

/** Wait the FIFO to become empty
  */            cx = EPLIP_FIFO_WAIT ;
                while( !(fifo_empty(dev) )) {

                        if( !(--cx) )
                                return HS_TIMEOUT ;
                }
/** Wtite FIFO_DEPTH pwords
  */
                outsl( (ioextent+PCI_DFIFO) , lbuff , burst_size );
                lbuff+=burst_size;

        } ;

        if( (burst_size= (snd->hh.h.length % (burst_size*4))/4) ) {
/** Wait the FIFO to become empty
  */            cx = EPLIP_FIFO_WAIT ;
                while( !fifo_empty(dev) ) {

                        if( !(--cx) )
                                return HS_TIMEOUT ;
                }

                outsl((ioextent+PCI_DFIFO) , lbuff , burst_size );
        }

        return OK ;
}

#endif
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_receive_data( struct eplip_local *rcv , ecp_dev *dev )
{
        __u16 burst_size = dev->fifo_depth;
        __u32 burst_count= rcv->hh.h.length / burst_size;
        __u16 ioextent   = dev->ioextent ;
        __u8  *lbuff     = rcv->skb->data ;
        __u32 cx ;

        while( burst_count--) {
/** Wait the FIFO to become full
  */            cx = EPLIP_FIFO_WAIT ;
                while( !(fifo_full(dev)) ) {

                        if( !(--cx) )
                                return TIMEOUT ;
                        udelay(1);
                }
/** Read FIFO_DEPTH pwords
  */
                ins( (ioextent+PCI_DFIFO) , lbuff , burst_size );
                lbuff+=burst_size;

        } ;

        burst_count = rcv->hh.h.length % burst_size;

        cx = EPLIP_FIFO_WAIT ;
        while(  burst_count-- ) {
/** Wait for a byte available in the FIFO
  */
                 while( fifo_empty(dev) ) {

                        if( !(--cx) )
                                return TIMEOUT ;
                        udelay(1);
                }
/** Read a Byte from the FIFO
  */
                *(lbuff++)=fifo_read_byte(dev) ;

        };

        return OK ;
}
/*--------------------------------------------------------------------------*/
#ifdef  EPLIP_16bit_FIFO_SUPPORT

__inline __u8
eplip_receive_data2( struct eplip_local *rcv , ecp_dev *dev )
{
        __u16 burst_size = dev->fifo_depth;
        __u32 burst_count= rcv->hh.h.length / ( burst_size*2);
        __u16 ioextent   = dev->ioextent ;
        __u16 *lbuff     = (__u16*)rcv->skb->data ;
        __u32 cx ;

        while( burst_count--) {
/** Wait the FIFO to become full
  */            cx = EPLIP_FIFO_WAIT ;
                while( !(fifo_full(dev)) ) {

                        if( !(--cx) )
                                return TIMEOUT ;
                        udelay(1);
                }
/** Read FIFO_DEPTH pwords
  */
                insw( (ioextent+PCI_DFIFO) , lbuff , burst_size );
                lbuff+=burst_size;

        } ;

        burst_count = ( rcv->hh.h.length % ( burst_size*2))/2;

        cx = EPLIP_FIFO_WAIT ;
        while(  burst_count-- ) {
/** Wait for a byte available in the FIFO
  */
                 while( fifo_empty(dev) ) {

                        if( !(--cx) )
                                return TIMEOUT ;
                        udelay(1);
                }
/** Read a Word from the FIFO
  */
                *(lbuff++)=fifo_read_word(dev) ;

        };

        return OK ;
}
/*--------------------------------------------------------------------------*/
__inline __u8
eplip_receive_data4( struct eplip_local *rcv , ecp_dev *dev )
{
        __u16 burst_size = dev->fifo_depth;
        __u32 burst_count= rcv->hh.h.length / ( burst_size*4);
        __u16 ioextent   = dev->ioextent ;
        __u32 *lbuff     = (__u32*)rcv->skb->data ;
        __u32 cx ;

        while( burst_count--) {
/** Wait the FIFO to become full
  */            cx = EPLIP_FIFO_WAIT ;
                while( !(fifo_full(dev)) ) {

                        if( !(--cx) )
                                return TIMEOUT ;
                        udelay(1);
                }
/** Read FIFO_DEPTH pwords
  */
                insl( (ioextent+PCI_DFIFO) , lbuff , burst_size );
                lbuff+=burst_size;

        } ;

        burst_count = ( rcv->hh.h.length % ( burst_size*4))/4;

        cx = EPLIP_FIFO_WAIT ;
        while(  burst_count-- ) {
/** Wait for a byte available in the FIFO
  */
                 while( fifo_empty(dev) ) {

                        if( !(--cx) )
                                return TIMEOUT ;
                        udelay(1);
                }
/** Read a Word from the FIFO
  */
                *(lbuff++)=fifo_read_long(dev) ;

        };

        return OK ;
}

#endif
/*--------------------------------------------------------------------------*/
/*
 * Open/initialize the board. This is called (in the current kernel)
 * sometime after booting when the 'ifconfig' program is run.
 *
 * This routine should set everything up anew at each open, even
 * registers that "should" only need to be set once at boot, so that
 * there is non-reboot way to recover if something goes wrong.
 */
static int
eplip_open(struct net_device *dev)
{

        struct net_local *nl = netdev_priv(dev);
	struct in_device *in_dev;
        unsigned long   intrflags = ( nl->dev->flags & ECPDEV_IRQ_TRIGGER ) ? IRQF_SHARED : 0;

        nl->connection=EPLIP_CN_NONE;

        ecp_open( nl->dev );

        /* Claim the IRQ line */
        if (request_irq(dev->irq, &eplip_interrupt, intrflags, dev->name, dev)) {
		return -EAGAIN;
	}

        nl->dev->flags |=ECPDEV_IRQ_CLAIMED ;

	/*
	 * Always allocate the DMA channel after the IRQ,
	 * and clean up on failure.
	 */

	if( dev->dma ){
	        if (request_dma(dev->dma, dev->name)) {
		        free_irq(dev->irq, dev);
                        nl->dev->flags &= ~ECPDEV_IRQ_CLAIMED ;
		        return -EAGAIN;
	        }
                nl->dev->flags|=ECPDEV_DMA_CLAIMED ;
        }

        /* Initialize the state machine. */
        nl->rcv_data.state = nl->snd_data.state = EPLIP_PK_DONE;
	nl->rcv_data.skb = nl->snd_data.skb = NULL;
	nl->connection = EPLIP_CN_NONE;
        nl->dma_state = EPLIP_DMA_NONE;
        nl->is_deferred = 0;

        /* Fill in the MAC-level header.
	   We used to abuse dev->broadcast to store the point-to-point
	   MAC address, but we no longer do it. Instead, we fetch the
	   interface address whenever it is needed, which is cheap enough
	   because we use the hh_cache. Actually, abusing dev->broadcast
	   didn't work, because when using eplip_open the point-to-point
	   address isn't yet known.
	   EPLIP doesn't have a real MAC address, but we need it to be
	   DOS compatible, and to properly support taps (otherwise,
	   when the device address isn't identical to the address of a
	   received frame, the kernel incorrectly drops it).             */

	if ((in_dev=dev->ip_ptr) != NULL) {
		/* Any address will do - we take the first. We already
		   have the first two bytes filled with 0xfc, from
		   eplip_init_dev(). */
                struct in_ifaddr *ifa=in_dev->ifa_list;

                if (ifa != NULL &&  ! (memcmp(dev->dev_addr+2,"\x0\x0\x0\x0",4) )) {
                        memcpy(dev->dev_addr+2, &ifa->ifa_local, 4);
		}
	}
/* Reset the HWD. Set Reverse Idle Mode */

        set_mode_idle( nl->dev );

        /** Start acception queued packets */
        netif_start_queue (dev);

        try_module_get(THIS_MODULE);

	return 0;
}

/* The inverse routine to net_open(). */
static int
eplip_close(struct net_device *dev)
{
	struct net_local *nl = netdev_priv(dev);
        struct eplip_local *snd = &nl->snd_data;
	struct eplip_local *rcv = &nl->rcv_data;

	netif_stop_queue(dev);

        if( nl->dev->flags & ECPDEV_DMA_CLAIMED )
	        disable_dma(dev->dma);

/** There is no need to check for ECPDEV_IRQ_CLAIMED
  * eplip_open() will fail if it can not claim an IRQ line
  * for the target device, so at time eplip_close() is called
  * this flag will be always set. But disable_irq() should be
  * called only if our HWD does not share the IRQ line(FIX it)
  */
        disable_irq(dev->irq);
        synchronize_irq(dev->irq);

	nl->is_deferred = 0;
	nl->connection = EPLIP_CN_NONE;
        snd->state = EPLIP_PK_DONE;
	if (snd->skb) {
		dev_kfree_skb(snd->skb);
		snd->skb = NULL;
	}
	rcv->state = EPLIP_PK_DONE;
	if (rcv->skb) {
		kfree_skb(rcv->skb);
		rcv->skb = NULL;
	}

        if( nl->dev->flags & ECPDEV_DMA_CLAIMED )
                free_dma(nl->dev->dma);

        if( nl->dev->flags & ECPDEV_IRQ_CLAIMED )
                free_irq(nl->dev->irq, dev);

        nl->dev->flags &=~(ECPDEV_IRQ_CLAIMED|ECPDEV_DMA_CLAIMED);

        ecp_close(nl->dev);

        module_put(THIS_MODULE);

        return 0;

}

/* Returns NET statistics
 *
 */
static struct net_device_stats *
eplip_get_stats(struct net_device *dev)
{
	struct net_local *nl = netdev_priv(dev);
	struct net_device_stats *r = &nl->enet_stats;

	return r;
}

static int eplip_change_mtu(struct net_device *dev, int new_mtu)
{
        struct net_local *nl = netdev_priv(dev);


        spin_lock(&nl->lock);
        if( netif_running(dev) ) {
                if( nl->snd_data.state!=EPLIP_PK_DONE )
                {
#if NET_DEBUG > 2
                        printk(KERN_DEBUG"%s:change_mtu() while TX in progress\n",dev->name);
#endif
                        spin_unlock(&nl->lock);
                        return -EBUSY;
                }
        }
	dev->mtu = new_mtu;
        spin_unlock(&nl->lock);
	return 0;
}

#ifdef  EPLIP_16bit_FIFO_SUPPORT
static int
eplip_ioctl(struct net_device *dev, struct ifreq *rq, int cmd)
{
	struct net_local *nl = netdev_priv(dev);
	struct plipconf *pc = (struct plipconf *) &rq->ifr_data;

	switch(pc->pcmd) {
	case PLIP_GET_TIMEOUT:
                pc->trigger = nl->trigger_timeout;
		pc->nibble  = nl->fifo_timeout/nl->dev->fifo_depth;
		break;
	case PLIP_SET_TIMEOUT:
                if(!capable(CAP_NET_ADMIN))
			return -EPERM;
		nl->trigger_timeout = pc->trigger;
		nl->fifo_timeout  = pc->nibble*nl->dev->fifo_depth;
               	break;
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}
#endif
/*
 * The typical workload of the driver:
 * Handle the network interface interrupts.
 */
static irqreturn_t
eplip_interrupt(int irq, void *dev_id)
{
        struct net_device *dev = dev_id;
	struct net_local *nl;
	struct eplip_local *rcv;
	unsigned long flags;

#if NET_DEBUG > 2
	if (dev == NULL) {
		printk(KERN_DEBUG "%s:interrupt: irq %d for unknown device.\n",dev->name, irq);
		return;
	}
#endif
	nl = netdev_priv(dev);
	rcv = &nl->rcv_data;

	spin_lock_irqsave (&nl->lock, flags);

        switch (nl->connection) {

        case EPLIP_CN_SEND:
                if( nl->dma_state == EPLIP_DMA_SEND )  {

                        __u8 ecr = read_econtrol(nl->dev);
                        if( ecr & SRVC_INTR ){
                                nl->dma_state = EPLIP_DMA_TERM ;
                                schedule_work(&nl->immediate);

#if NET_DEBUG > 3
                                printk(KERN_DEBUG "%s: DMA Terminatinal Count Interrupt\n",dev->name);
#endif
                        }
                        else {
#if NET_DEBUG > 2
                                printk(KERN_DEBUG "%s Interrupt in DMA Send but SRVC_INTR cleared\n" , dev->name);
#endif
                        }
                        break;
                }
      	case EPLIP_CN_NONE:

                /** Note that there exists a race condition,
                  * where we are executing eplip_bh(), we are
                  * not yet called the exact bottom half handler
                  * for the immediate request ( thats calling connection_state_table[nl->connection]] )
                  * the state is set to EPLIP_CN_SEND , an interrupt occures
                  * and we got here. Now we set nl->connection=EPLIP_CN_RECEIVE and
                  * reschedule the bottom half handler for execution,
                  * and when the eplip_bh() resumes it's execution it will call
                  * connection_state_table[EPLIP_CN_RECEIVE]. The bottom half
                  * is still queued in the BH queue, since we've just schedule it.
                  * This race( I think ) may occure in the original plip too, but
                  * while with the original plip it doesn't hurt( plip_receive_packet()
                  * will not yeld the CPU(s) until it sets nl->connection=EPLIP_CN_NONE
                  * ( or EPLIP_CN_ERROR ) and plip_bh will call plip_none(), with eplip
                  * which does __not__ spin_lock() for the whole execution of eplip_receive_packet()
                  * ( note that eplip_bh is guaranteed to never be reentered - by means of
                  * task scheduling policies ), but in the case of DMA receive eplip_bh() will
                  * return after eplip_receive_packet() has programmed the DMAC for the transfer,
                  * and will be called for second time before the DMA rcv has finished/timeouted.
                  */

#if NET_DEBUG > 2
                printk(KERN_DEBUG "%s: will accept REQ.\n", dev->name);
#endif
                rcv->state = EPLIP_PK_TRIGGER;
		nl->connection = EPLIP_CN_RECEIVE;
                schedule_work(&nl->immediate);

		break;

	case EPLIP_CN_RECEIVE:
		/* May occur because there is race condition
		   around test and set of dev->interrupt.
		   Ignore this interrupt.
                   Also occures on DMA Terminal Count Reached while DMA RCVing
                */

                if( nl->dma_state == EPLIP_DMA_RECEIVE ){

                        __u8 ecr = read_econtrol(nl->dev);
                        if( ecr & SRVC_INTR ){
                                nl->dma_state = EPLIP_DMA_TERM ;
                                schedule_work(&nl->immediate);
#if NET_DEBUG > 3
                                printk(KERN_DEBUG "%s: DMA Terminatinal Count Interrupt\n",dev->name);
#endif
                        }
                        else {
#if NET_DEBUG > 2
                                printk(KERN_DEBUG "%s Interrupt in DMA Received but SRVC_INTR cleared\n" , dev->name);
#endif
                        }

                }
                else {
#if NET_DEBUG > 2
                        printk(KERN_DEBUG "%s: Interrupt in RCV state.\n", dev->name);
#endif
                }
		break;
        }
	spin_unlock_irqrestore(&nl->lock, flags);
	return IRQ_HANDLED;
}

/*****************************************************************************/
/*      MODULE Initialization/Cleanup/Parameters Stuff
 */

static int  io[EPLIP_MAX] = { [0 ... EPLIP_MAX-1] = -1 };
static int irq[EPLIP_MAX] = { [0 ... EPLIP_MAX-1] = -1 };
static int dma[EPLIP_MAX] = { [0 ... EPLIP_MAX-1] = -1 };
static int mode[EPLIP_MAX]= { [0 ... EPLIP_MAX-1] = -1 };
static unsigned long hwaddr[EPLIP_MAX] =  { [0 ... EPLIP_MAX-1] =  0 };

module_param_array(io , int, NULL, 0);
module_param_array(irq, int, NULL, 0);
module_param_array(dma, int, NULL, 0);
module_param_array(mode, int, NULL, 0);
module_param_array(hwaddr, ulong, NULL, 0);

MODULE_PARM_DESC(io  , "The Base IO port this device uses") ;
MODULE_PARM_DESC(irq , "The IRQ Line this device uses");
MODULE_PARM_DESC(dma , "The DMA Chanel this device uses");
MODULE_PARM_DESC(mode, "The Operation Mode for eplip driver for this device");
MODULE_PARM_DESC(hwaddr , "The rightmost four octets of the MAC address");

MODULE_AUTHOR ("Angel Valkov <whitefang@dir.bg>");
MODULE_DESCRIPTION ("Enhanced PLIP Driver");
MODULE_LICENSE("GPL");

static __inline void
cleanup_ecp_dev( ecp_dev  *dev )
{
        if( dev != NULL ) {
                if ( dev ->iobase  ) {

                        if( dev->flags & ECPDEV_IOEXT_CLAIMED  ) {
                                release_region ( dev->ioextent , IO_EXTENT_ECP );
                                dev->flags &=(~ECPDEV_IOEXT_CLAIMED ) ;
                        }
                        release_region( dev->iobase,IO_EXTENT_BASE ) ;
                        dev->flags &= (~ ECPDEV_IOBASE_CLAIMED )  ;
                        dev->iobase=0;
                }
        }
}

static void
cleanup(struct net_device *eplip_net_dev[])
{
        int i  = EPLIP_MAX ;
        while( i-- ) {
		if (eplip_net_dev[i]) {

                        struct net_local *nl = netdev_priv(eplip_net_dev[i]);

			if (!nl) continue;

                        unregister_netdev(eplip_net_dev[i]);

                        if( nl->bounce_buff ) {
                                kfree( nl->bounce_buff );
                        }

                        cleanup_ecp_dev( nl->dev );
                        kfree( nl->dev);

                        /*kfree( eplip_net_dev[i].priv );
                        eplip_net_dev[i].priv = NULL;*/

                }

#if 0
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0)
#else
                if( eplip_net_dev[i].name ) {
                        kfree(eplip_net_dev[i].name);
                        eplip_net_dev[i].name=NULL;
                }
#endif
#endif
        }

}

static int
eplip_probe_dev( ecp_dev* dev , int user_irq , int user_dma  )

{
        if( (!dev) || (!dev->iobase) || (!dev->iobase < 0 )) {

                printk(KERN_ERR "%s: I've hit a BUG\n",driver);
                return -ENOSYS ;

        }

        if( !request_region( dev->iobase , IO_EXTENT_BASE, driver  ) ) {
                printk(KERN_ERR "%s: BASEIO:%x Already Claimed by another device\n",driver,dev->iobase);
                return -EAGAIN ;
        }
        dev->flags |= ECPDEV_IOBASE_CLAIMED;


/** We do not support PCI relocation yet
  * So set the dev->ioextent according to the ISA ECP spec.
  */
        dev->ioextent = dev->iobase + 0x400 ;
        if( !request_region( dev->ioextent,IO_EXTENT_ECP, driver ) ) {
                printk(KERN_ERR "%s: ECPIO:%x Already Claimed by another device\n",driver,dev->iobase+0x400);
                return -EAGAIN ;
        }

        dev->flags |=ECPDEV_IOEXT_CLAIMED;

        getPCAPS( dev );

        if( dev->flags & ECPDEV_SMODE_ECP ) {

                if( simple_probe( dev ) == EOK ){

                        if(  dev->pword!=8   ){
#ifdef  EPLIP_16bit_FIFO_SUPPORT
                                if( dev->pword!=16)
#endif
                                {
                                        printk(KERN_ERR "%s: %ibit PWORD for ECP HWD @iobase %#3x not supported.\n",driver,dev->pword,dev->iobase);
                                        return -ENOSYS ;
                                }
                        }
                        if( !((__s16)dev->irq > 0 ) ) {
                                if( user_irq > 0 )
                                        dev->irq=user_irq;
                                else
                                        return -EAGAIN;
                        }
                        if( !((__s16)dev->dma > 0 ) ) {
                                if( user_dma > 0 )
                                        dev->dma=user_dma;
                                else
                                        dev->dma = 0;
                        }
                        if( dev->dma && ( dev->mode == EPLIP_MODE_ECP_DMA) ) {
                                if( ( dev->pword == 8 ) && ( dev->dma > 3 ) ){
                                        printk(KERN_INFO "%s: 16bit DMA chan(%i) for 8bit ECP HWD @iobase:%#3x.Falling to PIO ECP FIFO\n",driver,dev->dma,dev->iobase);
                                        dev->dma  = 0;
                                        dev->mode = EPLIP_MODE_ECP_FIFO;
                                }
                                else{
                                        if( ( dev->pword == 16 ) && ( dev->dma < 4 ) ){
                                                printk(KERN_INFO "%s: 8bit DMA chan(%i) for 16bit ECP HWD @iobase:%#3x.Falling to PIO ECP FIFO\n",driver,dev->dma,dev->iobase);
                                                dev->dma  = 0;
                                                dev->mode = EPLIP_MODE_ECP_FIFO;
                                        }
                                }
                        }

                        if( dev->dma )
                                printk(KERN_INFO"%s: ECP HWD at IOBASE:%#3x irq:%u dma:%u\n",driver,dev->iobase,dev->irq,dev->dma) ;
                        else
                                printk(KERN_INFO"%s: ECP HWD at IOBASE:%#3x irq:%u\n",driver,dev->iobase,dev->irq) ;
                }
                else{
                        printk(KERN_ERR "%s: ECP HWD probing routine failed.It's bogus\n",driver);
                        return -ENOSYS ;
                }
        }
        else {
                printk(KERN_ERR"%s: NO ECP HWD at IOBASE:%#3x\n", driver,dev->iobase);
                return -EAGAIN ;
        }

        return OK ;
}

static struct net_device *dev_eplip[EPLIP_MAX] ;

/* Initialize the module */
static int
eplip_init(void)
{
	int i = -1 ;
        int ndevs = 0 ;
        int error = 0 ;
	char name[IFNAMSIZ];


	printk(KERN_INFO "%s-%s(%s) by %s(%s)\n",driver,version,date,author,email);

        memset( dev_eplip , 0 , (sizeof( struct net_device *) )* EPLIP_MAX ) ;

        while(  ( (++i) < EPLIP_MAX )  && ( io[i] > 0 ) ) {

        	ecp_dev* dev = 0L ;
		sprintf(name, "eplip%d", i);
		dev_eplip[i] = alloc_etherdev(sizeof(struct net_local));
		if (!dev_eplip[i]) {
			printk(KERN_ERR "eplip: memory squeeze\n");
			return -ENOMEM;
		}
		strcpy(dev_eplip[i]->name, name);

        	dev = kmalloc(sizeof ( ecp_dev ) , GFP_KERNEL ) ;
        	if( !dev ) {
                	printk( KERN_ERR"%s Memory squize \n", driver ) ;
                	cleanup( dev_eplip ) ;
                	return -ENOMEM ;
         	}

         	memset( dev , 0 , sizeof ( ecp_dev ));
         	dev->iobase = io[i];

                switch( mode[i] ){
                        case EPLIP_MODE_ECP_FIFO:
                        case EPLIP_MODE_ECP_DMA :
#ifdef FIFO_TEST_SUPPORT
                        case EPLIP_MODE_TEST    :
#endif
                                                                                        dev->mode   = mode[i];
                                                                                        break;
                        case EPLIP_MODE_NIBBLE  :
                        case EPLIP_MODE_ECP_BYTE:
                        case EPLIP_MODE_ECP_SPP : printk(KERN_INFO "%s: Mode %#x Not yet supported.Falling to PIO ECP FIFO mode.\n",driver,mode[i] );

                        default                 : dev->mode = EPLIP_MODE_ECP_DMA;	//EPLIP_MODE_ECP_FIFO ;
                }

                if( ( error = eplip_probe_dev( dev , irq[i] , dma[i] )  ) ) {

                        cleanup_ecp_dev(dev) ;
                        kfree(dev);
        		cleanup( dev_eplip ) ;
         		return error;
         	}

#ifdef FIFO_TEST_SUPPORT
                if( dev->mode!=EPLIP_MODE_TEST ) {
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,4,0)
#else
/*
		        dev_eplip[i].name = kmalloc(strlen("eplipXXX"), GFP_KERNEL);
		        if (!dev_eplip[i].name) {
			        printk(KERN_ERR "%s: memory squeeze.\n",driver);
                                cleanup_ecp_dev(dev);
                                kfree(dev);
			        cleanup( dev_eplip );
			        return -ENOMEM ;
		        }
*/
#endif
/*		        sprintf(dev_eplip[i].name,"eplip%d", i ); */

                        if ( ( error = eplip_init_dev(dev_eplip[i], dev, hwaddr[i]/*,crc[i]*/ ) ) ) {

                                cleanup_ecp_dev(dev);
                                kfree(dev);
                                cleanup( dev_eplip );
                                return error;
		        }
                        //((struct net_local*)dev_eplip[i].priv)->uses_adler32crc = crc[i];

                        if( ( dev->mode == EPLIP_MODE_ECP_DMA ) && !(dev_eplip[i]->dma) )
                                dev->mode = EPLIP_MODE_ECP_FIFO ;

                        if ((error = register_netdev(dev_eplip[i]))) {

                                printk(KERN_ERR "%s: Failed to register device:%s\n",driver, dev_eplip[i]->name);
                                cleanup( dev_eplip );
                                return error;
                        }
#ifdef FIFO_TEST_SUPPORT
                }
                else {
                        int error;
                        printk(KERN_INFO"%s: EPLIP_MODE_TEST for ECP HWD @iobase:%#3x.Won't register net device\n",driver,dev->iobase);
                        printk(KERN_INFO"%s: Running \"falling blind\" algorithm on the FIFO:",driver);

                        if( set_mode_test( dev )!=0 ) {
                                printk("Could'n set TEST mode\n");
                        }
                        else {
                                if( (error = fifo_falling_blind(dev,0x55555555) ) > 0 ) {
                                        printk("%i pwords w/r OK\n",error);
                                }
                                else
                                        switch( error ) {
                                                case  0 :printk("Two many PWORDs w/r\n");      break;
                                                case -1 :printk("FULL/EMPTY trigger stuck\n"); break;
                                                case -2 :printk("PWORD pattern mismatch\n");   break;
                                                case -3 :printk("N:of PWORDS w/r differ\n");   break;
                                                case -4 :printk("Invalid PWORD size\n");       break;
                                                default :printk("Unknown error\n");            break;
                                        }
                        }

                        ecp_close( dev );
                        cleanup_ecp_dev(dev);
                        kfree(dev);
                }
#endif
                ndevs++;
        }

        if( !i ) {
      		printk ( KERN_ERR"%s:no iobase address.Try passing io=0xXXX as minimum module parameter\n",driver );
                return -ENXIO ;
        }
        if( !ndevs ){
                cleanup( dev_eplip );
                return -ENXIO ;
        }

        return OK ;
}

/* Cleanup - undid whatever init_module did */
static void
eplip_exit(void)
{
        cleanup( dev_eplip ) ;
}


#ifdef MODULE

module_init( eplip_init )       ;
module_exit( eplip_exit )       ;

#endif
/*****************************************************************************/
