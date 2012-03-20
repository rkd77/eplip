#ifndef _SYSDEP_H_
#define _SYSDEP_H_

#ifndef LINUX_VERSION_CODE
#  include <linux/version.h>
#endif

#ifndef KERNEL_VERSION /* pre-2.1.90 didn't have it */
#  define KERNEL_VERSION(vers,rel,seq) ( ((vers)<<16) | ((rel)<<8) | (seq) )
#endif

/* only allow  2.2.y and 2.4.z */

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,2,0) /* not < 2.2 */
#  error "This kernel is too old: not supported by this file"
#endif

//#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,5,0) /* not > 2.4, by now */
//#  error "This kernel is too recent: not supported by this file"
//#endif
#if (LINUX_VERSION_CODE & 0xff00) == 3 /* not 2.3 */
#  error "Please don't use linux-2.3, use 2.4 instead"
#endif

/*
 * MODULE handling changes
 */
#ifndef SET_MODULE_OWNER
#  define SET_MODULE_OWNER(structure) /* nothing */
#endif
/*
 * Softnet changes in 2.4
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,4,0)
#  ifdef _LINUX_NETDEVICE_H /* only if netdevice.h was included */
#  define netif_start_queue(dev)        clear_bit(0, (void *) &(dev)->tbusy)
#  define netif_stop_queue(dev)         set_bit(0, (void *) &(dev)->tbusy)
#  define netif_queue_stopped(dev)      test_bit(0,(void *) &(dev)->tbusy)
#  define netif_running(dev)            test_bit(0, (void *) &(dev)->start)
static inline void netif_wake_queue(struct device *dev)
{
        clear_bit(0, (void *) &(dev)->tbusy);
        mark_bh(NET_BH);
}

/** Error codes that hard_start_xmit() may return                       */
#  define NET_XMIT_SUCCESS	0
#  define NET_XMIT_DROP		1	/* skb dropped			*/
#  define NET_XMIT_CN		2	/* congestion notification	*/
#  define NET_XMIT_POLICED	3	/* skb is shot by police	*/
#  define NET_XMIT_BYPASS	4	/* packet does not leave via dequeue;*/

/* struct device became struct net_device */
#  define net_device device
#  endif /* netdevice.h */

/*
 * skbuff changes in 2.4( especially __dev_alloc_skb() )
 */
#ifdef HAVE_ALLOC_SKB /* only if skbuff.h was included */

static inline struct sk_buff *__dev_alloc_skb(unsigned int length,
					      int gfp_mask)
{
	struct sk_buff *skb;

	skb = alloc_skb(length+16, gfp_mask);
	if (skb)
		skb_reserve(skb,16);
	return skb;
}

#endif

/** MODULE_LICENSE macro is not present on kernels prior to 2.4*/
#ifndef MODULE_LICENSE
#	define MODULE_LICENSE(a)
#endif

#endif /* ! LINUX_24 */

#endif /* _SYSDEP_H_ */
