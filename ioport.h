/***************************************************************************
                          ioport.h  -  IO port access macro definitions
                             -------------------
        begin      	: Wed May 3 2000
	last change	: Sat May 19 2001
	ver         	: 0.4
        copyright  	: (C) 2001 by Angel Valkov
        email      	: whitefang@dir.bg
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
        CHANGES	:
	        ver.0.1	Initial
                ver.0.2	Added MS-DOS and MS-Windows support
		ver.0.3	Cosmetic changes.
                ver.0.4 Adaptions for GLibC 2.2 and KERNEL level usage
 ***************************************************************************/

#ifndef IOPORT_H
#define IOPORT_H

#ifdef linux	        /** Under Linux
                          */

#ifdef __KERNEL__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <asm/io.h>

#else

#include <sys/types.h>  /** Just to get __GLIBC__ macros
                          */

#ifdef debug											
#define extern 		/** This defines "extern" as empty,which is required,
			  * in order GCC to include IO Port access macros in
                          * the source,	if optimizations are turned off
                          * for debuging reasons.
                          */

#endif 			/** Otherwise you should compile with optimizations turned on
                          */

#if __GLIBC__ >= 2
#include <gnu/libc-version.h>
#if __GLIBC_MINOR__ >= 2
#include <sys/io.h>	/** Contains declarations of IO Port routines
                          */
#else

#include <asm/io.h>     /** GLIBC versions prior to 2.2 have io.h in asm/
                          * GLIBC > 2.2 have architecture dependant stuff in sys/
                          * and the asm/ is a symlink to linux/include/asm
                          */
#endif
#endif

#endif                  /** #ifdef __KERNEL__
                          */
#define	in(port)		inb(port)
#define out(value,port)         outb(value,port)
#define ins(port,addr,count)    insb(port,addr,count)
#define outs(port,addr,count)   outsb(port,addr,count)

#else

#ifdef dos 		/** Under MS-DOS
                          */

#include <conio.h>      /** Contains declarations of IO Port routines
                          */
#define in(port)	inp(port)
#define out(value,port) outp(port,value)

#define ins(port,addr,count)    
#define outs(port,addr,count)   

#else

#ifdef _WIN32	        /** Under Windows9x-Not WinNT or 2K(ila-Boza)
                          */
extern __inline unsigned char	__fastcall	_inb  ( unsigned short port); 
extern __inline unsigned short	__fastcall	_inw  ( unsigned short port); 
extern __inline unsigned long	__fastcall	_inl  ( unsigned short port);
extern __inline void		__fastcall	_outb ( unsigned char  value , unsigned short port);
extern __inline void		__fastcall	_outw ( unsigned short value , unsigned short port);
extern __inline void		__fastcall	_outl ( unsigned long  value , unsigned short port);
extern __inline void		__fastcall	_outsb( unsigned long  count , unsigned short port, unsigned char*  addr );
extern __inline void		__fastcall	_outsw( unsigned long  count , unsigned short port, unsigned short* addr );
extern __inline void		__fastcall	_outsl( unsigned long  count , unsigned short port, unsigned long*  addr );
extern __inline void		__fastcall	_insb ( unsigned long  count , unsigned short port, unsigned char*  addr );
extern __inline void		__fastcall	_insw ( unsigned long  count , unsigned short port, unsigned short* addr );
extern __inline void		__fastcall	_insl ( unsigned long  count , unsigned short port, unsigned long*  addr );

#define in(port)		_inb((unsigned short)(port))
#define inw(port)               _inw((unsigned short)(port))
#define inl(port)               _inl((unsigned short)(port))
#define out(value,port)		_outb((unsigned char)(value),(unsigned short)(port))
#define outw(value,port)        _outw((unsigned char)(value),(unsigned short)(port))
#define outl(value,port)        _outl((unsigned char)(value),(unsigned short)(port))

#define ins(port,addr,count)	_insb(count,(unsigned short)(port),addr)
#define insw(port,addr,count)   _insw(count,(unsigned short)(port),addr)
#define insl(port,addr,count)   _insl(count,(unsigned short)(port),addr)
#define outs(port,addr,count)   _outsb(count,(unsigned short)(port),addr)
#define outsw(port,addr,count)  _outsw(count,(unsigned short)(port),addr)
#define outsl(port,addr,count)  _outsl(count,(unsigned short)(port),addr)


#endif			/** ifdef _WINDOWS
                          */
#endif		 	/** ifdef dos
                          */
#endif     		/** ifdef linux
                          */
#endif		 	/** ifndef IOPORT_H
                          */