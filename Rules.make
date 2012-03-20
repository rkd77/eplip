# -*-makefile-*-
#
# This file is part of the sample code for the book "Linux Device Drivers",
# second edition. It is meant to be generic and is designed to be recycled
# by other drivers. The comments should be clear enough.
# It partly comes from Linux Makefile, and needs GNU make. <rubini@linux.it>

# TOPDIR is declared by the Makefile including this file.
ifndef TOPDIR
	TOPDIR = .
endif

# KERNELDIR can be speficied on the command line or environment
ifndef KERNELDIR
	KERNELDIR = /usr/src/linux
endif
# The headers are taken from the kernel
	INCLUDEDIR = $(KERNELDIR)/include

# We need the configuration file, for CONFIG_SMP and possibly other stuff
# (especiall for RISC platforms, where CFLAGS depends on the exact
# processor being used).
ifeq ($(KERNELDIR)/.config,$(wildcard $(KERNELDIR))/.config)
	include $(KERNELDIR)/.config
else
	MESSAGE := $(shell echo "WARNING: no .config file in $(KERNELDIR)")
endif

# ARCH can be speficed on the comdline or env. too, and defaults to this arch
# Unfortunately, we can't easily extract if from kernel configuration
# (well, we could look athe asm- symlink... don't know if worth the effort)
ifndef ARCH
  ARCH := $(shell uname -m | sed -e s/i.86/i386/ -e s/sun4u/sparc64/ \
		-e s/arm.*/arm/ -e s/sa110/arm/)
endif

# This is useful if cross-compiling. Taken from kernel Makefile (CC changed)
AS      =$(CROSS_COMPILE)as
LD      =$(CROSS_COMPILE)ld
CC      =$(CROSS_COMPILE)gcc
CPP     =$(CC) -E
AR      =$(CROSS_COMPILE)ar
NM      =$(CROSS_COMPILE)nm
STRIP   =$(CROSS_COMPILE)strip
OBJCOPY =$(CROSS_COMPILE)objcopy
OBJDUMP =$(CROSS_COMPILE)objdump

# The platform-specific Makefiles include portability nightmares.
# Some platforms, though, don't have one, so check for existence first
ARCHMAKEFILE = $(TOPDIR)/Makefile.$(ARCH)
ifeq ($(ARCHMAKEFILE),$(wildcard $(ARCHMAKEFILE)))
  include $(ARCHMAKEFILE)
endif

#Install dir
VERSIONFILE = $(INCLUDEDIR)/linux/version.h
VERSION     = $(shell awk -F\" '/REL/ {print $$2}' $(VERSIONFILE))
VERSION_MINOR=$(shell echo $(VERSION)| cut -c3)
ifeq ($(VERSION_MINOR),2)
	INSTALLDIR = /lib/modules/$(VERSION)
else
	INSTALLDIR = /lib/modules/$(VERSION)/kernel/drivers
endif

ifdef CONFIG_X86
#	include $(KERNELDIR)/arch/$(ARCH)/Makefile
	CFLAGS=
	include Rules.arch

ifndef CONFIG_ISA
ifeq ($(VERSION_MINOR),2)
else
	KCONFIG_ERROR := $(shell echo "ERROR: You Should have enabled ISA support")
endif
endif

else
	KCONFIG_ERROR := $(shell echo "ERROR: This module is intended only for x86 machines")
endif

# CFLAGS: all assignments to CFLAGS are inclremental, so you can specify
# the initial flags on the command line or environment, if needed.

	CFLAGS +=  -Wall -Wstrict-prototypes  -Wwrite-strings -D__KERNEL__ -DMODULE -I$(INCLUDEDIR)

ifdef CONFIG_SMP
	CFLAGS += -D__SMP__ -DSMP
endif

# Prepend modversions.h if we're running with versioning.
ifdef CONFIG_MODVERSIONS
	CFLAGS += -DMODVERSIONS -include $(KERNELDIR)/include/linux/modversions.h
endif
