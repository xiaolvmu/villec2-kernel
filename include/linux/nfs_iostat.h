/*
 *  User-space visible declarations for NFS client per-mount
 *  point statistics
 *
 *  Copyright (C) 2005, 2006 Chuck Lever <cel@netapp.com>
 *
 *  NFS client per-mount statistics provide information about the
 *  health of the NFS client and the health of each NFS mount point.
 *  Generally these are not for detailed problem diagnosis, but
 *  simply to indicate that there is a problem.
 *
 *  These counters are not meant to be human-readable, but are meant
 *  to be integrated into system monitoring tools such as "sar" and
 *  "iostat".  As such, the counters are sampled by the tools over
 *  time, and are never zeroed after a file system is mounted.
 *  Moving averages can be computed by the tools by taking the
 *  difference between two instantaneous samples  and dividing that
 *  by the time between the samples.
 */

#ifndef _LINUX_NFS_IOSTAT
#define _LINUX_NFS_IOSTAT

#define NFS_IOSTAT_VERS		"1.1"

enum nfs_stat_bytecounters {
	NFSIOS_NORMALREADBYTES = 0,
	NFSIOS_NORMALWRITTENBYTES,
	NFSIOS_DIRECTREADBYTES,
	NFSIOS_DIRECTWRITTENBYTES,
	NFSIOS_SERVERREADBYTES,
	NFSIOS_SERVERWRITTENBYTES,
	NFSIOS_READPAGES,
	NFSIOS_WRITEPAGES,
	__NFSIOS_BYTESMAX,
};

enum nfs_stat_eventcounters {
	NFSIOS_INODEREVALIDATE = 0,
	NFSIOS_DENTRYREVALIDATE,
	NFSIOS_DATAINVALIDATE,
	NFSIOS_ATTRINVALIDATE,
	NFSIOS_VFSOPEN,
	NFSIOS_VFSLOOKUP,
	NFSIOS_VFSACCESS,
	NFSIOS_VFSUPDATEPAGE,
	NFSIOS_VFSREADPAGE,
	NFSIOS_VFSREADPAGES,
	NFSIOS_VFSWRITEPAGE,
	NFSIOS_VFSWRITEPAGES,
	NFSIOS_VFSGETDENTS,
	NFSIOS_VFSSETATTR,
	NFSIOS_VFSFLUSH,
	NFSIOS_VFSFSYNC,
	NFSIOS_VFSLOCK,
	NFSIOS_VFSRELEASE,
	NFSIOS_CONGESTIONWAIT,
	NFSIOS_SETATTRTRUNC,
	NFSIOS_EXTENDWRITE,
	NFSIOS_SILLYRENAME,
	NFSIOS_SHORTREAD,
	NFSIOS_SHORTWRITE,
	NFSIOS_DELAY,
	NFSIOS_PNFS_READ,
	NFSIOS_PNFS_WRITE,
	__NFSIOS_COUNTSMAX,
};

enum nfs_stat_fscachecounters {
	NFSIOS_FSCACHE_PAGES_READ_OK,
	NFSIOS_FSCACHE_PAGES_READ_FAIL,
	NFSIOS_FSCACHE_PAGES_WRITTEN_OK,
	NFSIOS_FSCACHE_PAGES_WRITTEN_FAIL,
	NFSIOS_FSCACHE_PAGES_UNCACHED,
	__NFSIOS_FSCACHEMAX,
};

#endif	
