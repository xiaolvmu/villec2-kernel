/*
 $License:
    Copyright (C) 2010 InvenSense Corporation, All Rights Reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
  $
 */

#include "mlos.h"
#include <linux/delay.h>
#include <linux/slab.h>

void *MLOSMalloc(unsigned int numBytes)
{
	return kmalloc(numBytes, GFP_KERNEL);
}

tMLError MLOSFree(void *ptr)
{
	kfree(ptr);
	return ML_SUCCESS;
}

tMLError MLOSCreateMutex(HANDLE *mutex)
{
	
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

tMLError MLOSLockMutex(HANDLE mutex)
{
	
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

tMLError MLOSUnlockMutex(HANDLE mutex)
{
	
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

tMLError MLOSDestroyMutex(HANDLE handle)
{
	
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}

FILE *MLOSFOpen(char *filename)
{
	
	return NULL;
}

void MLOSFClose(FILE *fp)
{
	
}

void MLOSSleep(int mSecs)
{
	msleep(mSecs);
}

unsigned long MLOSGetTickCount(void)
{
	
	return ML_ERROR_FEATURE_NOT_IMPLEMENTED;
}
