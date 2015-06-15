#ifndef _ASM_MUTEX_H
#define _ASM_MUTEX_H


#define __mutex_slowpath_needs_to_unlock()	1
#include <asm-generic/mutex-xchg.h>
#endif
