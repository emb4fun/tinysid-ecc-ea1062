/**************************************************************************
*  This file is part of the TAL project (Tiny Abstraction Layer)
*
*  Copyright (c) 2013 by Michael Fischer (www.emb4fun.de).
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without 
*  modification, are permitted provided that the following conditions 
*  are met:
*  
*  1. Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer.
*
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the 
*     documentation and/or other materials provided with the distribution.
*
*  3. Neither the name of the author nor the names of its contributors may 
*     be used to endorse or promote products derived from this software 
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL 
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED 
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF 
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF 
*  SUCH DAMAGE.
*
***************************************************************************
*  History:
*
*  28.07.2013  mifi  First Version.
**************************************************************************/
#if !defined(__TALCPU_CM_H__)
#define __TALCPU_CM_H__

/**************************************************************************
*  Includes
**************************************************************************/

/**************************************************************************
*  Global Definitions
**************************************************************************/

/**************************************************************************
*  Macro Definitions
**************************************************************************/

#define TAL_CPU_IRQ_ENTER()   __disable_irq()  
#define TAL_CPU_IRQ_EXIT()    __enable_irq()


/*
 * Disable and enable interrupt macros.
 */
 
__attribute__( ( always_inline ) ) static inline uint32_t _DisableAllInts (void)
{
   uint32_t dMask;

   __asm__ volatile ("MRS %0, primask" : "=r" (dMask) );
   __asm__ volatile ("cpsid i" : : : "memory");
  
   return(dMask);
} /* _DisableAllInts */
  
__attribute__( ( always_inline ) ) static inline void _EnableAllInts (uint32_t dMask)
{
   __set_PRIMASK(dMask);
} /* _EnableAllInts */
  
#define TAL_CPU_DISABLE_ALL_INTS() { uint32_t _dIntMask = _DisableAllInts();
#define TAL_CPU_ENABLE_ALL_INTS()    _EnableAllInts(_dIntMask); }

/**************************************************************************
*  Functions Definitions
**************************************************************************/

#endif /* !__TALCPU_CM_H__ */

/*** EOF ***/
