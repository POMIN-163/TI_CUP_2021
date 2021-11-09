/*
 * Copyright (c) 2012-2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * */
/*
 * ======== listlib.h ========
 *
 * 	Contains structures and exported function that are used by the linked
 * 	list library.
 *
 */

#ifndef __LISTLIB_H__
#define __LISTLIB_H__

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************
 * STRUCTURE -  NDK_LIST_NODE
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the 
 *	previous and next element in the list.
 **************************************************************************/
typedef struct NDK_LIST_NODE
{
	void*	p_next;		/* Pointer to the next element in the list. 	*/	
    void*   p_prev;     /* Pointer to the prev element in the list. */
}NDK_LIST_NODE;

/************************ EXTERN Functions *********************************/

extern void list_add (NDK_LIST_NODE **ptr_list, NDK_LIST_NODE *ptr_node);
extern NDK_LIST_NODE* list_remove (NDK_LIST_NODE **ptr_list);
extern NDK_LIST_NODE* list_get_head (NDK_LIST_NODE **ptr_list);
extern NDK_LIST_NODE* list_get_next (NDK_LIST_NODE *ptr_list);
extern int list_remove_node (NDK_LIST_NODE **ptr_list, NDK_LIST_NODE *ptr_remove);
extern void list_cat (NDK_LIST_NODE **ptr_dst, NDK_LIST_NODE **ptr_src);
extern NDK_LIST_NODE* list_replicate (NDK_LIST_NODE* ptr_srcList, int size, void* alloc(uint32_t size), void free (void *ptr));
extern void list_clean (NDK_LIST_NODE* ptr_list, void free (void *ptr));

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif	/* __LISTLIB_H__ */



