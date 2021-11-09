/*
 * Copyright (c) 2012-2018, Texas Instruments Incorporated
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
 * ======== inetaddr.c ========
 *
 * Standard address conversion functions
 *
 */

#include <stdlib.h>
#include <ctype.h>
#include <netmain.h>
#include <stkmain.h>
#include <_stack.h>

/*
 * inet_addr()
 * Converts string to network format IP address
 */
uint32_t inet_addr( const char *str )
{
    struct in_addr result;

    if( inet_aton(str, &result) )
        return( result.s_addr );
    return( 0 );
}

/*
 * inet_aton()
 * Convert string to network address structure
 */
int inet_aton( const char *str, struct in_addr *addr)
{
    uint32_t Val[4];
    uint32_t Base;
    int    Sect;
    char   c;

    Sect = -1;
    while( *str )
    {
        /* New section */
        Sect++;

        /* Get the base for this number */
        Base = 10;
        if (*str == '0')
        {
            if( *(str+1) == 'x' || *(str+1) == 'X' )
            {
                Base = 16;
                str += 2;
            }
            else
            {
                Base = 8;
                str++;
            }
        }

        /* Now decode this number */
        Val[Sect] = 0;
        for(;;)
        {
            c = *str++;

            if( (c >= '0' && c <= '9') )
                Val[Sect] = (Val[Sect]*Base) + (c-'0');
            else if( Base == 16 && (c >= 'A' && c <= 'F') )
                Val[Sect] = (Val[Sect]*16) + (c-'A') + 10;
            else if( Base == 16 && (c >= 'a' && c <= 'f') )
                Val[Sect] = (Val[Sect]*16) + (c-'a') + 10;
            else if( c == '.' )
            {
                /* Validate value */
                if( Val[Sect] > 255 )
                    return(0);

                /* Once we have four sections, quit */
                /* We want to accept: "1.2.3.4.in-addr.arpa" */
                if( Sect == 3 )
                    goto done;

                /* Break this section */
                break;
            }
            else if( !c )
                goto done;
            else if( c != ' ' )
                return(0);
        }
    }

done:
    /* What we do changes based on the number of sections */
    switch( Sect )
    {
    default:
        return(0);
    case 0:
        addr->s_addr = Val[0];
        break;
    case 1:
        if( Val[1] > 0xffffff )
            return(0);
        addr->s_addr = Val[0]<<24;
        addr->s_addr += Val[1];
        break;
    case 2:
        if( Val[2] > 0xffff )
            return(0);
        addr->s_addr = Val[0]<<24;
        addr->s_addr += (Val[1]<<16);
        addr->s_addr += Val[2];
        break;
    case 3:
        if( Val[3] > 0xff )
            return(0);
        addr->s_addr = Val[0]<<24;
        addr->s_addr += (Val[1]<<16);
        addr->s_addr += (Val[2]<<8);
        addr->s_addr += Val[3];
        break;
    }

    addr->s_addr = NDK_htonl(addr->s_addr);
    return(1);
}
