/*
** stub main for testing Ficl
** $Id: main.c,v 1.2 2010/09/10 09:01:28 asau Exp $
*/
/*
** Copyright (c) 1997-2001 John Sadler (john_sadler@alum.mit.edu)
** All rights reserved.
**
** Get the latest Ficl release at http://ficl.sourceforge.net
**
** I am interested in hearing from anyone who uses Ficl. If you have
** a problem, a success story, a defect, an enhancement request, or
** if you would like to contribute to the Ficl release, please
** contact me by email at the address above.
**
** L I C E N S E  and  D I S C L A I M E R
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions
** are met:
** 1. Redistributions of source code must retain the above copyright
**    notice, this list of conditions and the following disclaimer.
** 2. Redistributions in binary form must reproduce the above copyright
**    notice, this list of conditions and the following disclaimer in the
**    documentation and/or other materials provided with the distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
** OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
** HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
** LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
** OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
** SUCH DAMAGE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <apps/readline.h>
#include "ficl.h"
#include "fcntl.h"


int ficl_main(int argc, char **argv)
{
    int returnValue = 0;
    char buffer[256];
    ficlVm *vm;
	ficlSystem *system;

    system = ficlSystemCreate(NULL);
    ficlSystemCompileExtras(system);
    vm = ficlSystemCreateVm(system);

    returnValue = ficlVmEvaluate(vm, ".ver .( " __DATE__ " ) cr quit");

    /*
    ** load files specified on command-line
    */
    if (argc  > 1)
    {
        sprintf(buffer, ".( loading %s ) cr load %s\n cr", argv[1], argv[1]);
        returnValue = ficlVmEvaluate(vm, buffer);
    }

    while (returnValue != FICL_VM_STATUS_USER_EXIT)
    {
        fputs(FICL_PROMPT,stdout);
	 fflush(stdout);
        //if (fgets(buffer, sizeof(buffer), stdin) == NULL) break;
        readline(buffer,sizeof(buffer),stdin,stdout);
        returnValue = ficlVmEvaluate(vm, buffer);
    }
    printf("Exiting Ficl environment\n");
    ficlSystemDestroy(system);
    return 0;

}

