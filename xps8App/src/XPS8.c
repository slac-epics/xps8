#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dbDefs.h>
#include <dbAccess.h>
#include <recGbl.h>
#include <recSup.h>
#include <devSup.h>
#include <iocsh.h>

#include "epicsExport.h"

char asTemplatePath[256] = "";
char asTemplateName[ 80] = "";


/******************************************************************************/
void set_asTemplate_path( char *path )
{
    strcpy( asTemplatePath, path );
    if ( path[strlen(path)-1] != '/' ) strcat( asTemplatePath, "/" );
}

/******************************************************************************/
void set_asTemplate_name( char *name )
{
    strcpy( asTemplateName, name );
}

/* ioc-shell command registration *********************************************/
#define IOCSH_ARG     static const iocshArg
#define IOCSH_ARGs    static const iocshArg * const
#define IOCSH_FUNCDEF static const iocshFuncDef

IOCSH_ARG     set_asTemplate_path_Arg0    = { "path", iocshArgString    };
IOCSH_ARGs    set_asTemplate_path_Args[1] = { &set_asTemplate_path_Arg0 };
IOCSH_FUNCDEF set_asTemplate_path_FuncDef = { "set_asTemplate_path", 1,
                                              set_asTemplate_path_Args  };
static void  set_asTemplate_path_CallFunc( const iocshArgBuf *args )
{
    set_asTemplate_path( args[0].sval );
}

IOCSH_ARG     set_asTemplate_name_Arg0    = { "name", iocshArgString    };
IOCSH_ARGs    set_asTemplate_name_Args[1] = { &set_asTemplate_name_Arg0 };
IOCSH_FUNCDEF set_asTemplate_name_FuncDef = { "set_asTemplate_name", 1,
                                              set_asTemplate_name_Args  };
static void  set_asTemplate_name_CallFunc( const iocshArgBuf *args )
{
    set_asTemplate_name( args[0].sval );
}

void xps8_asRegister( void )
{
    iocshRegister( &set_asTemplate_path_FuncDef, set_asTemplate_path_CallFunc );
    iocshRegister( &set_asTemplate_name_FuncDef, set_asTemplate_name_CallFunc );
}

epicsExportRegistrar( xps8_asRegister );

