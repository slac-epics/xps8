/* xps8Record.c */
/* Record support module */

#include <algorithm>

#include <math.h>
#include <time.h>

#include <stddef.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "alarm.h"
#include "recGbl.h"
#include "dbScan.h"
#include "dbDefs.h"
#include "dbAccess.h"
#include "dbEvent.h"
#include "dbStaticLib.h"
#include "devSup.h"
#include "errMdef.h"
#include "recSup.h"
#include "special.h"

#define GEN_SIZE_OFFSET
#include "xps8Record.h"
#undef  GEN_SIZE_OFFSET

#include "epicsExport.h"

#include "XPS8_drivers.h"
#include "XPS8.h"


/* Create RSET - Record Support Entry Table */
#define report             NULL
#define initialize         NULL

static long init_record( dbCommon *precord, int pass  );
static long process    ( dbCommon *precord            );
static long special    ( dbAddr   *pDbAddr, int after );
static long cvt_dbaddr ( dbAddr   *pDbAddr            );

#define get_value          NULL
#define get_array_info     NULL
#define put_array_info     NULL
#define get_units          NULL
#define get_precision      NULL
#define get_enum_str       NULL
#define get_enum_strs      NULL
#define put_enum_str       NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double   NULL

rset xps8RSET = {
	RSETNUMBER,
	report,
	initialize,
	(RECSUPFUN) init_record,
	(RECSUPFUN) process,
	(RECSUPFUN) special,
	get_value,
	(RECSUPFUN) cvt_dbaddr,
	get_array_info,
	put_array_info,
	get_units,
	get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double
};

epicsExportAddress( rset, xps8RSET );


XPS8 *XPS8_Ctrl = NULL;

int xps8Debug = 0;

extern "C" { epicsExportAddress( int, xps8Debug ); }


static long init_ctrl  ( xps8Record *prec                                 );
static long log_msg    ( xps8Record *prec, int dlvl, const char *fmt, ... );
static void post_fields( xps8Record *prec, unsigned short all             );
static void post_msgs  ( xps8Record *prec                                 );


using namespace std;



static long init_record( dbCommon *precord, int pass )
{
    xps8Record *prec = (xps8Record *)precord;

    long        status = 0;

    if ( pass == 1 ) return( status );

    prec->vers = VERSION;

    prec->serr = (char *)calloc( 80, sizeof(char) );
    prec->fver = (char *)calloc( 80, sizeof(char) );
    prec->sstr = (char *)calloc( 80, sizeof(char) );

    status = init_ctrl( prec );

    prec->udf = 0;

    post_fields( prec, 1 );
    post_msgs  ( prec    );

    return( status );
}

static long process( dbCommon *precord )
{
    xps8Record  *prec = (xps8Record *)precord;
    struct XPS8 *ctrl;

    int          old_snum;
    short        old_amap;

    long         status = -1;

    if ( prec->pact ) return( 0 );

    prec->pact = TRUE;

    recGblGetTimeStamp( prec );

    ctrl = (struct XPS8 *)prec->dpvt;

    old_snum = prec->snum;
    old_amap = prec->amap;

    if ( ctrl->socket < 0 )                  // try to connect to the controller
    {
        ctrl->socket = TCP_ConnectToServer(prec->ctrl, prec->port, TCP_TIMEOUT);
        strncpy( prec->serr, TCP_GetError(ctrl->socket), 80 );

        if ( ctrl->socket < 0 )
        {
            log_msg( prec, 0, "Failed to connect to the controller" );
            prec->snum = -1;
            prec->amap =  0;

            MARK( M_SNUM );

            goto finish_up;
        }
    }

    status = ControllerStatusGet( ctrl->socket, &prec->snum );
    prec->snum &= 3;

    if ( status != 0 )
    {
        log_msg( prec, 0, "Failed to read the controller status" );
        prec->snum = -1;
        prec->amap =  0;

        MARK( M_SNUM );

        // close all sockets
        ctrl->uMutex->lock();

        CloseAllOtherSockets( ctrl->socket );
        TCP_CloseSocket     ( ctrl->socket );

        ctrl->socket    = -1;
        ctrl->connected =  0;
        ctrl->update    =  0;
        for ( int pi = 0; pi < 8; pi++ )
        {
            ctrl->positioner[pi].usocket = -1;
            ctrl->positioner[pi].msocket = -1;
        }

        ctrl->uMutex->unlock();

        prec->stup = 1 - prec->stup;
        db_post_events( prec, &prec->stup, DBE_VAL_LOG );
    }
    else if ( (ctrl->connected == 0) || (prec->snum != old_snum) )
    {
        status = ControllerStatusStringGet(ctrl->socket, prec->snum,prec->sstr);
        if ( status != 0 )
        {
            log_msg( prec, 0, "Failed to read the controller status string" );
            prec->snum = -1;
        }
        else if ( ctrl->connected == 0 )
        {
            log_msg( prec, 0, "Controller is up, please (auto-config,) re-connect" );
            prec->snum = -1;
        }

        MARK( M_SNUM );
    }

    finish_up:
    if ( prec->snum != old_snum ) MARK( M_SNUM );
    if ( prec->amap != old_amap ) MARK( M_AMAP );

    if ( (status != 0) || (prec->snum != 0) )
        recGblSetSevr( (dbCommon *) prec, STATE_ALARM, MAJOR_ALARM );

    post_fields( prec, 0 );
    post_msgs  ( prec    );

    recGblFwdLink( prec );                      // process the forward scan link

    prec->proc = 0;
    prec->pact = FALSE;

    return( status );
}

static long special( dbAddr *pDbAddr, int after )
{
    xps8Record  *prec = (xps8Record *)pDbAddr->precord;
    struct XPS8 *ctrl;

    int          fieldIndex = dbGetFieldIndex( pDbAddr );
    long         status = 0;

    if ( after != TRUE ) return( status );

    if ( fieldIndex == xps8RecordRBUT )
    {
        log_msg( prec, 0, "Reboot the controller ..." );

        ctrl   = (struct XPS8 *)prec->dpvt;
        status = Reboot( ctrl->socket );

        ctrl->socket    = -1;
        ctrl->connected =  0;
        ctrl->update    =  0;
        for ( int pi = 0; pi < 8; pi++ )
        {
            ctrl->positioner[pi].usocket = -1;
            ctrl->positioner[pi].msocket = -1;
        }

        prec->snum = -1;
        prec->amap =  0;

        prec->rbut =  0;
        prec->stup = 1 - prec->stup;
        db_post_events( prec, &prec->stup, DBE_VAL_LOG );

        recGblSetSevr( (dbCommon *)prec, COMM_ALARM, INVALID_ALARM );

        post_fields( prec, 1 );
    }
    else if ( fieldIndex == xps8RecordRCON )
    {
        ctrl = (struct XPS8 *)prec->dpvt;

        if ( ctrl->socket >= 0 )
        {
            ctrl->uMutex->lock();
            status = init_ctrl( prec );
            ctrl->uMutex->unlock();

            prec->ocon = 1 - prec->ocon;
            db_post_events( prec, &prec->ocon, DBE_VAL_LOG );

            post_fields( prec, 1 );
        }

        prec->rcon = 0;
    }
    else if ( fieldIndex == xps8RecordKALL )
    {
        log_msg( prec, 0, "Kill-All, please re-init, then home / reference" );

        ctrl   = (struct XPS8 *)prec->dpvt;
        status = KillAll( ctrl->socket );

        prec->kall = 0;
        prec->stup = 1 - prec->stup;
        db_post_events( prec, &prec->stup, DBE_VAL_LOG );
    }
    else if ( fieldIndex == xps8RecordIALL )
    {
        log_msg( prec, 0, "Initialize all positioners" );

        prec->iall = 0;
        prec->oini = 1 - prec->oini;
        db_post_events( prec, &prec->oini, DBE_VAL_LOG );
    }
    else if ( fieldIndex == xps8RecordRALL )
    {
        log_msg( prec, 0, "Reference all positioners" );

        prec->rall = 0;
        prec->oref = 1 - prec->oref;
        db_post_events( prec, &prec->oref, DBE_VAL_LOG );
    }
    else if ( fieldIndex == xps8RecordSDFT )
    {
        log_msg( prec, 0, "Use stage limits and max speeds" );

        prec->sdft = 0;
        prec->osdf = 1 - prec->osdf;
        db_post_events( prec, &prec->osdf, DBE_VAL_LOG );
    }

    post_msgs( prec );

    return( status );
}

static long cvt_dbaddr( dbAddr *pDbAddr )
{
    xps8Record *prec = (xps8Record *)pDbAddr->precord;
    int         fieldIndex = dbGetFieldIndex( pDbAddr );

    long        status = 0;

    switch ( fieldIndex )
    {
        case xps8RecordSERR:
        {
            pDbAddr->pfield         = (char *)prec->serr;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordFVER:
        {
            pDbAddr->pfield         = (char *)prec->fver;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordSSTR:
        {
            pDbAddr->pfield         = (char *)prec->sstr;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGA:
        {
            pDbAddr->pfield         = (char *)prec->loga;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGB:
        {
            pDbAddr->pfield         = (char *)prec->logb;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGC:
        {
            pDbAddr->pfield         = (char *)prec->logc;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGD:
        {
            pDbAddr->pfield         = (char *)prec->logd;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGE:
        {
            pDbAddr->pfield         = (char *)prec->loge;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGF:
        {
            pDbAddr->pfield         = (char *)prec->logf;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGG:
        {
            pDbAddr->pfield         = (char *)prec->logg;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8RecordLOGH:
        {
            pDbAddr->pfield         = (char *)prec->logh;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
    }

    return( status );
}

static long init_ctrl( xps8Record *prec )
{
    long  rtnval, status = 0;

    if ( ! XPS8_Ctrl )
    {
        XPS8_Ctrl = (struct XPS8 *) malloc( sizeof(struct XPS8) );

        XPS8_Ctrl->uMutex    = new epicsMutex();
        XPS8_Ctrl->socket    = -1;

        XPS8_Ctrl->lMutex    = new epicsMutex();
        XPS8_Ctrl->nMessages = 8;
        XPS8_Ctrl->mLength   = 61;
        XPS8_Ctrl->cIndex    = 0;
        XPS8_Ctrl->sAddr     = (char *)calloc( 8*61, sizeof(char) );

        prec->loga           = XPS8_Ctrl->sAddr;
        prec->logb           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 1;
        prec->logc           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 2;
        prec->logd           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 3;
        prec->loge           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 4;
        prec->logf           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 5;
        prec->logg           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 6;
        prec->logh           = XPS8_Ctrl->sAddr + XPS8_Ctrl->mLength * 7;

        for ( int pi = 0; pi < 8; pi++ )
        {
            strcpy( XPS8_Ctrl->positioner[pi].gname, "" );
            strcpy( XPS8_Ctrl->positioner[pi].pname, "" );
            strcpy( XPS8_Ctrl->positioner[pi].stage, "" );
        }

        prec->dpvt = XPS8_Ctrl;
    }

    for ( int pi = 0; pi < 8; pi++ )
    {
        XPS8_Ctrl->positioner[pi].usocket = -1;
        XPS8_Ctrl->positioner[pi].msocket = -1;
    }

    log_msg( prec, 0, "Connect to the controller, initialize ..." );

    strcpy( prec->fver, "" );

    prec->snum = -1;
    prec->amap =  0;

    if ( XPS8_Ctrl->socket >= 0 )
    {
        CloseAllOtherSockets( XPS8_Ctrl->socket );
        // TCP_CloseSocket  ( XPS8_Ctrl->socket );
    }
    else
    {
        XPS8_Ctrl->socket = TCP_ConnectToServer( prec->ctrl, prec->port,
                                                 TCP_TIMEOUT );
        strncpy( prec->serr, TCP_GetError(XPS8_Ctrl->socket), 80 );

        if ( XPS8_Ctrl->socket < 0 )
        {
            log_msg( prec, 0, "Failed to connect to the controller" );
            status = -9;

            goto finished;
        }
    }

    TCP_SetTimeout( XPS8_Ctrl->socket, TCP_TIMEOUT );

    rtnval = Login( XPS8_Ctrl->socket, "Administrator", "Administrator" );

    rtnval = FirmwareVersionGet ( XPS8_Ctrl->socket,  prec->fver );
    if ( rtnval != 0 )
    {
        log_msg( prec, 0, "Failed to read the firmware version" );
        status = -1;
    }

    rtnval = ControllerStatusGet( XPS8_Ctrl->socket, &prec->snum );
    prec->snum &= 3;

    if ( rtnval != 0 )
    {
        log_msg( prec, 0, "Failed to read the controller status" );
        status = -1;
    }
    else
    {
        char sstr[80];

        rtnval = ControllerStatusStringGet(XPS8_Ctrl->socket, prec->snum, sstr);
        if ( rtnval != 0 )
        {
            log_msg( prec, 0, "Failed to read the controller status string" );
            status = -1;
        }
        else if ( prec->snum != 0 )
        {
            strncpy( prec->sstr, sstr, 80 );
            status = -1;
        }
        else if ( status == 0 )
        {
            int  psocket;
            char gname[80], pname[80], par[80]="SmartStageName", pstr[80];

            strncpy( prec->sstr, sstr, 80 );

            // find the active channels
            psocket = TCP_ConnectToServer( prec->ctrl, prec->port, TCP_TIMEOUT);
            strncpy( prec->serr, TCP_GetError(psocket), 80 );

            if ( psocket >= 0 )
            {
                TCP_SetTimeout( psocket, TCP_TIMEOUT );

                for ( int pi = 0; pi < 8; pi++ )
                {
                    if ( strncmp( prec->fver, "XPS-C8", 6 ) == 0 )
                    {
                        sprintf( gname, "GROUP%d",            pi+1 );
                        sprintf( pname, "GROUP%d.POSITIONER", pi+1 );
                    }
                    else
                    {
                        sprintf( gname, "Group%d",            pi+1 );
                        sprintf( pname, "Group%d.Pos",        pi+1 );
                    }
                    rtnval = PositionerStageParameterGet( psocket, pname,
                                                          par, pstr );
                    if ( rtnval == 0 )
                    {
                        log_msg( prec, 0, "Found positioner #%d, type %s", pi+1, pstr );
                        XPS8_Ctrl->positioner[pi].usocket = psocket;
                        strncpy( XPS8_Ctrl->positioner[pi].gname, gname, 80 );
                        strncpy( XPS8_Ctrl->positioner[pi].pname, pname, 80 );
                        strncpy( XPS8_Ctrl->positioner[pi].stage, pstr,  80 );

                        psocket = TCP_ConnectToServer( prec->ctrl, prec->port,
                                                       TCP_TIMEOUT );
                        strncpy( prec->serr, TCP_GetError(psocket), 80 );

                        if ( psocket >= 0 )
                        {
                            TCP_SetTimeout( psocket, 0.0 );
                            XPS8_Ctrl->positioner[pi].msocket = psocket;
                        }
                        else
                        {
                            log_msg( prec, 0, "Failed to open socket for positioners" );
                            status = -9;
                            break;
                        }

                        prec->amap += ( 1 << pi );

                        // open a new socket
                        if ( pi < 7 )
                        {
                            psocket = TCP_ConnectToServer( prec->ctrl,
                                                           prec->port,
                                                           TCP_TIMEOUT );
                            strncpy( prec->serr, TCP_GetError(psocket), 80 );

                            if ( psocket >= 0 )
                                TCP_SetTimeout( psocket, TCP_TIMEOUT );
                            else
                            {
                                log_msg( prec, 0, "Failed to open socket for positioners" );
                                status = -9;
                                break;
                            }
                        }
                    }
                    else if ( pi == 7 ) TCP_CloseSocket( psocket );

                    epicsThreadSleep( 0.1 );
                }
            }
            else
            {
                log_msg( prec, 0, "Failed to open socket for positioners" );
                status = -9;
            }
        }
    }

    finished:
    XPS8_Ctrl->update = 0;
    if ( status == -9 )
        XPS8_Ctrl->connected = 0;
    else
    {
        XPS8_Ctrl->connected = 1;
        if ( prec->amap > 0 ) XPS8_Ctrl->update = 1;
    }

    if ( status == -9 )
        recGblSetSevr( (dbCommon *)prec, COMM_ALARM,  INVALID_ALARM );
    else if ( (status == -1) || (prec->snum != 0) )
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM   );

    return( status );
}

static void post_fields( xps8Record *prec, unsigned short all )
{
    unsigned short  alarm_mask = recGblResetAlarms( prec ), field_mask;
    changed_fields  cmap;

    cmap.All = prec->cmap;

    if ( (field_mask = alarm_mask | (all                  ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->vers, field_mask );
        db_post_events( prec,  prec->desc, field_mask );
        db_post_events( prec,  prec->ctrl, field_mask );
        db_post_events( prec,  prec->fver, field_mask );
        db_post_events( prec, &prec->kall, field_mask );
        db_post_events( prec, &prec->rbut, field_mask );
        db_post_events( prec, &prec->rcon, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_SERR) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec,  prec->serr, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_SNUM) ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->snum, field_mask );
        db_post_events( prec,  prec->sstr, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_AMAP) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->amap, field_mask );

    UNMARK_ALL;

    return;
}

static void post_msgs( xps8Record *prec )
{
    XPS8_Ctrl->lMutex->lock();

    if ( XPS8_Ctrl->newMsg )
    {
        db_post_events( prec,  prec->loga, DBE_VAL_LOG );
        db_post_events( prec,  prec->logb, DBE_VAL_LOG );
        db_post_events( prec,  prec->logc, DBE_VAL_LOG );
        db_post_events( prec,  prec->logd, DBE_VAL_LOG );
        db_post_events( prec,  prec->loge, DBE_VAL_LOG );
        db_post_events( prec,  prec->logf, DBE_VAL_LOG );
        db_post_events( prec,  prec->logg, DBE_VAL_LOG );
        db_post_events( prec,  prec->logh, DBE_VAL_LOG );

        XPS8_Ctrl->newMsg = 0;
    }

    XPS8_Ctrl->lMutex->unlock();

    return;
}

static long log_msg( xps8Record *prec, int dlvl, const char *fmt, ... )
{
    timespec   ts;
    struct tm  timeinfo;
    char       timestamp[40], msec[4], msg[512];

    va_list    args;

    // if () return( 0 );

    clock_gettime( CLOCK_REALTIME, &ts );
    localtime_r( &ts.tv_sec, &timeinfo );

    strftime( timestamp, 40, "%m/%d %H:%M:%S", &timeinfo );
    sprintf ( msec, "%03d", int(ts.tv_nsec*1.e-6 + 0.5) );

    va_start( args, fmt      );
    vsprintf( msg, fmt, args );
    va_end  ( args           );

    XPS8_Ctrl->lMutex->lock();

    if ( XPS8_Ctrl->cIndex > 7 )
        memmove( XPS8_Ctrl->sAddr,
                 XPS8_Ctrl->sAddr+XPS8_Ctrl->mLength, XPS8_Ctrl->mLength*7 );

    snprintf( XPS8_Ctrl->sAddr+XPS8_Ctrl->mLength*min(XPS8_Ctrl->cIndex,7), 61,
              "%s %s", timestamp+6, msg );

    if ( XPS8_Ctrl->cIndex <= 7 ) XPS8_Ctrl->cIndex++;

    XPS8_Ctrl->newMsg = 1;

    XPS8_Ctrl->lMutex->unlock();

    if ( dlvl <= xps8Debug )
        printf( "%s.%s %s -- %s\n", timestamp, msec, prec->name, msg );

    return( 1 );
}

