/* xps8pRecord.c */
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
#include <unistd.h>

#include <callback.h>

#include "alarm.h"
#include "dbAccess.h"
#include "recGbl.h"
#include "dbScan.h"
#include "dbEvent.h"
#include "dbDefs.h"
#include "dbAccess.h"
#include "devSup.h"
#include "errMdef.h"
#include "recSup.h"
#include "special.h"

#include "dbStaticLib.h"

#define GEN_SIZE_OFFSET
#include "xps8pRecord.h"
#undef  GEN_SIZE_OFFSET

#include "epicsExport.h"

#include "XPS8_drivers.h"
#include "XPS8.h"


/* Create RSET - Record Support Entry Table */
#define report             NULL
#define initialize         NULL

static long init_record  ( dbCommon *precord, int pass        );
static long process      ( dbCommon *precord                  );
static long special      ( dbAddr   *pDbAddr, int after       );
static long cvt_dbaddr   ( dbAddr   *pDbAddr                  );

static long get_precision( dbAddr   *pDbAddr, long *precision );

#define get_value          NULL
#define get_array_info     NULL
#define put_array_info     NULL
#define get_units          NULL
#define get_enum_str       NULL
#define get_enum_strs      NULL
#define put_enum_str       NULL
#define get_graphic_double NULL
#define get_control_double NULL
#define get_alarm_double   NULL

rset xps8pRSET = {
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
	(RECSUPFUN) get_precision,
	get_enum_str,
	get_enum_strs,
	put_enum_str,
	get_graphic_double,
	get_control_double,
	get_alarm_double
};

epicsExportAddress( rset, xps8pRSET );


extern XPS8 *XPS8_Ctrl;

struct positioner_info
{
    CALLBACK             callback;
    struct xps8pRecord *precord;
    epicsMutex          *uMutex;
    epicsEvent          *uEvent;
    int                  usocket;
    int                  msocket;
    int                  poll;

    timespec             uTime;
    int                  update;
    int                  status;

    int                  dstatus;
    char                 dstr[80];
    int                  hstatus;
    char                 hstr[80];
    int                  error;
    char                 estr[800];

    int                  state;
    char                 sstr[80];
    double               drbv;
    double               velo;

    epicsMutex          *lMutex;
    int                  nMessages;
    int                  mLength;
    char                *sAddr;
    int                  cIndex;
    bool                 newMsg;
};

static long init_positioner      ( xps8pRecord *prec );
static long update_misc          ( xps8pRecord *prec );
static void check_software_limits( xps8pRecord *prec );

static long set_parameter        ( struct positioner_info *pinfo,
                                   char *pName, char *par, char *pstr );
static void positioner_callback  ( struct positioner_info *pinfo );
static void checkStatus          ( struct positioner_info *pinfo );

static long log_msg    ( xps8pRecord *prec, int dlvl, const char *fmt, ... );
static void post_fields( xps8pRecord *prec, unsigned short alarm_mask,
                                            unsigned short all             );
static void post_msgs  ( xps8pRecord *prec                                 );

#define MIP_DONE     0x0000    // No motion is in progress
#define MIP_MOVE     0x0001    // A move not resulting from Jog* or Hom*
#define MIP_RETRY    0x0002    // A retry is in progress
#define MIP_NEW      0x0004    // Stop current move for a new move
#define MIP_HOMF     0x0010    // A home-forward command is in progress
#define MIP_HOMR     0x0020    // A home-reverse command is in progress
#define MIP_HOME     (MIP_HOMF | MIP_HOMR)
#define MIP_JOGF     0x0100    // Jog forward
#define MIP_JOGR     0x0200    // Jog backward
#define MIP_JOG      (MIP_JOGF | MIP_JOGR)
#define MIP_PAUSE    0x1000    // Move is paused
#define MIP_STOP     0x2000    // We're trying to stop.  If a home command
                               // is issued when the motor is moving, we
                               // stop the motor first
#define MIP_KILL     0x4000    // A kill command is in progress
#define MIP_INIT     0x8000    // A initialize command is in progress

extern int xps8Debug;

using namespace std;



static long init_record( dbCommon *precord, int pass )
{
    xps8pRecord     *prec = (xps8pRecord *)precord;
    positioner_info *pinfo;

    long             status = 0;

    if ( pass == 0 ) return( status );

    prec->dstr = (char *)calloc( 80, sizeof(char) );
    prec->hstr = (char *)calloc( 80, sizeof(char) );
    prec->sstr = (char *)calloc( 80, sizeof(char) );
    prec->estr = (char *)calloc( 80, sizeof(char) );

    pinfo = (struct positioner_info *)malloc( sizeof(struct positioner_info) );
    pinfo->precord       = prec;
    pinfo->uMutex        = new epicsMutex();
    pinfo->uEvent        = new epicsEvent( epicsEventEmpty );
    pinfo->poll          = 0;
    pinfo->uTime.tv_sec  = 0;
    pinfo->uTime.tv_nsec = 0;
    pinfo->update        = 0;

    pinfo->lMutex        = new epicsMutex();
    pinfo->nMessages     = 8;
    pinfo->mLength       = 61;
    pinfo->cIndex        = 0;
    pinfo->sAddr         = (char *)calloc( 8*61, sizeof(char) );

    prec->loga           = pinfo->sAddr;
    prec->logb           = pinfo->sAddr + pinfo->mLength * 1;
    prec->logc           = pinfo->sAddr + pinfo->mLength * 2;
    prec->logd           = pinfo->sAddr + pinfo->mLength * 3;
    prec->loge           = pinfo->sAddr + pinfo->mLength * 4;
    prec->logf           = pinfo->sAddr + pinfo->mLength * 5;
    prec->logg           = pinfo->sAddr + pinfo->mLength * 6;
    prec->logh           = pinfo->sAddr + pinfo->mLength * 7;

    prec->dpvt           = pinfo;

#ifdef _BSD_SOURCE || _XOPEN_SOURCE >= 500
    gethostname( prec->host, 60            );
#else
    strcpy     ( prec->host, getenv("IOC") );
#endif
    strcpy     ( prec->iocn, getenv("IOC") );

    callbackSetCallback( (void (*)(struct callbackPvt *)) positioner_callback,
                         &(pinfo->callback) );
    callbackSetPriority( priorityMedium, &(pinfo->callback) );

    epicsThreadCreate  ( prec->name, epicsThreadPriorityMedium,
                         epicsThreadGetStackSize(epicsThreadStackMedium),
                         (EPICSTHREADFUNC)checkStatus, (void *)pinfo );

    recGblResetAlarms( prec );

    status = init_positioner( prec );

    return( status );
}

static long process( dbCommon *precord )
{
    xps8pRecord     *prec  = (xps8pRecord *)precord;
    positioner_info *pinfo = (positioner_info *)prec->dpvt;

    int              old_movn, old_dmov, old_mip, old_rcnt, old_butc;
    double           old_val,  old_dval, old_diff;
    unsigned int     old_msta;
    unsigned short   alarm_mask;
    msta_field       msta;

    long             old_err,  status = 0;

    if ( prec->pact ) return( 0 );

    prec->pact = TRUE;

    recGblGetTimeStamp( prec );

    old_msta = prec->msta;
    old_butc = prec->butc;

    if ( (pinfo->update  == 0) ||             // invoked by external action (pp)
         (pinfo->usocket <  0) || (pinfo->msocket <  0) )
    {
        msta.All = prec->msta;
        goto finished;
    }

    // Called back from checkStatus
    pinfo->uMutex->lock();

    if ( pinfo->state != prec->snum )                        // update the state
    {
        prec->snum = pinfo->state;
        strncpy( prec->sstr, pinfo->sstr, 80 );

        MARK( M_SNUM );
    }

    if ( (strncmp(prec->sstr, "Not ", 4) != 0) && (pinfo->drbv != prec->drbv) )
    {                                     // first time or when DRBV has changed
        prec->drbv = pinfo->drbv;
        prec->rbv  = prec->drbv * (1. - 2.*prec->dir) + prec->off;

        MARK( M_DRBV );
        MARK( M_RBV  );
    }

    if ( pinfo->update != 9 )                    // still moving, partial update
    {
        if ( (prec->mip & MIP_JOG) && (prec->mip & MIP_STOP) )
        {
            if ( fabs(pinfo->velo) < 1.e-9 )
            {
                char gName[80];

                strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );
                status = GroupJogModeDisable( pinfo->msocket, gName );
            }
        }

        pinfo->update = 0;
        pinfo->uMutex->unlock();
        msta.All = prec->msta;

        goto finished;
    }

    old_mip  = prec->mip ;
    old_movn = prec->movn;
    old_dmov = prec->dmov;
    old_val  = prec->val ;
    old_dval = prec->dval;
    old_diff = prec->diff;
    old_rcnt = prec->rcnt;
    old_err  = prec->err ;

    msta.All = 0;

    // Moving stopped, full update
    if ( pinfo->error != prec->err )                         // update the error
    {
        prec->err  = pinfo->error;
        strncpy( prec->estr, pinfo->estr, 80 );
    }

    if ( pinfo->dstatus != prec->dsta )              // update the driver status
    {
        prec->dsta = pinfo->dstatus;
        strncpy( prec->dstr, pinfo->dstr, 80 );

        MARK( M_DSTA );
    }

    if ( prec->dsta != 0 ) msta.Bits.RA_PROBLEM = 1;         // hardward problem

    if ( pinfo->hstatus != prec->hsta )            // update the hardware status
    {
        prec->hsta = pinfo->hstatus;
        strncpy( prec->hstr, pinfo->hstr, 80 );

        MARK( M_HSTA );
    }

    if ( (prec->hsta & 0x00000100) > 0 )           // Minus end of run activated
    {
        if ( prec->dir == xps8pDIR_Pos ) msta.Bits.RA_MINUS_LS = 1;
        else                             msta.Bits.RA_PLUS_LS  = 1;
    }

    if ( (prec->hsta & 0x00000200) > 0 )            // Plus end of run activated
    {
        if ( prec->dir == xps8pDIR_Pos ) msta.Bits.RA_PLUS_LS  = 1;
        else                             msta.Bits.RA_MINUS_LS = 1;
    }

    if ( old_dmov == 1 ) pinfo->poll = 0;

    pinfo->update = 0;
    pinfo->uMutex->unlock();

    // If we run into any trouble, stop, no more action
    if ( (prec->snum < 10) || ((prec->snum > 19) && (prec->snum != 70)
                                                 && (prec->snum != 77)) )
    {
        msta.Bits.RA_PROBLEM = 1;                            // hardward problem

        prec->movn = 0;
        prec->dmov = 1;
        prec->mip  = MIP_DONE;
    }

    if ( prec->mip == MIP_DONE ) goto finish_up;

    if ( (prec->mip == MIP_HOME) ||
         (prec->mip  & MIP_STOP) || (prec->mip  & MIP_PAUSE) )
    {                                             // are we homing or stopping ?
        if ( prec->mip == MIP_HOME )                    // set the at-home bit ?
        {
        }

        prec->movn = 0;
        prec->dmov = 1;
        prec->diff = 0;
        if ( (prec->mip == MIP_HOME) || (prec->mip  & MIP_STOP) )
        {
            prec->mip  = MIP_DONE;
            prec->val  = prec->rbv;
            prec->dval = prec->drbv;
        }
    }
    else if ( !(prec->mip & MIP_NEW) || (prec->snum != 10) )
    {                                                  // was moving or retrying
        prec->diff = prec->rbv - prec->val;
        if ( (fabs(prec->diff) >= prec->rdbd) &&
             (prec->rtry > 0) && (prec->rcnt < prec->rtry) )            // retry
        {
            char gName[80], pName[80];

            prec->mip |= MIP_RETRY;
            prec->rcnt++;

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );
            strncpy( pName, XPS8_Ctrl->positioner[prec->card].pname, 80 );

            pinfo->uMutex->lock();
            status = PositionerSGammaParametersSet( pinfo->msocket, pName,
                                                    prec->velo, prec->accl,
                                                    prec->minj, prec->maxj );
            status = GroupMoveAbsolute            ( pinfo->msocket, gName,
                                                    1, &prec->dval );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            epicsThreadSleep( 0.1 );

            pinfo->uEvent->signal();

            log_msg( prec, 0, "Desired %.6g, reached %.6g, retrying %d", prec->val, prec->rbv, prec->rcnt );
        }
        else                                // close enough, or no retry allowed
        {
            prec->movn = 0;
            prec->dmov = 1;
            prec->mip  = MIP_DONE;

            if ( fabs(prec->diff) < prec->rdbd )
                log_msg( prec, 0, "Desired %.6g, reached %.6g",                  prec->val, prec->rbv );
            else
                log_msg( prec, 0, "Desired %.6g, reached %.6g after %d retries", prec->val, prec->rbv, prec->rcnt );
        }
    }

    finish_up:
    if ( prec->mip == MIP_DONE )
    {
        prec->diff = prec->rbv - prec->val;
        if ( fabs(prec->diff) > prec->pdbd )
        {
            msta.Bits.EA_SLIP_STALL = 1;
            if ( prec->err == 0 )
            {
                prec->err = 0x88888888;
                strcpy( prec->estr, "Slip/Stall detected" );
            }
        }

        if ( prec->diff != old_diff ) MARK( M_DIFF );
    }

    if ( prec->mip  != old_mip  ) MARK( M_MIP  );
    if ( prec->movn != old_movn ) MARK( M_MOVN );
    if ( prec->dmov != old_dmov ) MARK( M_DMOV );
    if ( prec->val  != old_val  ) MARK( M_VAL  );
    if ( prec->dval != old_dval ) MARK( M_DVAL );
    if ( prec->rcnt != old_rcnt ) MARK( M_RCNT );
    if ( prec->err  != old_err  ) MARK( M_ERR  );

    finished:
    msta.Bits.RA_DONE = prec->dmov;

    prec->msta = msta.All;
    if ( prec->msta != old_msta ) MARK( M_MSTA );

    if      ( pinfo->usocket < 0                              ) prec->butc =-1;
    else if ( strncmp(prec->sstr, "Not initialized", 15) == 0 ) prec->butc = 1;
    else if ( strncmp(prec->sstr, "Not referenced",  14) == 0 ) prec->butc = 2;
    else if ( (strncmp(prec->sstr, "Ready ", 6) == 0) ||
              (strncmp(prec->sstr, "Moving", 6) == 0)         ) prec->butc = 3;
    else                                                        prec->butc = 0;
    
    if ( prec->butc != old_butc ) MARK( M_BUTC );

    // check the alarms
    if      ( pinfo->usocket < 0                            )
        recGblSetSevr( (dbCommon *)prec, UDF_ALARM,   INVALID_ALARM );
    else if ( msta.Bits.RA_PROBLEM  ||
              msta.Bits.RA_MINUS_LS || msta.Bits.RA_PLUS_LS )
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM   );
    else if ( msta.Bits.EA_SLIP_STALL                       )
        recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MINOR_ALARM   );

    recGblFwdLink( prec );                      // process the forward scan link

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 0 );
    post_msgs  ( prec                );

    prec->proc = 0;
    prec->pact = FALSE;

    return( status );
}

static long special( dbAddr *pDbAddr, int after )
{
    xps8pRecord     *prec  = (xps8pRecord *)pDbAddr->precord;
    positioner_info *pinfo = (positioner_info *)prec->dpvt;
    int              fieldIndex = dbGetFieldIndex( pDbAddr );
    char             gName[80], pName[80], par[80], pstr[80];
    double           nval, hllm, hhlm, dval, velo;
    unsigned short   alarm_mask = 0;

    long             status = 0;

    if ( after != TRUE )
    {
        if      ( fieldIndex == xps8pRecordVAL  ) prec->oval = prec->val;
        else if ( fieldIndex == xps8pRecordDVAL ) prec->oval = prec->dval;
        else if ( fieldIndex == xps8pRecordLLM  ) prec->oval = prec->llm;
        else if ( fieldIndex == xps8pRecordHLM  ) prec->oval = prec->hlm;
        else if ( fieldIndex == xps8pRecordDLLM ) prec->oval = prec->dllm;
        else if ( fieldIndex == xps8pRecordDHLM ) prec->oval = prec->dhlm;
        else if ( fieldIndex == xps8pRecordMINJ ) prec->oval = prec->minj;
        else if ( fieldIndex == xps8pRecordMAXJ ) prec->oval = prec->maxj;
        else if ( fieldIndex == xps8pRecordVELO ) prec->oval = prec->velo;
        else if ( fieldIndex == xps8pRecordACCL ) prec->oval = prec->accl;
        else if ( fieldIndex == xps8pRecordHVEL ) prec->oval = prec->hvel;
        else if ( fieldIndex == xps8pRecordHACC ) prec->oval = prec->hacc;
        else if ( fieldIndex == xps8pRecordDIR  ) prec->oval = prec->dir;
        else if ( fieldIndex == xps8pRecordOFF  ) prec->oval = prec->off;
        else if ( fieldIndex == xps8pRecordSET  ) prec->oval = prec->set;
        else if ( fieldIndex == xps8pRecordBL   ) prec->oval = prec->bl;
        else if ( fieldIndex == xps8pRecordSPG  ) prec->oval = prec->spg;

        return( status );
    }

    strncpy( pName, XPS8_Ctrl->positioner[prec->card].pname, 80 );
    switch ( fieldIndex )
    {
        case ( xps8pRecordSTUP ):
            if ( XPS8_Ctrl->positioner[prec->card].usocket >= 0 )
            {
                status = update_misc( prec );
                pinfo->uEvent->signal();
            }
            else if ( pinfo->usocket >= 0 )
            {
                pinfo->usocket = -1;
                pinfo->msocket = -1;

                strcpy( prec->sstr, ""                         );
                strcpy( prec->dstr, ""                         );
                strcpy( prec->hstr, ""                         );
                strcpy( prec->type, ""                         );
                strcpy( prec->estr, "Positioner not available" );
                prec->err  = 999;

                prec->butc =  -1;
                prec->udf  =   1;
                recGblSetSevr( (dbCommon *)prec, UDF_ALARM, INVALID_ALARM );

                alarm_mask = recGblResetAlarms( prec );
            }

            break;
        case ( xps8pRecordVAL  ):
            if ( (strncmp(prec->sstr, "Ready ", 6) != 0) &&
                 (strncmp(prec->sstr, "Moving", 6) != 0)    )
            {
                prec->err  = 999;
                sprintf( prec->estr, "Can only move in a \"Ready/Moving\" state" );

                prec->val  = prec->oval;

                MARK( M_ERR  );
                MARK( M_VAL  );

                break;
            }

            if ( (prec->val < prec->llm) || (prec->val > prec->hlm) )
            {                                    // violated the software limits
                prec->err  = 999;
                if ( prec->val < prec->llm )
                    sprintf(prec->estr,"Violated software low limit, no move" );
                else
                    sprintf(prec->estr,"Violated software high limit, no move");

                prec->val  = prec->oval;
                prec->lvio = 1;                     // set limit violation alarm

                MARK( M_ERR  );
                MARK( M_VAL  );
                MARK( M_LVIO );

                break;
            }

            do_move1:
            if ( prec->set == xps8pSET_Use )            // do it only when "Use"
            {
                prec->dval = (prec->val - prec->off) * (1. - 2.*prec->dir);
                MARK( M_DVAL );
            }
            goto do_move2;
        case ( xps8pRecordDVAL ):
            if ( (strncmp(prec->sstr, "Ready ", 6) != 0) &&
                 (strncmp(prec->sstr, "Moving", 6) != 0)    )
            {
                prec->err  = 999;
                sprintf( prec->estr, "Can only move in a \"Ready/Moving\" state" );

                prec->dval = prec->oval;

                MARK( M_ERR  );
                MARK( M_DVAL );

                break;
            }

            if ( (prec->dval < prec->dllm) || (prec->dval > prec->dhlm) )
            {                                    // violated the hardware limits
                prec->err  = 999;
                if ( prec->dval < prec->dllm )
                    sprintf( prec->estr, "Violated dial low limit, no move"  );
                else
                    sprintf( prec->estr, "Violated dial high limit, no move" );

                prec->dval = prec->oval;
                prec->lvio = 1;                     // set limit violation alarm

                MARK( M_ERR  );
                MARK( M_DVAL );
                MARK( M_LVIO );

                break;
            }

            prec->val = prec->dval * (1. - 2.*prec->dir) + prec->off;
            MARK( M_VAL  );

            do_move2:
            if ( (prec->set != xps8pSET_Use) ||               // do it only when
                 (prec->spg != xps8pSPG_Go )    ) break;      // "Set" and "Go"

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            if ( prec->dmov == 0 )                    // stop current move first
            {
                log_msg( prec, 0, "Stop current move" );

                pinfo->uMutex->lock();
                status = GroupMoveAbort( pinfo->msocket, gName );
                pinfo->uMutex->unlock();

                prec->mip  = MIP_NEW;

                epicsThreadSleep( 0.3 );
            }

            log_msg( prec, 0, "Move to: %.6g (DVAL: %.6g)", prec->val, prec->dval );

            pinfo->uMutex->lock();
            status = PositionerSGammaParametersSet( pinfo->msocket, pName,
                                                    prec->velo, prec->accl,
                                                    prec->minj, prec->maxj );
            status = GroupMoveAbsolute            ( pinfo->msocket, gName,
                                                    1, &prec->dval );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->lvio = 0;
            prec->mip &= ~MIP_PAUSE;
            prec->mip |=  MIP_MOVE;
            prec->movn = 1;
            prec->dmov = 0;
            prec->rcnt = 0;

            MARK( M_LVIO );
            MARK( M_MIP  );
            MARK( M_MOVN );
            MARK( M_DMOV );
            MARK( M_RCNT );

            epicsThreadSleep( 0.1 );

            pinfo->uEvent->signal();

            break;
        case ( xps8pRecordTWF  ):
            nval = prec->val + prec->twv;
            goto tweak;
        case ( xps8pRecordTWR  ):
            nval = prec->val - prec->twv;

            tweak:
            if ( (strncmp(prec->sstr, "Ready ", 6) != 0) &&
                 (strncmp(prec->sstr, "Moving", 6) != 0)    )
            {
                prec->err  = 999;
                sprintf( prec->estr, "Can only move in a \"Ready/Moving\" state" );

                MARK( M_ERR  );

                break;
            }

            if ( (nval < prec->llm) || (nval > prec->hlm) )
            {                                    // violated the software limits
                prec->err  = 999;
                if ( nval < prec->llm )
                    sprintf(prec->estr,"Violated software low limit, no move" );
                else
                    sprintf(prec->estr,"Violated software high limit, no move");

                prec->lvio = 1;                     // set limit violation alarm

                MARK( M_ERR  );
                MARK( M_LVIO );

                break;
            }

            prec->val = nval;
            MARK( M_VAL  );
            goto do_move1;
        case ( xps8pRecordJOGF ):
        case ( xps8pRecordJOGR ):
            if ( (prec->jogf == 0) && (prec->jogr == 0) )
            {
                if ( prec->mip & MIP_JOG ) goto do_stop;         // stop jogging

                break;
            }
            else if ( (prec->set != xps8pSET_Use)             ||// jog only when
                      (prec->spg != xps8pSPG_Go )             ||// "Set", "Go" &
                      (strncmp(prec->sstr, "Ready ", 6) != 0)    )    // "Ready"
            {
                prec->jogf =   0;
                prec->jogr =   0;

                prec->err  = 999;
                if      ( prec->set != xps8pSET_Use )
                    sprintf( prec->estr, "Can only jog in \"Use\" mode"      );
                else if ( prec->spg != xps8pSPG_Go  )
                    sprintf( prec->estr, "Can only jog when \"Go\""          );
                else
                    sprintf( prec->estr, "Can only jog in a \"Ready\" state" );

                MARK( M_ERR  );

                break;
            }

            if ( prec->jogf > 0 )
            {
                if ( prec->dir == xps8pDIR_Pos ) velo =   prec->velo;
                else                             velo = - prec->velo;

                prec->mip  = MIP_JOGF;
                log_msg( prec, 0, "Jogging forward ..." );
            }
            else
            {
                if ( prec->dir == xps8pDIR_Pos ) velo = - prec->velo;
                else                             velo =   prec->velo;

                prec->mip  = MIP_JOGR;
                log_msg( prec, 0, "Jogging backward ..." );
            }

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            pinfo->uMutex->lock();

            status = GroupJogModeEnable   ( pinfo->msocket, gName );

            epicsThreadSleep( 0.2 );

            status = GroupJogParametersSet( pinfo->msocket, gName, 1,
                                            &velo, &prec->accl );

            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->lvio = 0;
            prec->movn = 1;
            prec->dmov = 0;
            prec->rcnt = 0;

            MARK( M_LVIO );
            MARK( M_MIP  );
            MARK( M_MOVN );
            MARK( M_DMOV );
            MARK( M_RCNT );

            epicsThreadSleep( 0.1 );

            pinfo->uEvent->signal();

            break;
        case ( xps8pRecordSPG  ):
            if ( (prec->spg == prec->oval            ) ||
                 (prec->mip&(MIP_MOVE | MIP_JOG) == 0)    ) break;

            if ( prec->spg == xps8pSPG_Go )
            {
                if      ( prec->mip &  MIP_MOVE ) goto do_move2;
                else if ( prec->mip &  MIP_JOG  )
                {
                    if ( prec->jogf > 0 )
                    {
                        if ( prec->dir == xps8pDIR_Pos ) velo =   prec->velo;
                        else                             velo = - prec->velo;

                        log_msg( prec, 0, "Jogging forward ..." );
                    }
                    else
                    {
                        if ( prec->dir == xps8pDIR_Pos ) velo = - prec->velo;
                        else                             velo =   prec->velo;

                        log_msg( prec, 0, "Jogging backward ..." );
                    }

                    prec->mip  &= ~MIP_PAUSE;

                    strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname,80);

                    pinfo->uMutex->lock();
                    status = GroupJogParametersSet( pinfo->msocket, gName, 1,
                                                    &velo, &prec->accl );
                    pinfo->uMutex->unlock();
                }
            }
            else
            {
                strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

                pinfo->uMutex->lock();

                if ( (prec->spg == xps8pSPG_Stop) || (prec->mip & MIP_MOVE) )
                    status = GroupMoveAbort       ( pinfo->msocket, gName );
                else
                {
                    velo   = 0;
                    status = GroupJogParametersSet( pinfo->msocket, gName, 1,
                                                    &velo, &prec->accl );
                }

                pinfo->poll = 1;
                pinfo->uMutex->unlock();

                if ( prec->spg == xps8pSPG_Stop )
                {
                    prec->mip |= MIP_STOP;
                    log_msg( prec, 0, "Stopping ..." );
                }
                else
                {
                    prec->mip |= MIP_PAUSE;
                    log_msg( prec, 0, "Pausing ..."  );
                }
            }

            MARK( M_MIP  );

            epicsThreadSleep( 0.1 );

            pinfo->uEvent->signal();

            break;
        case ( xps8pRecordSTOP ):
            prec->stop = 0;

            do_stop:
            if ( prec->mip & (MIP_MOVE | MIP_JOG) == 0 ) break;

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            pinfo->uMutex->lock();
            status = GroupMoveAbort( pinfo->msocket, gName );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->mip |= MIP_STOP;
            log_msg( prec, 0, "Stopping ..." );

            MARK( M_MIP  );

            epicsThreadSleep( 0.1 );

            pinfo->uEvent->signal();

            break;
        case ( xps8pRecordKILL ):
            if ( prec->spg != xps8pSPG_Go ) break; // do it when "Go" is choosed

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            pinfo->uMutex->lock();
            status = GroupKill( pinfo->msocket, gName );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->mip  = MIP_KILL;

            log_msg( prec, 0, "Kill" );

            goto kill_init_home;
        case ( xps8pRecordINIT ):
            if ( strncmp(prec->sstr, "Not initialized", 15) != 0 ) break;

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            pinfo->uMutex->lock();
            status = GroupInitialize( pinfo->msocket, gName );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->mip  = MIP_INIT;

            log_msg( prec, 0, "Initialize" );

            goto kill_init_home;
        case ( xps8pRecordREFR ):
        case ( xps8pRecordREF2 ):
            if ( strncmp(prec->sstr, "Not referenced",  14) != 0 ) break;

            if ( fieldIndex == xps8pRecordREFR ) nval = prec->drbv;
            else                                 nval = prec->ref2;

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            pinfo->uMutex->lock();
            status = GroupReferencingStart        ( pinfo->msocket, gName );
            status = GroupReferencingActionExecute( pinfo->msocket, pName,
                                                    "SetPosition", "None",
                                                    nval );
            status = GroupReferencingStop         ( pinfo->msocket, gName );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->mip  = MIP_HOME;

            log_msg( prec, 0, "Reference to %f", nval );

            goto kill_init_home;
        case ( xps8pRecordHOME ):
            if ( (strncmp(prec->sstr, "Not referenced",  14) != 0) ||
                 (prec->spg != xps8pSPG_Go                       )    ) break;

            strncpy( gName, XPS8_Ctrl->positioner[prec->card].gname, 80 );

            pinfo->uMutex->lock();
            status = PositionerSGammaParametersSet( pinfo->msocket, pName,
                                                    prec->hvel, prec->hacc,
                                                    prec->minj, prec->maxj );
            status = GroupHomeSearch              ( pinfo->msocket, gName );
            pinfo->poll = 1;
            pinfo->uMutex->unlock();

            prec->mip  = MIP_HOME;

            log_msg( prec, 0, "Homing ..." );

            kill_init_home:
            prec->lvio = 0;
            prec->movn = 1;
            prec->dmov = 0;
            prec->rcnt = 0;

            MARK( M_LVIO );
            MARK( M_MIP  );
            MARK( M_MOVN );
            MARK( M_DMOV );
            MARK( M_RCNT );

            post_msgs( prec );

            epicsThreadSleep( 0.1 );

            pinfo->uEvent->signal();

            break;
        case ( xps8pRecordDLLM ):
            if ( (prec->dllm < prec->sllm) || (prec->dllm > prec->dhlm) )
            {
                prec->err  = 999;
                if ( prec->dllm < prec->sllm )
                    sprintf( prec->estr, "Violated stage low limit, no change");
                else
                    sprintf( prec->estr, "Violated dial high limit, no change");

                MARK( M_ERR  );

                prec->dllm = prec->oval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );

                break;
            }

            nval   = prec->dllm * (1. - 2.*prec->dir) + prec->off;

            if ( prec->dir == xps8pDIR_Pos )
            {
                prec->llm = nval;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            }
            else
            {
                prec->hlm = nval;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            }

            check_limit_violation:
            status = PositionerUserTravelLimitsSet( pinfo->usocket, pName,
                                                    prec->dllm, prec->dhlm );

            if ( (prec->val < prec->llm) || (prec->val > prec->hlm) )
            {
                prec->lvio = 1;                     // set limit violation alarm
                MARK( M_LVIO );

                prec->err  = 999;
                if ( prec->val < prec->llm )
                    sprintf( prec->estr, "Violated software low limit"  );
                else
                    sprintf( prec->estr, "Violated software high limit" );

                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordDHLM ):
            if ( (prec->dhlm > prec->shlm) || (prec->dhlm < prec->dllm) )
            {
                prec->err  = 999;
                if ( prec->dhlm > prec->shlm )
                    sprintf( prec->estr,"Violated stage high limit, no change");
                else
                    sprintf( prec->estr,"Violated dial low limit, no change"  );

                MARK( M_ERR  );

                prec->dhlm = prec->oval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );

                break;
            }

            nval   = prec->dhlm * (1. - 2.*prec->dir) + prec->off;

            if ( prec->dir == xps8pDIR_Pos )
            {
                prec->hlm = nval;
                db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            }
            else
            {
                prec->llm = nval;
                db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            }

            goto check_limit_violation;
        case ( xps8pRecordLLM  ):
            nval = (prec->llm - prec->off) * (1. - 2.*prec->dir);

            if ( prec->dir == xps8pDIR_Pos )
            {
                if ( (nval < prec->sllm) || (nval > prec->dhlm) )
                {
                    prec->err  = 999;
                    if ( nval < prec->sllm )
                        sprintf( prec->estr, "Violated stage low limit, no change" );
                    else
                        sprintf( prec->estr, "Violated dial high limit, no change" );

                    MARK( M_ERR  );

                    prec->llm  = prec->oval;
                    db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

                    break;
                }

                prec->dllm = nval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
            }
            else
            {
                if ( (nval > prec->shlm) || (nval < prec->dllm) )
                {
                    prec->err  = 999;
                    if ( nval > prec->shlm )
                        sprintf( prec->estr, "Violated stage high limit, no change" );
                    else
                        sprintf( prec->estr, "Violated dial low limit, no change"   );

                    MARK( M_ERR  );

                    prec->llm  = prec->oval;
                    db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

                    break;
                }

                prec->dhlm = nval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
            }

            goto check_limit_violation;
        case ( xps8pRecordHLM  ):
            nval = (prec->hlm - prec->off) * (1. - 2.*prec->dir);

            if ( prec->dir == xps8pDIR_Pos )
            {
                if ( (nval > prec->shlm) || (nval < prec->dllm) )
                {
                    prec->err  = 999;
                    if ( nval > prec->shlm )
                        sprintf( prec->estr, "Violated stage high limit, no change" );
                    else
                        sprintf( prec->estr, "Violated dial low limit, no change"   );

                    MARK( M_ERR  );

                    prec->hlm  = prec->oval;
                    db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

                    break;
                }

                prec->dhlm = nval;
                db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
            }
            else
            {
                if ( (nval < prec->sllm) || (nval > prec->dhlm) )
                {
                    prec->err  = 999;
                    if ( nval < prec->sllm )
                        sprintf( prec->estr, "Violated stage low limit, no change" );
                    else
                        sprintf( prec->estr, "Violated dial high limit, no change" );

                    MARK( M_ERR  );

                    prec->hlm  = prec->oval;
                    db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

                    break;
                }

                prec->dllm = nval;
                db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
            }

            goto check_limit_violation;
        case ( xps8pRecordMINJ ):
            if ( (prec->minj < prec->slj ) || (prec->minj > prec->maxj) )
            {
                prec->minj = prec->oval;
                db_post_events( prec, &prec->minj, DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Out of range, no change" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordMAXJ ):
            if ( (prec->maxj > prec->shj ) || (prec->maxj < prec->minj) )
            {
                prec->maxj = prec->oval;
                db_post_events( prec, &prec->maxj, DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Out of range, no change" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordVELO ):
            if ( (prec->velo <= 0        ) || (prec->velo > prec->svel) )
            {
                prec->velo = prec->oval;
                db_post_events( prec, &prec->velo, DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Out of range, no change" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordACCL ):
            if ( (prec->accl <= 0        ) || (prec->accl > prec->sacc) )
            {
                prec->accl = prec->oval;
                db_post_events( prec, &prec->accl, DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Out of range, no change" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordHVEL ):
            if ( (prec->hvel <= 0        ) || (prec->hvel > prec->shve) )
            {
                prec->hvel = prec->oval;
                db_post_events( prec, &prec->hvel, DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Out of range, no change" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordHACC ):
            if ( (prec->hacc <= 0        ) || (prec->hacc > prec->shac) )
            {
                prec->hacc = prec->oval;
                db_post_events( prec, &prec->hacc, DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Out of range, no change" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordDIR  ):
            if ( prec->dir == prec->oval ) break;

            if ( prec->dir == xps8pDIR_Pos )
            {
                hllm      = prec->off - prec->hlm;
                hhlm      = prec->off - prec->llm;
                prec->llm = prec->off + hllm;
                prec->hlm = prec->off + hhlm;
            }
            else
            {
                hllm      = prec->llm - prec->off;
                hhlm      = prec->hlm - prec->off;
                prec->llm = prec->off - hhlm;
                prec->hlm = prec->off - hllm;
            }

            dval       = (prec->val - prec->off ) * (2.*prec->oval - 1.);

            goto change_dir_off;
        case ( xps8pRecordOFF  ):
            prec->llm += prec->off - prec->oval;
            prec->hlm += prec->off - prec->oval;

            dval       = (prec->val - prec->oval) * (1. - 2.*prec->dir);

            change_dir_off:
            prec->val  =       dval * (1. - 2.*prec->dir) + prec->off;
            prec->rbv  = prec->drbv * (1. - 2.*prec->dir) + prec->off;

            db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );
            MARK( M_VAL  );
            MARK( M_RBV  );

            check_software_limits( prec );

            break;
        case ( xps8pRecordSET  ):
            if ( prec->set == prec->oval ) break;

            if ( strncmp(prec->sstr, "Ready ", 6) == 0 )
            {                    // can only Use/Set position in a "Ready state"
                if ( prec->set == xps8pSET_Use )
                {
                    prec->off = prec->val - prec->dval * (1. - 2.*prec->dir);
                    prec->rbv = prec->drbv * (1. - 2.*prec->dir) + prec->off;
                    db_post_events( prec, &prec->off,  DBE_VAL_LOG );
                    db_post_events( prec, &prec->rbv,  DBE_VAL_LOG );

                    check_software_limits( prec );
                }
            }
            else                      // action not allowed, restore old setting
            {
                prec->set	= static_cast<epicsEnum16>(prec->oval);
                db_post_events( prec, &prec->set,  DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Can only Use/Set position in a \"Ready state\"" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordBL   ):
            if ( strncmp(prec->sstr, "Not initialized", 15) == 0 )
            {       // can only dis/enable backlash in a "Not initialized state"
                pinfo->uMutex->lock();

                if ( prec->bl == xps8pBL_Enable )
                    status = PositionerBacklashEnable ( pinfo->msocket, pName );
                else
                    status = PositionerBacklashDisable( pinfo->msocket, pName );

                pinfo->uMutex->unlock();
            }
            else                      // action not allowed, restore old setting
            {
                prec->bl	= static_cast<epicsEnum16>(prec->oval);
                db_post_events( prec, &prec->bl,   DBE_VAL_LOG );

                prec->err  = 999;
                sprintf( prec->estr, "Can only en/disable backlash in a \"Not initialized state\"" );
                MARK( M_ERR  );
            }

            break;
        case ( xps8pRecordBDST ):
            pinfo->uMutex->lock();
            status = PositionerBacklashSet( pinfo->msocket, pName, prec->bdst );
            pinfo->uMutex->unlock();

            break;
        case ( xps8pRecordHSET ):
            sprintf( par,  "HomePreset"             );
            sprintf( pstr, "%f",         prec->hset );

            status = set_parameter( pinfo, pName, par, pstr );

            break;
        case ( xps8pRecordHTYP ):                                  // needs work
            dbMenu* pMenu;

            pMenu = dbFindMenu( pdbbase, "xps8pHTYP" );

            sprintf( par,  "HomeSearchSequenceType" );
            status = set_parameter( pinfo, pName, par,
                                    pMenu->papChoiceValue[prec->htyp] );

            break;
        case ( xps8pRecordSDFT ):
            log_msg( prec, 0, "Use stage limits and max speeds" );

            prec->dllm = prec->sllm;
            prec->dhlm = prec->shlm;

            if ( prec->dir == xps8pDIR_Pos )
            {
                prec->llm  = prec->dllm * (1. - 2.*prec->dir) + prec->off;
                prec->hlm  = prec->dhlm * (1. - 2.*prec->dir) + prec->off;
            }
            else
            {
                prec->llm  = prec->dhlm * (1. - 2.*prec->dir) + prec->off;
                prec->hlm  = prec->dllm * (1. - 2.*prec->dir) + prec->off;
            }

            db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
            db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
            db_post_events( prec, &prec->llm,  DBE_VAL_LOG );
            db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

            prec->velo = prec->svel;
            prec->accl = prec->sacc;

            prec->hvel = prec->shve;
            prec->hacc = prec->shac;

            prec->minj = prec->slj;
            prec->maxj = prec->shj;

            db_post_events( prec, &prec->velo, DBE_VAL_LOG );
            db_post_events( prec, &prec->accl, DBE_VAL_LOG );
            db_post_events( prec, &prec->hvel, DBE_VAL_LOG );
            db_post_events( prec, &prec->hacc, DBE_VAL_LOG );
            db_post_events( prec, &prec->minj, DBE_VAL_LOG );
            db_post_events( prec, &prec->maxj, DBE_VAL_LOG );

            prec->sdft = 0;

            goto check_limit_violation;
        case ( xps8pRecordRCON ):
            status = init_positioner( prec );
            break;
    }

    switch ( fieldIndex )
    {
        case ( xps8pRecordHSET ):
        case ( xps8pRecordHTYP ):
            prec->err  = 999;
            sprintf( prec->estr, "Please reboot the controller for the change to take effect" );
            MARK( M_ERR  );

            break;
        default:
            break;
    }

    post_fields( prec, alarm_mask, 0 );
    post_msgs  ( prec                );

    return( status );
}

static long cvt_dbaddr( dbAddr *pDbAddr )
{
    xps8pRecord *prec = (xps8pRecord *)pDbAddr->precord;
    int          fieldIndex = dbGetFieldIndex( pDbAddr );

    long         status = 0;

    switch ( fieldIndex )
    {
        case xps8pRecordDSTR:
        {
            pDbAddr->pfield         = (char *)prec->dstr;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordHSTR:
        {
            pDbAddr->pfield         = (char *)prec->hstr;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordSSTR:
        {
            pDbAddr->pfield         = (char *)prec->sstr;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordESTR:
        {
            pDbAddr->pfield         = (char *)prec->estr;
            pDbAddr->no_elements    = 80;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGA:
        {
            pDbAddr->pfield         = (char *)prec->loga;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGB:
        {
            pDbAddr->pfield         = (char *)prec->logb;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGC:
        {
            pDbAddr->pfield         = (char *)prec->logc;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGD:
        {
            pDbAddr->pfield         = (char *)prec->logd;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGE:
        {
            pDbAddr->pfield         = (char *)prec->loge;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGF:
        {
            pDbAddr->pfield         = (char *)prec->logf;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGG:
        {
            pDbAddr->pfield         = (char *)prec->logg;
            pDbAddr->no_elements    = XPS8_Ctrl->mLength;
            pDbAddr->field_type     = DBF_CHAR;
            pDbAddr->field_size     = sizeof(char);
            pDbAddr->dbr_field_type = DBR_CHAR;
            break;
        }
        case xps8pRecordLOGH:
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

static long get_precision( dbAddr *pDbAddr, long *precision )
{
    int          fieldIndex = dbGetFieldIndex( pDbAddr );

    switch ( fieldIndex )
    {
//      case xps8pRecordVERS:
//          *precision = 2;
//          break;
        default:
            recGblGetPrec( pDbAddr, precision );
            break;
    }

    return ( 0 );
}

static long init_positioner( xps8pRecord *prec )
{
    positioner_info *pinfo = (positioner_info *)prec->dpvt;
    unsigned short   alarm_mask;
    char             pName[80];
    bool             defaults;

    long             status = 0;

    if ( (XPS8_Ctrl->connected == 1) && (XPS8_Ctrl->update == 1) )
        prec->type[0] = '\0';

    pinfo->usocket = XPS8_Ctrl->positioner[prec->card].usocket;
    pinfo->msocket = XPS8_Ctrl->positioner[prec->card].msocket;
    if ( (pinfo->usocket >= 0) && (pinfo->msocket >= 0) )
    {
        log_msg( prec, 0, "Initialize positioner" );

        status  = update_misc( prec );

        strncpy( pName, XPS8_Ctrl->positioner[prec->card].pname, 80 );

        pinfo->uMutex->lock();

        status |= Login( pinfo->msocket, "Administrator", "Administrator" );

        defaults = (strcmp(prec->ptyp, prec->type) != 0) &&
                   (XPS8_Ctrl->defaults            == 1);

        // set the user limits and backlash etc
        if ( (prec->dllm < prec->sllm) || (defaults == 1) )
        {
            prec->dllm = prec->sllm;
            db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
        }

        if ( (prec->dhlm > prec->shlm) || (defaults == 1) )
        {
            prec->dhlm = prec->shlm;
            db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );
        }
        status |= PositionerUserTravelLimitsSet( pinfo->msocket, pName,
                                                 prec->dllm, prec->dhlm );

                  PositionerBacklashSet        ( pinfo->msocket, pName,
                                                 prec->bdst );

        pinfo->uMutex->unlock();

        if ( status != 0 )
        {
            recGblSetSevr( (dbCommon *)prec, STATE_ALARM, MAJOR_ALARM );
            recGblResetAlarms( prec );
        }

        check_software_limits( prec );

        if ( (prec->velo > prec->svel) || (defaults == 1) )
        {
            prec->velo = prec->svel;
            db_post_events( prec, &prec->velo, DBE_VAL_LOG );
        }

        if ( (prec->accl > prec->sacc) || (defaults == 1) )
        {
            prec->accl = prec->sacc;
            db_post_events( prec, &prec->accl, DBE_VAL_LOG );
        }

        if ( (prec->hvel > prec->shve) || (defaults == 1) )
        {
            prec->hvel = prec->shve;
            db_post_events( prec, &prec->hvel, DBE_VAL_LOG );
        }

        if ( (prec->hacc > prec->shac) || (defaults == 1) )
        {
            prec->hacc = prec->shac;
            db_post_events( prec, &prec->hacc, DBE_VAL_LOG );
        }

        if ( (prec->minj < prec->slj ) || (defaults == 1) )
        {
            prec->minj = prec->slj;
            db_post_events( prec, &prec->minj, DBE_VAL_LOG );
        }

        if ( (prec->maxj > prec->shj ) || (defaults == 1) )
        {
            prec->maxj = prec->shj;
            db_post_events( prec, &prec->maxj, DBE_VAL_LOG );
        }

        pinfo->uEvent->signal();

        prec->snum =  -1;
        prec->udf  =   0;
    }
    else
    {
        prec->err  = 999;
        sprintf( prec->estr, "Positioner not available" );

        recGblSetSevr( (dbCommon *)prec, UDF_ALARM, INVALID_ALARM );

        MARK( M_ERR  );

        prec->udf  =   1;
    }

    strcpy( prec->ptyp, prec->type );
    db_post_events( prec,  prec->type, DBE_VAL_LOG );
    db_post_events( prec,  prec->ptyp, DBE_VAL_LOG );

    alarm_mask = recGblResetAlarms( prec );
    post_fields( prec, alarm_mask, 1 );
    post_msgs  ( prec                );

    return( status );
}

static long update_misc( xps8pRecord *prec )
{
    positioner_info *pinfo = (positioner_info *)prec->dpvt;
    dbMenu*          pMenu;
    char             pName[80], par[80], pstr[80];
    int              ic;

    long             status = 0;

    if ( pinfo->usocket < 0 ) return( status );

    strncpy( pName,      XPS8_Ctrl->positioner[prec->card].pname, 60 );
    strncpy( prec->type, XPS8_Ctrl->positioner[prec->card].stage, 60 );

    pinfo->uMutex->lock();

    sprintf( par,  "Unit"                          );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    strncpy( prec->egu , pstr, 16 );
    db_post_events( prec,  prec->egu , DBE_VAL_LOG );

    // DHZ: when new stage database is deployed, put back the "|" for status
    sprintf( par,  "MinimumTargetPosition"         );
    status  = PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->sllm ) != 1 );
    db_post_events( prec, &prec->sllm, DBE_VAL_LOG );

    sprintf( par,  "MaximumTargetPosition"         );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->shlm ) != 1 );
    db_post_events( prec, &prec->shlm, DBE_VAL_LOG );

    sprintf( par,  "MaximumVelocity"               );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->svel ) != 1 );
    db_post_events( prec, &prec->svel, DBE_VAL_LOG );

    sprintf( par,  "MaximumAcceleration"           );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->sacc ) != 1 );
    db_post_events( prec, &prec->sacc, DBE_VAL_LOG );

    sprintf( par,  "HomeSearchMaximumVelocity"     );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->shve ) != 1 );
    db_post_events( prec, &prec->shve, DBE_VAL_LOG );

    sprintf( par,  "HomeSearchMaximumAcceleration" );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->shac ) != 1 );
    db_post_events( prec, &prec->shac, DBE_VAL_LOG );

    sprintf( par,  "MinimumJerkTime"               );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->slj  ) != 1 );
    db_post_events( prec, &prec->slj,  DBE_VAL_LOG );

    sprintf( par,  "MaximumJerkTime"               );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->shj  ) != 1 );
    db_post_events( prec, &prec->shj,  DBE_VAL_LOG );

//  status |= PositionerUserTravelLimitsGet( pinfo->usocket, pName,
//                                           &prec->dllm, &prec->dhlm );
//  db_post_events( prec, &prec->dllm, DBE_VAL_LOG );
//  db_post_events( prec, &prec->dhlm, DBE_VAL_LOG );

    status |= PositionerBacklashGet( pinfo->usocket, pName, &prec->bdst, pstr );
    if ( strcmp(pstr, "Enable") == 0 ) prec->bl = xps8pBL_Enable;
    else                               prec->bl = xps8pBL_Disable;

    db_post_events( prec, &prec->bdst, DBE_VAL_LOG );
    db_post_events( prec, &prec->bl,   DBE_VAL_LOG );

    sprintf( par,  "HomePreset"                    );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );
    status |= ( sscanf( pstr, "%lf", &prec->hset ) != 1 );
    db_post_events( prec, &prec->hset, DBE_VAL_LOG );

    sprintf( par,  "HomeSearchSequenceType"        );
    status |= PositionerStageParameterGet( pinfo->usocket, pName, par, pstr );

    ic    = 0;
    pMenu = dbFindMenu( pdbbase, "xps8pHTYP" );
    while ( ic < pMenu->nChoice )
    {
        if ( strcmp( pstr, pMenu->papChoiceValue[ic] ) == 0 ) break;
        ic++;
    }

    if ( ic < pMenu->nChoice )
    {
        prec->htyp = ic;
        db_post_events( prec, &prec->htyp, DBE_VAL_LOG );
    }
    else
    {
        prec->err  = 999;
        sprintf( prec->estr, "Unknown HTYP: %s", pstr );

        MARK( M_ERR  );
    }

    pinfo->uMutex->unlock();

    return( status );
}

static void check_software_limits( xps8pRecord *prec )
{
    double  uval;

    uval = prec->dllm * (1. - 2.*prec->dir) + prec->off;

    if      ( (prec->dir == xps8pDIR_Pos ) && (prec->llm < uval) )
    {
        prec->llm  = uval;
        db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

        prec->err  = 999;
        sprintf( prec->estr, "Violated dial low limit" );
        db_post_events( prec, &prec->err,  DBE_VAL_LOG );
        db_post_events( prec,  prec->estr, DBE_VAL_LOG );
    }
    else if ( (prec->dir == xps8pDIR_Neg ) && (prec->hlm > uval) )
    {
        prec->hlm  = uval;
        db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

        prec->err  = 999;
        sprintf( prec->estr, "Violated dial low limit" );
        db_post_events( prec, &prec->err,  DBE_VAL_LOG );
        db_post_events( prec,  prec->estr, DBE_VAL_LOG );
    }

    uval = prec->dhlm * (1. - 2.*prec->dir) + prec->off;

    if      ( (prec->dir == xps8pDIR_Pos) && (prec->hlm > uval) )
    {
        prec->hlm  = uval;
        db_post_events( prec, &prec->hlm,  DBE_VAL_LOG );

        prec->err  = 999;
        sprintf( prec->estr, "Violated dial high limit" );
        db_post_events( prec, &prec->err,  DBE_VAL_LOG );
        db_post_events( prec,  prec->estr, DBE_VAL_LOG );
    }
    else if ( (prec->dir == xps8pDIR_Neg) && (prec->llm < uval) )
    {
        prec->llm  = uval;
        db_post_events( prec, &prec->llm,  DBE_VAL_LOG );

        prec->err  = 999;
        sprintf( prec->estr, "Violated dial high limit" );
        db_post_events( prec, &prec->err,  DBE_VAL_LOG );
        db_post_events( prec,  prec->estr, DBE_VAL_LOG );
    }

    if      ( prec->val < prec->llm )
    {
        prec->lvio =   1;
        prec->err  = 999;
        sprintf( prec->estr, "Violated software low limit" );
        db_post_events( prec, &prec->lvio, DBE_VAL_LOG );
        db_post_events( prec, &prec->err,  DBE_VAL_LOG );
        db_post_events( prec,  prec->estr, DBE_VAL_LOG );
    }
    else if ( prec->val > prec->hlm )
    {
        prec->lvio =   1;
        prec->err  = 999;
        sprintf( prec->estr, "Violated software high limit" );
        db_post_events( prec, &prec->lvio, DBE_VAL_LOG );
        db_post_events( prec, &prec->err,  DBE_VAL_LOG );
        db_post_events( prec,  prec->estr, DBE_VAL_LOG );
    }

    return;
}

static long set_parameter( struct positioner_info *pinfo,
                           char *pName, char *par, char *pstr )
{
    long status = 0;

    pinfo->uMutex->lock();
    status = PositionerStageParameterSet( pinfo->msocket, pName, par, pstr );
    pinfo->uMutex->unlock();

    return( status );
}

static void positioner_callback( struct positioner_info *pinfo )
{
    scanOnce( (struct dbCommon *)pinfo->precord );
}

static void checkStatus( struct positioner_info *pinfo )
{
    xps8pRecord *prec = pinfo->precord;
    char        *gName, *pName;
    timespec     uTime;

    long         dsec, dnsec, status;
    double       dtime, accl;

    gName = XPS8_Ctrl->positioner[pinfo->precord->card].gname;
    pName = XPS8_Ctrl->positioner[pinfo->precord->card].pname;

    while ( 1 )
    {
        // wait pinfo->precord->uint when state may be changing
        // otherwise wait one day (86400)
        if ( pinfo->poll == 1 ) pinfo->uEvent->wait( pinfo->precord->uint );
        else                    pinfo->uEvent->wait( 86400                );

        if ( (XPS8_Ctrl->update < 1) ||
             (pinfo->usocket    < 0) || (pinfo->msocket < 0) ) continue;

        log_msg( prec, 1, "Checking status ..." );

        clock_gettime( CLOCK_REALTIME, &uTime );

        pinfo->uMutex->lock();

        status  = 0;
        status |= GroupStatusGet      ( pinfo->usocket, gName, &pinfo->state );
        status |= GroupStatusStringGet( pinfo->usocket, pinfo->state,
                                        pinfo->sstr  );

        // only check the full status when finished moving
        if ( (pinfo->state != 40) &&      // Emergency braking
             (pinfo->state != 41) &&      // Motor initialization
             (pinfo->state != 43) &&      // Homing
             (pinfo->state != 44) &&      // Moving
             (pinfo->state != 45) &&      // Trajectory state
             (pinfo->state != 46) &&      // Slave state
             (pinfo->state != 47) &&      // Jogging
             (pinfo->state != 48) &&      // Analog tracking
             (pinfo->state != 49) &&      // Encoder calibrating
             (pinfo->state != 51) &&      // Spinning
             (pinfo->state != 64) &&      // Referencing
             (pinfo->state != 68) &&      // Auto-tuning
             (pinfo->state != 69)    )    // Scaling calibration
        {
            status |= PositionerErrorRead              ( pinfo->usocket, pName,
                                                         &pinfo->error   );
            if ( pinfo->error == 0 )
                strcpy( pinfo->estr, "No error" );
            else
                status |= PositionerErrorStringGet     ( pinfo->usocket,
                                                         pinfo->error,
                                                          pinfo->estr    );
            status |= PositionerDriverStatusGet        ( pinfo->usocket, pName,
                                                         &pinfo->dstatus );
            if ( pinfo->dstatus == 0 )
                strcpy( pinfo->dstr, "Ok" );
            else
                status |= PositionerDriverStatusStringGet( pinfo->usocket,
                                                           pinfo->dstatus,
                                                            pinfo->dstr  );

            status |= PositionerHardwareStatusGet      ( pinfo->usocket, pName,
                                                         &pinfo->hstatus );
            status |= PositionerHardwareStatusStringGet( pinfo->usocket,
                                                         pinfo->hstatus,
                                                          pinfo->hstr    );

            log_msg( prec, 2, "Driver status: %d -- %s", pinfo->dstatus, pinfo->dstr );
            log_msg( prec, 2, "HW status: %d -- %s",     pinfo->hstatus, pinfo->hstr );

            pinfo->update = 9;
        }
        else
        {
            pinfo->update = 1;

            if ( (prec->mip & MIP_JOG) && (prec->mip & MIP_STOP) )
                status |= GroupJogCurrentGet( pinfo->usocket, gName, 1,
                                              &pinfo->velo, &accl );
        }

        log_msg( prec, 2, "State: %d -- %s", pinfo->state, pinfo->sstr );

        dsec  = uTime.tv_sec  - pinfo->uTime.tv_sec;
        dnsec = uTime.tv_nsec - pinfo->uTime.tv_nsec;
        dtime = dsec + dnsec*1.e-9;
        if ( (pinfo->update == 9) || (dtime >= 0.5) )
        {
            status |= GroupPositionCurrentGet          ( pinfo->usocket, gName,
                                                         1, &pinfo->drbv  );

            log_msg( prec, 2, "Raw position: %f", pinfo->drbv );

            pinfo->uTime.tv_sec  = uTime.tv_sec;
            pinfo->uTime.tv_nsec = uTime.tv_nsec;
        }

        pinfo->status = status;

        pinfo->uMutex->unlock();

        while ( ! interruptAccept ) epicsThreadSleep( 1 );
        callbackRequest( (CALLBACK *)pinfo );
    }

    return;
}

static void post_fields( xps8pRecord *prec, unsigned short alarm_mask,
                                            unsigned short all )
{
    unsigned short  field_mask;
    changed_fields  cmap;

    cmap.All = prec->cmap;

    if ( (field_mask = alarm_mask | (all                  ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec,  prec->type, field_mask );
        db_post_events( prec,  prec->desc, field_mask );
        db_post_events( prec,  prec->egu,  field_mask );
        db_post_events( prec, &prec->shlm, field_mask );
        db_post_events( prec, &prec->sllm, field_mask );
        db_post_events( prec, &prec->dhlm, field_mask );
        db_post_events( prec, &prec->dllm, field_mask );
        db_post_events( prec, &prec->hlm,  field_mask );
        db_post_events( prec, &prec->llm,  field_mask );
        db_post_events( prec, &prec->slj,  field_mask );
        db_post_events( prec, &prec->shj,  field_mask );
        db_post_events( prec, &prec->minj, field_mask );
        db_post_events( prec, &prec->maxj, field_mask );
        db_post_events( prec, &prec->svel, field_mask );
        db_post_events( prec, &prec->sacc, field_mask );
        db_post_events( prec, &prec->velo, field_mask );
        db_post_events( prec, &prec->accl, field_mask );
        db_post_events( prec, &prec->shve, field_mask );
        db_post_events( prec, &prec->shac, field_mask );
        db_post_events( prec, &prec->hvel, field_mask );
        db_post_events( prec, &prec->hacc, field_mask );
        db_post_events( prec, &prec->bl,   field_mask );
        db_post_events( prec, &prec->bdst, field_mask );
        db_post_events( prec, &prec->dir,  field_mask );
        db_post_events( prec, &prec->off,  field_mask );
        db_post_events( prec, &prec->set,  field_mask );
        db_post_events( prec, &prec->twv,  field_mask );
        db_post_events( prec, &prec->twf,  field_mask );
        db_post_events( prec, &prec->twr,  field_mask );
        db_post_events( prec, &prec->rtry, field_mask );
        db_post_events( prec, &prec->rdbd, field_mask );
        db_post_events( prec, &prec->pdbd, field_mask );
        db_post_events( prec, &prec->spg,  field_mask );
        db_post_events( prec, &prec->kill, field_mask );
        db_post_events( prec, &prec->init, field_mask );
        db_post_events( prec, &prec->home, field_mask );
        db_post_events( prec, &prec->refr, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_DSTA) ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->dsta, field_mask );
        db_post_events( prec,  prec->dstr, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_HSTA) ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->hsta, field_mask );
        db_post_events( prec,  prec->hstr, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_SNUM) ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->snum, field_mask );
        db_post_events( prec,  prec->sstr, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_ERR ) ? DBE_VAL_LOG : 0)) )
    {
        db_post_events( prec, &prec->err,  field_mask );
        db_post_events( prec,  prec->estr, field_mask );
    }

    if ( (field_mask = alarm_mask | (all | MARKED(M_VAL ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->val,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DVAL) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->dval, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DRBV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->drbv, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RBV ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rbv,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DIFF) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->diff, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MIP ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->mip,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MOVN) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->movn, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_DMOV) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->dmov, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_RCNT) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->rcnt, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MISS) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->miss, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_LVIO) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->lvio, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_HLS ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->hls,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_LLS ) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->lls,  field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_MSTA) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->msta, field_mask );

    if ( (field_mask = alarm_mask | (all | MARKED(M_BUTC) ? DBE_VAL_LOG : 0)) )
        db_post_events( prec, &prec->butc, field_mask );

    UNMARK_ALL;

    return;
}

static void post_msgs( xps8pRecord *prec )
{
    positioner_info *pinfo = (positioner_info *)prec->dpvt;

    pinfo->lMutex->lock();

    if ( pinfo->newMsg )
    {
        db_post_events( prec,  prec->loga, DBE_VAL_LOG );
        db_post_events( prec,  prec->logb, DBE_VAL_LOG );
        db_post_events( prec,  prec->logc, DBE_VAL_LOG );
        db_post_events( prec,  prec->logd, DBE_VAL_LOG );
        db_post_events( prec,  prec->loge, DBE_VAL_LOG );
        db_post_events( prec,  prec->logf, DBE_VAL_LOG );
        db_post_events( prec,  prec->logg, DBE_VAL_LOG );
        db_post_events( prec,  prec->logh, DBE_VAL_LOG );

        pinfo->newMsg = 0;
    }

    pinfo->lMutex->unlock();

    return;
}

static long log_msg( xps8pRecord *prec, int dlvl, const char *fmt, ... )
{
    positioner_info *pinfo = (positioner_info *)prec->dpvt;
    timespec         ts;
    struct tm        timeinfo;
    char             timestamp[40], msec[4], msg[512];

    va_list          args;

    if ( (dlvl > prec->dlvl) && (dlvl > xps8Debug) ) return( 0 );

    clock_gettime( CLOCK_REALTIME, &ts );
    localtime_r( &ts.tv_sec, &timeinfo );

    strftime( timestamp, 40, "%m/%d %H:%M:%S", &timeinfo );
    sprintf ( msec, "%03d", int(ts.tv_nsec*1.e-6 + 0.5) );

    va_start( args, fmt      );
    vsprintf( msg, fmt, args );
    va_end  ( args           );

    if ( dlvl <= prec->dlvl )
    {
        pinfo->lMutex->lock();

        if ( pinfo->cIndex > 7 )
            memmove( pinfo->sAddr,
                     pinfo->sAddr+pinfo->mLength, pinfo->mLength*7 );

        snprintf( pinfo->sAddr+pinfo->mLength*min(pinfo->cIndex,7), 61,
                  "%s %s", timestamp+6, msg );

        if ( pinfo->cIndex <= 7 ) pinfo->cIndex++;

        pinfo->newMsg = 1;

        pinfo->lMutex->unlock();
    }

    if ( dlvl <= xps8Debug )
        printf( "%s.%s %s -- %s\n", timestamp, msec, prec->name, msg );

    return( 1 );
}

