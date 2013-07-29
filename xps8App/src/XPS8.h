#define VERSION 1.0

struct Positioner
{
    char        gname[80];
    char        pname[80];
    char        stage[80];
    int         usocket;
    int         msocket;
};

struct XPS8
{
    epicsMutex *uMutex;
    int         socket;
    int         connected;
    int         update;

    epicsMutex *lMutex;
    int         nMessages;
    int         mLength;
    int         cIndex;
    char       *sAddr;

    Positioner  positioner[8];
};

typedef union
{
    unsigned long All;
    struct
    {
	unsigned int RA_DIRECTION   :1;	/* (last) 0=Negative, 1=Positive */
	unsigned int RA_DONE        :1;	/* a motion is complete */
	unsigned int RA_PLUS_LS     :1; /* plus limit switch has been hit */
	unsigned int RA_HOME        :1; /* The home signal is on */
	unsigned int EA_SLIP        :1; /* continue on stall detect */
	unsigned int EA_POSITION    :1; /* position maintenence enabled */
	unsigned int EA_SLIP_STALL  :1; /* slip/stall detected */
	unsigned int EA_HOME        :1; /* encoder home signal on */
	unsigned int EA_PRESENT     :1; /* encoder is present */
	unsigned int RA_PROBLEM     :1; /* driver stopped polling */
	unsigned int RA_MOVING      :1;	/* non-zero velocity present */
	unsigned int GAIN_SUPPORT   :1;	/* Motor supports closed-loop position control. */
	unsigned int CNTRL_COMM_ERR :1;	/* Controller communication error. */
	unsigned int RA_MINUS_LS    :1;	/* minus limit switch has been hit */
	unsigned int RA_HOMED       :1; /* Axis has been homed.*/
	unsigned int RA_POWERUP     :1; /* Power-cycled */
	unsigned int RA_STALL       :1; /* Stall detected */
	unsigned int MCHB           :1; /* Missing MCode heart-beat */
	unsigned int NA             :6; /* N/A bits */
	unsigned int ERRNO          :8; /* error number */
    } Bits;                                
} msta_field;

#define TCP_TIMEOUT 2.0

/* Bit map of changed fields */
typedef union
{
    epicsUInt32 All;
    struct
    {
        unsigned int M_SERR     :1;
        unsigned int M_DSTA     :1;
        unsigned int M_HSTA     :1;
        unsigned int M_SNUM     :1;
        unsigned int M_ERR      :1;
        unsigned int M_AMAP     :1;
        unsigned int M_VAL      :1;
        unsigned int M_DVAL     :1;
        unsigned int M_DRBV     :1;
        unsigned int M_RBV      :1;
        unsigned int M_DIFF     :1;
        unsigned int M_MIP      :1;
        unsigned int M_MOVN     :1;
        unsigned int M_DMOV     :1;
        unsigned int M_RCNT     :1;
        unsigned int M_MISS     :1;
        unsigned int M_HLS      :1;
        unsigned int M_LLS      :1;
        unsigned int M_LVIO     :1;
        unsigned int M_MSTA     :1;
        unsigned int M_BUTC     :1;
    } Bits;
} changed_fields;

#define MARK(FIELD)   { changed_fields temp; temp.All = prec->cmap;            \
                        temp.Bits.FIELD = 1; prec->cmap = temp.All; }

#define MARKED(FIELD) ( cmap.Bits.FIELD )

#define UNMARK_ALL      prec->cmap = 0

/* All db_post_events() calls set both VALUE and LOG bits */
#define DBE_VAL_LOG (unsigned int) (DBE_VALUE | DBE_LOG)

void Debug( int debug, const char* fmt, ... );

