#ifndef SAEJ1939_H
#define SAEJ1939_H
// Function Return Codes
#include <string.h>
#include <stdlib.h>
//#include "protos.h"
#include "cmsis_os.h"
#include "SAEJ1939.h"

#define RC_SUCCESS                      0               /**< There is no error */
#define RC_ERROR                        1               /**< A generic error happens */
#define RC_ETIMEOUT                     2               /**< Timed out */
#define RC_EFULL                        3               /**< The resource is full */
#define RC_EEMPTY                       4               /**< The resource is empty */
#define RC_ENOMEM                       5               /**< No memory */
#define RC_ENOSYS                       6               /**< No system */
#define RC_EBUSY                        7               /**< Busy */
#define RC_EIO                          8               /**< IO error */
#define RC_QUEUEEMPTY					9
#define RC_CANNOTTRANSMIT              10
#define RC_TIMEOUT                     11


// Internal Constants

#define J1939_FALSE				0
#define J1939_TRUE				1


// J1939 Default Priorities

#define J1939_CONTROL_PRIORITY			0x03
#define J1939_INFO_PRIORITY			0x06
#define J1939_PROPRIETARY_PRIORITY		0x06
#define J1939_REQUEST_PRIORITY			0x06
#define J1939_ACK_PRIORITY			0x06
#define J1939_TP_CM_PRIORITY			0x07
#define J1939_TP_DT_PRIORITY			0x07


// J1939 Defined Addresses

#define J1939_GLOBAL_ADDRESS			255
#define J1939_NULL_ADDRESS			254


// Some J1939 PDU Formats, Control Bytes, and PGN's

#define J1939_PF_REQUEST2			201
#define J1939_PF_TRANSFER			202

#define J1939_PF_ACKNOWLEDGMENT			232
#define J1939_ACK_CONTROL_BYTE			0
#define J1939_NACK_CONTROL_BYTE			1
#define J1939_ACCESS_DENIED_CONTROL_BYTE	2
#define J1939_CANNOT_RESPOND_CONTROL_BYTE	3

#define J1939_PF_REQUEST			234

#define J1939_PF_DT				235		// Data Transfer message

#define J1939_PF_TP_CM				236		// Connection Management message
#define J1939_RTS_CONTROL_BYTE			16		// Request to Send control byte of CM message
#define J1939_CTS_CONTROL_BYTE			17		// Clear to Send control byte of CM message
#define J1939_EOMACK_CONTROL_BYTE		19		// End of Message control byte of CM message
#define J1939_BAM_CONTROL_BYTE			32		// BAM control byte of CM message
#define J1939_CONNABORT_CONTROL_BYTE		255		// Connection Abort control byte of CM message

#define J1939_PGN2_REQ_ADDRESS_CLAIM		0x00
#define J1939_PGN1_REQ_ADDRESS_CLAIM		0xEE
#define J1939_PGN0_REQ_ADDRESS_CLAIM		0x00

#define J1939_PGN2_COMMANDED_ADDRESS		0x00
#define J1939_PGN1_COMMANDED_ADDRESS		0xFE	        // (-81 PDU Format)
#define J1939_PGN0_COMMANDED_ADDRESS		0xD8	        // (-81 PDU Specific)

#define J1939_PF_ADDRESS_CLAIMED		238		// With global address
#define J1939_PF_CANNOT_CLAIM_ADDRESS		238		// With null address
#define J1939_PF_PROPRIETARY_A			239
#define J1939_PF_PROPRIETARY_B			255   

// Internal definitions

#define ADDRESS_CLAIM_TX			1
#define ADDRESS_CLAIM_RX			2
#define ECAN_CONFIG_MODE			0x80
#define ECAN_ERROR_INT_ENABLE		0xA0
#define ECAN_NORMAL_MODE			0x00
#define ECAN_RX_INT_ENABLE_FIFO		0x02
#define ECAN_RX_INT_ENABLE_LEGACY	0x03
#define ECAN_SELECT_RX_BUFFER		0x10
#define ECAN_SET_LEGACY_MODE		0x00
#define ECAN_SET_FIFO_MODE			0xA0
#define ECAN_TX_INT_ENABLE_LEGACY	0x0C


#define TRTIMEOUT (200)
#define THTIMEOUT (500)
#define T1TIMEOUT (750)
#define T2TIMEOUT (1250)
#define T3TIMEOUT (1250)
#define T4TIMEOUT (1050)
#define BAMDTTIME (55)
#define WAITFOREVER RT_WAITING_FOREVER

#define J1939_DATA_LENGTH	8

/*
*
*/
struct J1939_msg {
		unsigned int	rsv1				: 3;	// This needs pre and post processing.
		unsigned int	DataPage			: 1;
		unsigned int	edp					: 1;
		unsigned int	Priority			: 3;
		unsigned char	PDUFormat;					// CA should use only PDUFormat.
		unsigned char	PDUSpecific;
		unsigned char	SourceAddress;
		unsigned int	DataLength 			: 4;
		unsigned int	rsv2				: 4;	// RTR bit, value always 0x00
		unsigned char	Data[J1939_DATA_LENGTH];
	};
typedef struct J1939_msg * J1939_msg_t;



/*
*
*/
struct J1939_CAname {
	unsigned long identity 			: 21;
	unsigned long mcode 			: 11;
	unsigned int ECUInstance 		: 3;
	unsigned int FunInstance 		: 5;
	unsigned char Function;
	unsigned int RSV 				: 1;
	unsigned int VehicleSYS 		: 7;
	unsigned int VSYSInstance 		: 4;
	unsigned int IndustryGroup 		: 3;
	unsigned int ArbCapable 		: 1;	
	};
typedef struct J1939_CAname * J1939_CAname_t;


/*
*
*/
struct listnode {
	struct listnode *p;
	struct listnode *n;
	};


/*
*
*/
struct datalist {
	struct listnode node;
	unsigned char	Data[J1939_DATA_LENGTH];
	};
typedef struct datalist * datalist_t;

/*
*
*/
struct cm_timer {
	unsigned short value;
	unsigned char en;  /* 1 start timer;0 stop timer*/
	unsigned char release;/* 1 get data; 0 not get data*/
	};

/*
*
*/
struct J1939_CM {
	unsigned short int sendpackets;
	unsigned short int nextnum;
	unsigned short int sendnum;
	unsigned char cm_lock;
	unsigned char sendcomp;
	#if 0
	osMessageQId trq;  
	osMessageQId thq;	
	osMessageQId t1q;	
	osMessageQId t2q;	
	osMessageQId t3q;	
	#endif
	
	struct cm_timer tr;/* 200ms  */
	struct cm_timer th;/* 500ms */
	struct cm_timer t1;/* 750ms */
	struct cm_timer t2;/* 1250ms */
	struct cm_timer t3;/* 1250ms */
	struct cm_timer t4;/* 1050ms */
	unsigned short int rcvpackets;
	unsigned short int rcvbytes;
	unsigned short int rcvcount;
	unsigned char DTPF;
	unsigned char DTPS;
	unsigned char DTDP;
	};

/*
*
*/
struct J1939_BAM {
	//unsigned short int sendpackets;
	//unsigned short int nextnum;
	//unsigned short int sendnum;
	unsigned char cm_lock;
	//unsigned char sendcomp;
	#if 0
	osMessageQId trq;  
	osMessageQId thq;	
	osMessageQId t1q;	
	osMessageQId t2q;	
	osMessageQId t3q;	
	#endif
	
	//struct cm_timer tr;/* 200ms  */
	//struct cm_timer th;/* 500ms */
	struct cm_timer t1;/* 750ms */
	//struct cm_timer t2;/* 1250ms */
	//struct cm_timer t3;/* 1250ms */
	//struct cm_timer t4;/* 1050ms */
	unsigned short int rcvpackets;
	unsigned short int rcvbytes;
	unsigned short int rcvcount;
	};

/*
*
*/
struct J1939_nm {
	unsigned char cannotClaimAddr;
	unsigned char WaitingForAddressClaimContention;
	struct cm_timer t250;/* 250ms */
	};

/*
*
*/
struct CA_Msg {
	J1939_CAname_t CAname;
	struct J1939_msg msg;
	unsigned short int sendbytes;
	struct J1939_CM cm;
	struct J1939_BAM bam;
	struct J1939_nm nm;
	unsigned char *rcvdata;
	unsigned char *bamrcvdata;
	osMessageQId txq;	
	osMessageQId rxq;	
	};
typedef struct CA_Msg * CA_Msg_t;




struct list_node
{
    struct list_node *next;                          /**< point to next node. */
    struct list_node *prev;                          /**< point to prev node. */
};
typedef struct list_node * list_node_t;                  /**< Type for lists. */


/**
 * @brief insert a node before a list
 *
 * @param n new node to be inserted
 * @param l list to insert it
 */
static __inline void j1939_list_insert_before(struct list_node *l, struct list_node *n)
{
    l->prev->next = n;
    n->prev = l->prev;

    l->prev = n;
    n->next = l;
}

/**
 * @brief initialize a list
 *
 * @param l list to be initialized
 */
static __inline void list_init(struct list_node *l)
{
    l->next = l->prev = l;
}

/*
*
*/
struct CA_node {
	CA_Msg_t ca_msg;
	struct list_node node;
	};
typedef struct CA_node * CA_node_t;

/*
*
*/
struct CA_list {
	CA_node_t ca_node;
	struct list_node head;
	};
typedef struct CA_list * CA_list_t;

/*
*
*/
struct q_Message {
	unsigned short int nextnum;
	unsigned short int sendnum;
	unsigned short int timeout; /* 1: timout; 0: normal*/

	};
typedef struct q_Message * q_Message_t;

struct j1939_rx_msg
{
	char *data;
	unsigned short size;
	unsigned char PS;
	unsigned char PF;
	unsigned char dp;
};

struct rx_Message
{
	//struct j1939_rx_msg * j1939_rxmsg;
	unsigned char *data;
	unsigned short size;
	unsigned char PS;
	unsigned char PF;
	unsigned char dp;
};


unsigned char writeCan(J1939_msg_t j1939_msg);
unsigned char readCan(J1939_msg_t j1939_msg);

void receiveMessage(void);
unsigned char sendMessage(CA_Msg_t CA_msg, unsigned char *data);
unsigned char J1939_CA_register(CA_Msg_t ca_msg);
unsigned char initJ1939(void);




unsigned char J1939_RequestForAddressClaimHandling( J1939_msg_t j1939_msg ,CA_Msg_t CA_msg );
unsigned char J1939_AddressClaimHandling(  J1939_msg_t j1939_msg ,CA_Msg_t CA_msg  );
unsigned char sendAddrClaimed(CA_Msg_t CA_msg);
CA_Msg_t Addr_FindCA(CA_list_t calist, unsigned char addr);
unsigned char CA_FindAddress(CA_list_t calist,CA_Msg_t ca_msg);
unsigned char initAddr(CA_Msg_t ca_msg);

#endif
