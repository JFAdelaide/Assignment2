#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective repeat
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12      /* the min sequence space for SR must be at least windowsize*2 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your 
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for ( i=0; i<20; i++ ) 
    checksum += (int)(packet.payload[i]);

  return checksum;
}

bool IsCorrupted(struct pkt packet)
{
  if (packet.checksum == ComputeChecksum(packet))
    return (false);
  else
    return (true);
}


/********* Sender (A) variables and functions ************/

static struct pkt buffer[WINDOWSIZE];  /* array for storing packets waiting for ACK */
static int windowfirst, windowlast;    /* array indexes of the first/last packet awaiting ACK */
static int windowcount;                /* the number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */
static bool acked[WINDOWSIZE];         /* true if packet at index is ACKed */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if ( windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for ( i=0; i<20 ; i++ ) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    /* windowlast will always be 0 for alternating bit; but not for GoBackN */
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    /* Set ACK status to false for new packet */
    acked[windowlast] = false;
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3 (A, sendpkt);

    /* Start timer for first packet or if earliest packet is unacknowledged */
    if (windowcount == 1 || !acked[windowfirst])
      starttimer(A, RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked,  window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}


/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data.
*/
void A_input(struct pkt packet)
{
  /* Rewritten to handle individual ACKs instead of cumulative ACKs */
  if (!IsCorrupted(packet)) {
    if (TRACE > 0) 
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[windowlast].seqnum;
      /* Check if ACK is within the current window */
      bool in_window = ((seqfirst <= seqlast) && (packet.acknum >= seqfirst && packet.acknum <= seqlast)) ||
                       ((seqfirst > seqlast) && (packet.acknum >= seqfirst || packet.acknum <= seqlast));

      if (in_window) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;

        /* Ensure ACKed packet is marked correctly and handle wraparound */
        bool found = false;
        for (int i = 0; i < WINDOWSIZE; i++) {
          if (buffer[i].seqnum == packet.acknum && !acked[i]) {
            acked[i] = true;
            found = true;
            break;
          }
        }
        if (!found && TRACE > 0)
          printf("----A: Warning: ACK %d not found in buffer\n", packet.acknum);

        /* Slide window for consecutive ACKed packets */
        while (windowcount > 0 && acked[windowfirst]) {
          windowcount--;
          windowfirst = (windowfirst + 1) % WINDOWSIZE;
          /* Ensure timer is restarted if unacknowledged packets remain */
          if (windowcount > 0 && !acked[windowfirst]) {
            stoptimer(A);
            starttimer(A, RTT);
          }
        }

        /* Adjust timer: stop if no packets, restart if unacknowledged packets remain */
        stoptimer(A);
        if (windowcount > 0 && !acked[windowfirst])
          starttimer(A, RTT);
      }
      else if (TRACE > 0)
        printf("----A: ACK %d outside window, ignored\n", packet.acknum);
    }
  }
  else if (TRACE > 0)
    printf("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  /* Resend only the earliest unacknowledged packet */
  if (windowcount > 0 && !acked[windowfirst]) {
    if (TRACE > 0)
      printf("----A: timeout, resending packet %d\n", buffer[windowfirst].seqnum);

    tolayer3(A, buffer[windowfirst]);
    packets_resent++;

    /* Ensure timer is restarted to avoid infinite retransmissions */
    stoptimer(A);
    starttimer(A, RTT);
  }
}


/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  /* initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
		     new packets are placed in winlast + 1 
		     so initially this is set to -1
		   */
  windowcount = 0;

  /* Initialize ACK status for all buffer slots */
  for (int i = 0; i < WINDOWSIZE; i++)
    acked[i] = false;
}


/********* Receiver (B)  variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */

static struct pkt recv_buffer[WINDOWSIZE]; /* Buffer for out-of-order packets */
static bool received[WINDOWSIZE]; /* Track received status */


/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  /* Rewritten to buffer out-of-order packets and send individual ACKs */
  struct pkt sendpkt;
  int i;

  if (!IsCorrupted(packet)) {
    /* Check if packet is within receiver window */
    int window_start = expectedseqnum;
    int window_end = (expectedseqnum + WINDOWSIZE - 1) % SEQSPACE;
    bool in_window = ((window_start <= window_end) && (packet.seqnum >= window_start && packet.seqnum <= window_end)) ||
                     ((window_start > window_end) && (packet.seqnum >= window_start || packet.seqnum <= window_end));

    if (in_window) {
      if (TRACE > 0)
        printf("----B: packet %d received, in window\n", packet.seqnum);

      /* Find a buffer slot for the packet */
      int buffer_index = -1;
      for (i = 0; i < WINDOWSIZE; i++) {
        if (!received[i] || recv_buffer[i].seqnum == packet.seqnum) {
          buffer_index = i;
          break;
        }
      }

      /* Store packet if not already received */
      if (buffer_index >= 0 && !received[buffer_index]) {
        recv_buffer[buffer_index] = packet;
        received[buffer_index] = true;
        packets_received++;

        /* Send ACK for this packet */
        sendpkt.seqnum = B_nextseqnum;
        B_nextseqnum = (B_nextseqnum + 1) % 2;
        sendpkt.acknum = packet.seqnum;
        for (i = 0; i < 20; i++)
          sendpkt.payload[i] = '0';
        sendpkt.checksum = ComputeChecksum(sendpkt);
        tolayer3(B, sendpkt);

        if (TRACE > 0)
          printf("----B: sending ACK %d\n", sendpkt.acknum);

        /* Deliver in-order packets */
        /* Add safety check to prevent infinite loop */
        int max_deliveries = WINDOWSIZE; /* Prevent infinite loop */
        while (max_deliveries > 0) {
          bool found = false;
          for (i = 0; i < WINDOWSIZE; i++) {
            if (received[i] && recv_buffer[i].seqnum == expectedseqnum) {
              tolayer5(B, recv_buffer[i].payload);
              received[i] = false;
              expectedseqnum = (expectedseqnum + 1) % SEQSPACE;
              found = true;
              break;
            }
          }
          if (!found)
            break;
          max_deliveries--;
        }
      }
    }
    else if (TRACE > 0)
      printf("----B: packet %d outside window, discarded\n", packet.seqnum);
  }
  else if (TRACE > 0)
    printf("----B: packet %d corrupted, discarded\n", packet.seqnum);
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;

  /* Initialize receiver buffer */
  for (int i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
