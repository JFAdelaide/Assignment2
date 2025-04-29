#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective Repeat protocol.

   Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

#define RTT  16.0       /* round trip time. MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packets */
#define SEQSPACE 12     /* SR requires SEQSPACE >= 2 * WINDOWSIZE */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet. Used by both sender and receiver  
   the simulator will overwrite part of your packet with 'z's. It will not overwrite your 
   original checksum. This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/
int ComputeChecksum(struct pkt packet)
{
  int checksum = 0;
  int i;

  checksum = packet.seqnum;
  checksum += packet.acknum;
  for (i = 0; i < 20; i++) 
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
static bool acked[WINDOWSIZE];         /* tracks whether each packet is acknowledged */
static int windowfirst, windowlast;    /* array indexes of the first/last packet in window */
static int windowcount;                /* number of packets currently awaiting an ACK */
static int A_nextseqnum;               /* the next sequence number to be used by the sender */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  /* if not blocked waiting on ACK */
  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    /* create packet */
    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    /* put packet in window buffer */
    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    acked[windowlast] = false;          /* mark as unacknowledged */
    windowcount++;

    /* send out packet */
    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* start timer if this is the first unacknowledged packet */
    if (windowcount == 1)
      starttimer(A, RTT);

    /* get next sequence number, wrap back to 0 */
    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
  /* if blocked, window is full */
  else {
    if (TRACE > 0)
      printf("----A: New message arrives, send window is full\n");
    window_full++;
  }
}

/* called from layer 3, when a packet arrives for layer 4 
   In this practical this will always be an ACK as B never sends data. */
void A_input(struct pkt packet)
{
  int i, buf_index;

  /* if received ACK is not corrupted */ 
  if (!IsCorrupted(packet)) {
    if (TRACE > 0)
      printf("----A: uncorrupted ACK %d is received\n",packet.acknum);
    total_ACKs_received++;

    /* find the packet in the window */
    if (windowcount > 0) {
      for (i = 0; i < windowcount; i++) {
        buf_index = (windowfirst + i) % WINDOWSIZE;
        if (buffer[buf_index].seqnum == packet.acknum && !acked[buf_index]) {
          /* mark packet as acknowledged */
          acked[buf_index] = true;
          if (TRACE > 0)
            printf("----A: ACK %d is not a duplicate\n",packet.acknum);
          new_ACKs++;

          /* stop timer if this was the earliest unacknowledged packet */
          if (buf_index == windowfirst) {
            stoptimer(A);
            /* slide window if earliest packet is ACKed */
            while (windowcount > 0 && acked[windowfirst]) {
              windowcount--;
              windowfirst = (windowfirst + 1) % WINDOWSIZE;
            }
            /* restart timer if there are still unacknowledged packets */
            if (windowcount > 0)
              starttimer(A, RTT);
          }
          break;
        }
      }
    }
    else if (TRACE > 0)
      printf ("----A: duplicate ACK received, do nothing!\n");
  }
  else 
    if (TRACE > 0)
      printf ("----A: corrupted ACK is received, do nothing!\n");
}

/* called when A's timer goes off */
void A_timerinterrupt(void)
{
  int i;

  if (TRACE > 0)
    printf("----A: time out,resend packets!\n");

  /* find the earliest unacknowledged packet */
  for (i = 0; i < windowcount; i++) {
    int buf_index = (windowfirst + i) % WINDOWSIZE;
    if (!acked[buf_index]) {
      if (TRACE > 0)
        printf ("---A: resending packet %d\n", (buffer[(windowfirst+i) % WINDOWSIZE]).seqnum);
      tolayer3(A, buffer[buf_index]);
      packets_resent++;
      /* restart timer for the next unacknowledged packet */
      starttimer(A, RTT);
      break; /* only retransmit the earliest unacked packet */
    }
  }
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called */
void A_init(void)
{
  int i; /* Declare i at block start for C90 */
  /* Initialise A's window, buffer and sequence number */
  A_nextseqnum = 0;  /* A starts with seq num 0, do not change this */
  windowfirst = 0;
  windowlast = -1;   /* windowlast is where the last packet sent is stored.  
                        new packets are placed in windowlast + 1 
                        so initially this is set to -1 */
  windowcount = 0;
  for (i = 0; i < WINDOWSIZE; i++) {
      acked[i] = false;
  }
}

/********* Receiver (B) variables and procedures ************/

static struct pkt rcv_buffer[WINDOWSIZE]; /* buffer for out-of-order packets */
static bool received[WINDOWSIZE];         /* tracks which packets are buffered */
static int expectedseqnum;                /* the sequence number expected for in-order delivery */
/* static int B_nextseqnum;                   the sequence number for the next ACK packets sent by B */

/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  /* Declare all variables at block start for C90 */
  struct pkt sendpkt;
  int i;
  int window_start; 
  int window_end;
  bool in_window;
  bool below_window;
  int buffer_index;

  /* Initialize ACK packet */
  sendpkt.seqnum = NOTINUSE; /* B only sends ACKs, no sequence number needed */
  sendpkt.acknum = NOTINUSE; /* Will be set based on packet */
  for (i = 0; i < 20; i++)
    sendpkt.payload[i] = '0';

  /* If packet is not corrupted */
  if (!IsCorrupted(packet)) {
    /* Define receiver's window */
    window_start = expectedseqnum;
    window_end = (expectedseqnum + WINDOWSIZE - 1) % SEQSPACE;
    in_window = ((window_start <= window_end && packet.seqnum >= window_start && packet.seqnum <= window_end) ||
                (window_start > window_end && (packet.seqnum >= window_start || packet.seqnum <= window_end)));

    if (TRACE > 0)
      printf("----B: packet %d is correctly received, send ACK!\n",packet.seqnum);

    /* Check if packet is below the window (already delivered but needs ACK) */
    below_window = ((window_start <= packet.seqnum && packet.seqnum < window_start) ||
                  (window_start > packet.seqnum && (packet.seqnum <= window_end || packet.seqnum < window_start)));

    if (in_window || below_window) {
      /* Calculate buffer index for in-window packets */
      buffer_index = -1;
      if (in_window) {
        buffer_index = (packet.seqnum - window_start + SEQSPACE) % SEQSPACE;
      }

      /* Store packet if in window and not already received */
      if (in_window && buffer_index >= 0 && !received[buffer_index]) {
        rcv_buffer[buffer_index] = packet;
        received[buffer_index] = true;
        packets_received++;
        /*if (TRACE > 0)
          printf("----B: stored packet %d in buffer[%d]\n", packet.seqnum, buffer_index);*/
      } 
      /*else if (in_window && received[buffer_index]) {
        if (TRACE > 0)
          printf("----B: packet %d already in buffer[%d], sending ACK\n", packet.seqnum, buffer_index);
      } 
      else if (below_window) {
        if (TRACE > 0)
          printf("----B: packet %d below window, sending ACK\n", packet.seqnuåm);
      }*/

      /* Send ACK for this packet */
      sendpkt.acknum = packet.seqnum;
      sendpkt.checksum = ComputeChecksum(sendpkt);
      /*if (TRACE > 0)
        printf("----B: sending ACK %d\n", sendpkt.acknum);*/
      tolayer3(B, sendpkt);

      /* Deliver in-order packets to layer 5 and slide window */
      while (received[0] && rcv_buffer[0].seqnum == expectedseqnum) {
        /*if (TRACE > 0)
          printf("----B: delivering packet %d to layer 5\n", rcv_buffer[0].seqnum);*/
        tolayer5(B, rcv_buffer[0].payload);
        received[0] = false;
        expectedseqnum = (expectedseqnum + 1) % SEQSPACE;

        /* Shift buffer contents left */
        for (i = 0; i < WINDOWSIZE - 1; i++) {
          rcv_buffer[i] = rcv_buffer[i + 1];
          received[i] = received[i + 1];
        }
        rcv_buffer[WINDOWSIZE - 1].seqnum = NOTINUSE;
        received[WINDOWSIZE - 1] = false;

        /*if (TRACE > 0)
          printf("----B: window slid, new expectedseqnum=%d\n", expectedseqnum);*/
      }
    } 
    else {
      /* Packet is beyond window, send ACK for last in-order packet */
      sendpkt.acknum = (expectedseqnum - 1 + SEQSPACE) % SEQSPACE;
      sendpkt.checksum = ComputeChecksum(sendpkt);
      /*if (TRACE > 0)
        printf("----B: packet %d beyond window, sending ACK %d\n", packet.seqnum, sendpkt.acknum);*/
      tolayer3(B, sendpkt);
    }
  } 
  else {
    /* Corrupted packet, send ACK for last in-order packet */
    sendpkt.acknum = (expectedseqnum - 1 + SEQSPACE) % SEQSPACE;
    sendpkt.checksum = ComputeChecksum(sendpkt);
    if (TRACE > 0)
      printf("----B: packet corrupted or not expected sequence number, resend ACK!\n");
    tolayer3(B, sendpkt);
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  int i; /* Declare i at block start for C90 */
  expectedseqnum = 0;
  for (i = 0; i < WINDOWSIZE; i++) {
      received[i] = false;
      rcv_buffer[i].seqnum = NOTINUSE; /* Initialise buffer slots as empty */
  }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from A-to-B, there is no B_output() */
void B_output(struct msg message)  
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
