#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "emulator.h"
#include "sr.h"

/* ******************************************************************
   Selective repeat
**********************************************************************/

#define RTT  24.0       /* round trip time. TEMPORARY for testing; MUST BE SET TO 16.0 when submitting */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet */
#define SEQSPACE 12     /* the min sequence space for SR must be at least windowsize*2 */
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
static bool timer_running;             /* track if timer is running to avoid overlaps */

/* called from layer 5 (application layer), passed the message to be sent to other side */
void A_output(struct msg message)
{
  struct pkt sendpkt;
  int i;

  if (windowcount < WINDOWSIZE) {
    if (TRACE > 1)
      printf("----A: New message arrives, send window is not full, send new messge to layer3!\n");

    sendpkt.seqnum = A_nextseqnum;
    sendpkt.acknum = NOTINUSE;
    for (i = 0; i < 20; i++) 
      sendpkt.payload[i] = message.data[i];
    sendpkt.checksum = ComputeChecksum(sendpkt); 

    windowlast = (windowlast + 1) % WINDOWSIZE; 
    buffer[windowlast] = sendpkt;
    acked[windowlast] = false;
    windowcount++;

    if (TRACE > 0)
      printf("Sending packet %d to layer 3\n", sendpkt.seqnum);
    if (TRACE > 1)
      printf("----A: windowcount=%d, windowfirst=%d, windowlast=%d, seqnum=%d\n", 
             windowcount, windowfirst, windowlast, sendpkt.seqnum);
    tolayer3(A, sendpkt);

    /* Start timer only for the first packet if not already running */
    if (windowcount == 1 && !timer_running) {
      if (TRACE > 1)
        printf("----A: Starting timer for packet %d\n", sendpkt.seqnum);
      starttimer(A, RTT);
      timer_running = true;
    }

    A_nextseqnum = (A_nextseqnum + 1) % SEQSPACE;  
  }
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
  if (!IsCorrupted(packet)) {
    if (TRACE > 0) 
      printf("----A: uncorrupted ACK %d is received\n", packet.acknum);
    total_ACKs_received++;

    if (windowcount != 0) {
      int seqfirst = buffer[windowfirst].seqnum;
      int seqlast = buffer[windowlast].seqnum;
      /* Improved in_window check using modulo arithmetic */
      int dist_first = (packet.acknum - seqfirst + SEQSPACE) % SEQSPACE;
      int dist_last = (seqlast - seqfirst + SEQSPACE) % SEQSPACE;
      bool in_window = dist_first <= dist_last;

      if (TRACE > 1)
        printf("----A: Checking ACK %d, seqfirst=%d, seqlast=%d, dist_first=%d, dist_last=%d, in_window=%d\n",
               packet.acknum, seqfirst, seqlast, dist_first, dist_last, in_window);

      if (in_window) {
        if (TRACE > 0)
          printf("----A: ACK %d is not a duplicate\n", packet.acknum);
        new_ACKs++;

        /* Mark ACKed packet by searching buffer */
        bool found = false;
        for (int i = 0; i < WINDOWSIZE; i++) {
          int buffer_index = (windowfirst + i) % WINDOWSIZE;
          if (buffer[buffer_index].seqnum == packet.acknum && !acked[buffer_index]) {
            acked[buffer_index] = true;
            found = true;
            if (TRACE > 1)
              printf("----A: Marked packet %d as ACKed at index %d\n", packet.acknum, buffer_index);
            break;
          }
        }
        if (!found && TRACE > 0)
          printf("----A: Warning: ACK %d not found in buffer\n", packet.acknum);

        /* Debug: Print acked status before sliding */
        if (TRACE > 1) {
          printf("----A: Acked status before sliding: ");
          for (int i = 0; i < WINDOWSIZE; i++)
            printf("%d:%d ", i, acked[i]);
          printf("\n");
        }

        /* Stop timer if running and first packet is ACKed */
        if (timer_running && acked[windowfirst]) {
          if (TRACE > 1)
            printf("----A: Stopping timer before sliding\n");
          stoptimer(A);
          timer_running = false;
        }

        /* Slide window for consecutive ACKed packets */
        int packets_slid = 0;
        while (windowcount > 0 && acked[windowfirst]) {
          if (TRACE > 1)
            printf("----A: Sliding window, windowfirst=%d, windowcount=%d, seqnum=%d\n", 
                   windowfirst, windowcount, buffer[windowfirst].seqnum);
          windowcount--;
          packets_slid++;
          windowfirst = (windowfirst + 1) % WINDOWSIZE;
        }

        /* Reset acked status for slid packets */
        for (int i = 0; i < packets_slid; i++) {
          int index = (windowfirst - packets_slid + i + WINDOWSIZE) % WINDOWSIZE;
          acked[index] = false;
        }

        /* Debug: Print acked status and buffer contents after sliding */
        if (TRACE > 1) {
          printf("----A: Acked status after sliding: ");
          for (int i = 0; i < WINDOWSIZE; i++)
            printf("%d:%d ", i, acked[i]);
          printf("\n----A: Buffer seqnums: ");
          for (int i = 0; i < WINDOWSIZE; i++)
            printf("%d:%d ", i, buffer[i].seqnum);
          printf("\n");
        }

        /* Restart timer only if unacknowledged packets remain and timer is not running */
        if (windowcount > 0 && !acked[windowfirst] && !timer_running) {
          if (TRACE > 1)
            printf("----A: Restarting timer for packet %d\n", buffer[windowfirst].seqnum);
          starttimer(A, RTT);
          timer_running = true;
        }
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
  if (windowcount > 0 && !acked[windowfirst]) {
    if (TRACE > 0)
      printf("----A: timeout, resending packet %d, windowcount=%d, windowfirst=%d\n", 
             buffer[windowfirst].seqnum, windowcount, windowfirst);
    tolayer3(A, buffer[windowfirst]);
    packets_resent++;
    if (TRACE > 1)
      printf("----A: Resetting timer for packet %d\n", buffer[windowfirst].seqnum);
    if (timer_running) {
      stoptimer(A);
      timer_running = false;
    }
    starttimer(A, RTT);
    timer_running = true;
  } else if (TRACE > 0) {
    printf("----A: timeout ignored, no unacknowledged packets or window empty\n");
  }
}

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init(void)
{
  A_nextseqnum = 0;
  windowfirst = 0;
  windowlast = -1;
  windowcount = 0;
  timer_running = false;

  for (int i = 0; i < WINDOWSIZE; i++)
    acked[i] = false;

  if (TRACE > 1)
    printf("----A: Initialized, A_nextseqnum=%d, windowcount=%d\n", A_nextseqnum, windowcount);
}

/********* Receiver (B) variables and procedures ************/

static int expectedseqnum; /* the sequence number expected next by the receiver */
static int B_nextseqnum;   /* the sequence number for the next packets sent by B */

static struct pkt recv_buffer[WINDOWSIZE]; /* Buffer for out-of-order packets */
static bool received[WINDOWSIZE]; /* Track received status */

/* called from layer 3, when a packet arrives for layer 4 at B */
void B_input(struct pkt packet)
{
  struct pkt sendpkt;
  int i;

  if (!IsCorrupted(packet)) {
    int window_start = expectedseqnum;
    int window_end = (expectedseqnum + WINDOWSIZE - 1) % SEQSPACE;
    bool in_window = ((window_start <= window_end) && (packet.seqnum >= window_start && packet.seqnum <= window_end)) ||
                     ((window_start > window_end) && (packet.seqnum >= window_start || packet.seqnum <= window_end));

    if (in_window) {
      if (TRACE > 0)
        printf("----B: packet %d received, in window, expectedseqnum=%d\n", packet.seqnum, expectedseqnum);

      int buffer_index = -1;
      for (i = 0; i < WINDOWSIZE; i++) {
        if (!received[i]) {
          buffer_index = i;
          break;
        }
      }

      if (buffer_index >= 0) {
        recv_buffer[buffer_index] = packet;
        received[buffer_index] = true;
        packets_received++;

        sendpkt.seqnum = B_nextseqnum;
        B_nextseqnum = (B_nextseqnum + 1) % 2;
        sendpkt.acknum = packet.seqnum;
        for (i = 0; i < 20; i++)
          sendpkt.payload[i] = '0';
        sendpkt.checksum = ComputeChecksum(sendpkt);
        tolayer3(B, sendpkt);

        if (TRACE > 0)
          printf("----B: sending ACK %d\n", sendpkt.acknum);

        int max_deliveries = WINDOWSIZE;
        while (max_deliveries > 0) {
          bool found = false;
          for (i = 0; i < WINDOWSIZE; i++) {
            if (received[i] && recv_buffer[i].seqnum == expectedseqnum) {
              if (TRACE > 1)
                printf("----B: Delivering packet %d to layer 5\n", recv_buffer[i].seqnum);
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
  else if (TRACE > 0) {
    printf("----B: packet %d corrupted, discarded\n", packet.seqnum);
    if (TRACE > 1)
      printf("----B: Discarding corrupted packet %d, expectedseqnum=%d\n", packet.seqnum, expectedseqnum);
  }
}

/* the following routine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init(void)
{
  expectedseqnum = 0;
  B_nextseqnum = 1;

  for (int i = 0; i < WINDOWSIZE; i++)
    received[i] = false;
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

void B_output(struct msg message)  
{
}

void B_timerinterrupt(void)
{
}
