/*
 ***************************************************************************************************************
 ***************************************************************************************************************
 ***************************************************************************************************************

 File:		  	   tcpServerRAW.c
 Modified By:     ControllersTech.com
 Updated:    	   26-Jul-2021

 ***************************************************************************************************************
 Copyright (C) 2017 ControllersTech.com

 This is a free software under the GNU license, you can redistribute it and/or modify it under the terms
 of the GNU General Public License version 3 as published by the Free Software Foundation.
 This software library is shared with public for educational purposes, without WARRANTY and Author is not liable for any damages caused directly
 or indirectly by this software, read more about this on the GNU General Public License.

 ***************************************************************************************************************
 */

/**
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of and a contribution to the lwIP TCP/IP stack.
 *
 * Credits go to Adam Dunkels (and the current maintainers) of this software.
 *
 * Christiaan Simons rewrote this file to get a more stable  application.
 *
 **/

/* This file was modified by ST */

#include "tcpserverRAW.h"
#include "lwip/tcp.h"
#include "string.h"
#include "audio_player.h"
struct tcp_server_struct *esTx = 0;
struct tcp_pcb *pcbTx = 0;
/*  protocol states */
enum tcp_server_states {
	ES_NONE = 0, ES_ACCEPTED, ES_RECEIVED, ES_CLOSING
};
int change_pointer=0;
/* structure for maintaining connection infos to be passed as argument
 to LwIP callbacks*/
struct tcp_server_struct {
	u8_t state; /* current connection state */
	u8_t retries;
	struct tcp_pcb *pcb; /* pointer on the current tcp_pcb */
	struct pbuf *p; /* pointer on the received/to be transmitted pbuf */
};
volatile int start_first = 0;
volatile int start_second = 0;
volatile int next=0;
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
		err_t err);
static void tcp_server_error(void *arg, err_t err);
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb);
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len);
static void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es);
static void tcp_server_connection_close(struct tcp_pcb *tpcb,
		struct tcp_server_struct *es);

static void tcp_server_handle(struct tcp_pcb *tpcb,
		struct tcp_server_struct *es);
char s[1] = { 'a' };
char b[1] = { 'b' };
char filesize[4];
char receive_name[256];
int write_time=0;
int file_size=-1;
extern uint8_t wavsram[97000];
extern int wav_end;
extern int wav_index;
int direction =1;
/* Impementation for the TCP Server
 1. Create TCP Block.
 2. Bind the Block to server address, and port.
 3. Listen for the  incoming requests by the client
 4. Accept the Request, and now the server is ready for the data transfer
 */
int first_pack=1;
int server_index=0;
int total_circle=0;
int total_length=0;
void tcp_server_init(void) {
	/* 1. create new tcp pcb */
	struct tcp_pcb *tpcb;
	tpcb = tcp_new();
	err_t err;
	/* 2. bind _pcb to port 7 ( protocol) */
	ip_addr_t myIPADDR;
	IP_ADDR4(&myIPADDR, 192, 168, 1, 200);
	err = tcp_bind(tpcb, &myIPADDR, 12345);

	if (err == ERR_OK) {
		/* 3. start tcp listening for _pcb */
		tpcb = tcp_listen(tpcb);
		/* 4. initialize LwIP tcp_accept callback function */
		tcp_accept(tpcb, tcp_server_accept);
	} else {
		/* deallocate the pcb */
		printf("444=====\n");

		memp_free(MEMP_TCP_PCB, tpcb);
	}
}

/**
 * @brief  This function is the implementation of tcp_accept LwIP callback
 * @param  arg: not used
 * @param  newpcb: pointer on tcp_pcb struct for the newly created tcp connection
 * @param  err: not used
 * @retval err_t: error status
 */
static err_t tcp_server_accept(void *arg, struct tcp_pcb *newpcb, err_t err) {
	err_t ret_err;
	struct tcp_server_struct *es;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);
	//printf("connnect 111\r\n");
	/* set priority for the newly accepted tcp connection newpcb */
	tcp_setprio(newpcb, TCP_PRIO_MIN);

	/* allocate structure es to maintain tcp connection information */
	es = (struct tcp_server_struct*) mem_malloc(
			sizeof(struct tcp_server_struct));
	if (es != NULL) {
		es->state = ES_ACCEPTED;
		es->pcb = newpcb;
		es->retries = 0;
		es->p = NULL;

		/* pass newly allocated es structure as argument to newpcb */
		tcp_arg(newpcb, es);

		/* initialize lwip tcp_recv callback function for newpcb  */
		tcp_recv(newpcb, tcp_server_recv);

		/* initialize lwip tcp_err callback function for newpcb  */
		tcp_err(newpcb, tcp_server_error);

		/* initialize lwip tcp_poll callback function for newpcb */
		tcp_poll(newpcb, tcp_server_poll, 0);
		tcp_server_handle(newpcb, es);
		send_information(s);

		ret_err = ERR_OK;
	} else {
		/*  close tcp connection */
		tcp_server_connection_close(newpcb, es);
		/* return memory error */
		ret_err = ERR_MEM;
	}
	return ret_err;
}

/**
 * @brief  This function is the implementation for tcp_recv LwIP callback
 * @param  arg: pointer on a argument for the tcp_pcb connection
 * @param  tpcb: pointer on the tcp_pcb connection
 * @param  pbuf: pointer on the received pbuf
 * @param  err: error information regarding the reveived pbuf
 * @retval err_t: error code
 */
static err_t tcp_server_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p,
		err_t err) {
	struct tcp_server_struct *es;
	err_t ret_err;

	LWIP_ASSERT("arg != NULL", arg != NULL);

	es = (struct tcp_server_struct*) arg;

	/* if we receive an empty tcp frame from client => close connection */
	if (p == NULL) {
		/* remote host closed connection */
		es->state = ES_CLOSING;
		if (es->p == NULL) {
			/* we're done sending, close connection */
			tcp_server_connection_close(tpcb, es);
		} else {
			/* we're not done yet */
			/* acknowledge received packet */
			tcp_sent(tpcb, tcp_server_sent);

			/* send remaining data*/
		}
		ret_err = ERR_OK;
	}
	/* else : a non empty frame was received from client but for some reason err != ERR_OK */
	else if (err != ERR_OK) {
		/* free received pbuf*/
		if (p != NULL) {
			es->p = NULL;
			pbuf_free(p);
		}
		ret_err = err;
	} else if (es->state == ES_ACCEPTED) {
		/* first data chunk in p->payload */
		es->state = ES_RECEIVED;

		/* store reference to incoming pbuf (chain) */
		// es->p = p;
		/* initialize LwIP tcp_sent callback function */
		tcp_sent(tpcb, tcp_server_sent);
		tcp_recved(tpcb, p->tot_len);

		/* handle the received data */
		//tcp_server_handle(tpcb, es);
		printf("connect%d \n",p->tot_len);
		//memcpy(filesize,p->payload,4);
		memcpy(wavsram+server_index*970,p->payload,p->tot_len);
		server_index+=1;

		send_information(s);

		ret_err = ERR_OK;
	} else if (es->state == ES_RECEIVED) {
		/* more data received from client and previous data has been already sent*/
		if (es->p == NULL) {
			printf("111====>%d\n", *(uint8_t*) p->payload);
			tcp_recved(tpcb, p->tot_len);
			if (p != NULL) {
				pbuf_free(p);
			}
			//  printf("hello world %d\n",p->tot_len);
			/* handle the received data */
			//tcp_server_handle(tpcb, es);
		} else {
			//if(change_pointer%2==0) {
			tcp_recved(tpcb, p->tot_len);
			total_circle+=1;

			if(server_index>=10 || server_index<=0 ){
				//direction=-direction;
				server_index=0;
			}
			memcpy(wavsram+server_index*970,p->payload,p->tot_len);
		    total_length+=p->tot_len;
		    if( first_pack){
		    	Audio_Player_Start();
		    	first_pack=0;
		    	wav_end=0;
		    	direction=1;
		    }
		    if(total_length==file_size){
		    	printf("end %d\n",file_size);
		    	total_length=0;
		    	server_index=0;
		    	wav_end=1;
		    	first_pack=1;
			}else{
				server_index+=direction;
				while(server_index-wav_index>6*direction){
					HAL_Delay(5);
				}
				send_information(s);
				printf("------%d %d %d\n",server_index,total_circle,wav_index);
			}
			if (p != NULL) {
				pbuf_free(p);
			}

		}


		ret_err = ERR_OK;
	} else if (es->state == ES_CLOSING) {
		/* odd case, remote side closing twice, trash data */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	} else {
		/* unknown es->state, trash data  */
		tcp_recved(tpcb, p->tot_len);
		es->p = NULL;
		pbuf_free(p);
		ret_err = ERR_OK;
	}
	return ret_err;
}

/**
 * @brief  This function implements the tcp_err callback function (called
 *         when a fatal tcp_connection error occurs.
 * @param  arg: pointer on argument parameter
 * @param  err: not used
 * @retval None
 */
static void tcp_server_error(void *arg, err_t err) {
//  struct tcp_server_struct *es;
//
//  LWIP_UNUSED_ARG(err);
//
//  es = (struct tcp_server_struct *)arg;
//  if (es != NULL)
//  {
//    /*  free es structure */
//    mem_free(es);
//  }
}

/**
 * @brief  This function implements the tcp_poll LwIP callback function
 * @param  arg: pointer on argument passed to callback
 * @param  tpcb: pointer on the tcp_pcb for the current tcp connection
 * @retval err_t: error code
 */
static err_t tcp_server_poll(void *arg, struct tcp_pcb *tpcb) {
	err_t ret_err;
	struct tcp_server_struct *es;

	es = (struct tcp_server_struct*) arg;
	if (es != NULL) {
		if (es->p != NULL) {
			tcp_sent(tpcb, tcp_server_sent);
			/* there is a remaining pbuf (chain) , try to send data */
			// tcp_server_send(tpcb, es);
		} else {
			/* no remaining pbuf (chain)  */
			if (es->state == ES_CLOSING) {
				/*  close tcp connection */
				tcp_server_connection_close(tpcb, es);
			}
		}
		ret_err = ERR_OK;
	} else {
		/* nothing to be done */
		tcp_abort(tpcb);
		ret_err = ERR_ABRT;
	}
	return ret_err;
}

/**
 * @brief  This function implements the tcp_sent LwIP callback (called when ACK
 *         is received from remote host for sent data)
 * @param  None
 * @retval None
 */
static err_t tcp_server_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
	struct tcp_server_struct *es;

	LWIP_UNUSED_ARG(len);

	es = (struct tcp_server_struct*) arg;
	es->retries = 0;

	if (es->p != NULL) {
		/* still got pbufs to send */
		tcp_sent(tpcb, tcp_server_sent);
		//tcp_server_send(tpcb, es);
	} else {
		/* if no more data to send and client closed connection*/
		if (es->state == ES_CLOSING)
			tcp_server_connection_close(tpcb, es);
	}
	return ERR_OK;
}

/**
 * @brief  This function is used to send data for tcp connection
 * @param  tpcb: pointer on the tcp_pcb connection
 * @param  es: pointer on _state structure
 * @retval None
 */
static void tcp_server_send(struct tcp_pcb *tpcb, struct tcp_server_struct *es) {
	struct pbuf *ptr;

	ptr = es->p;

	tcp_write(tpcb, ptr->payload, ptr->len, 0);

	tcp_output(tpcb);
}

/**
 * @brief  This functions closes the tcp connection
 * @param  tcp_pcb: pointer on the tcp connection
 * @param  es: pointer on _state structure
 * @retval None
 */
static void tcp_server_connection_close(struct tcp_pcb *tpcb,
		struct tcp_server_struct *es) {

	/* remove all callbacks */
	tcp_arg(tpcb, NULL);
	tcp_sent(tpcb, NULL);
	tcp_recv(tpcb, NULL);
	tcp_err(tpcb, NULL);
	tcp_poll(tpcb, NULL, 0);

	/* delete es structure */
	if (es != NULL) {
		mem_free(es);
	}

	/* close tcp connection */
	tcp_close(tpcb);
}

/* Handle the incoming TCP Data */

static void tcp_server_handle(struct tcp_pcb *tpcb,
		struct tcp_server_struct *es) {
	esTx = es;
	pcbTx = tpcb;

}

void send_information(char* z) {

	esTx->p = pbuf_alloc(PBUF_RAW, 1, PBUF_POOL);
	if (esTx->p != NULL) {
		pbuf_take(esTx->p, z, 1);
		tcp_server_send(pcbTx, esTx);
		pbuf_free(esTx->p);
	}

}
