/*
 * Copyright (c) 2013, IHMC
 * All rights reserved.

 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name "IHMC" may not be used to endorse or promote
 *    products derived from this software without prior written permission
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pcap.h>
#include <errno.h>
#include <signal.h>
#include <arpa/inet.h>
#include <sys/time.h>

#include "ethernet.h"
#include "ip.h"
#include "tcp.h"

#include <inttypes.h>
#include "hiredis/hiredis.h"

/* Set to the frequency to print stats, in seconds. Set to zero to disable */
#define UPDATE_REDIS_EVERY_X_SECONDS 1

/* default snap length (maximum bytes per packet to capture) */
#define SNAP_LEN 1518

#define UDP_HEADER_LENGTH 8

/* Redis key for storing the current byte usage */
static const char REDIS_UPLINK_KEY[] = "vrc/bytes/current/uplink";
static const char REDIS_DOWNLINK_KEY[] = "vrc/bytes/current/downlink";

struct in_addr ocuIP; /* OCU IP */
uint32_t totalPackets = 0;

uint32_t ipPacketsToOCU = 0;
uint32_t ipPacketsFromOCU = 0;

uint32_t tcpPacketsToOCU = 0;
uint32_t udpPacketsToOCU = 0;
uint32_t otherPacketsToOCU = 0;

uint32_t tcpPacketsFromOCU = 0;
uint32_t udpPacketsFromOCU = 0;
uint32_t otherPacketsFromOCU = 0;

uint64_t totalLength = 0;

uint64_t totalDataBytesToOCU = 0;
uint64_t totalDataBytesFromOCU = 0;

redisContext *db; /* Redis database */


#if UPDATE_REDIS_EVERY_X_SECONDS
uint64_t lastPrintTime = 0;
uint64_t getTimeInMilliSeconds()
{
	struct timeval tv;
	gettimeofday(&tv, NULL );
	return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}
#endif

void print_stats()
{
	printf("\n\nCaptured %d packets\n", totalPackets);
	printf(" * %d IP packets sent from OCU\n", ipPacketsFromOCU);
	printf("   * %d TCP packets\n", tcpPacketsFromOCU);
	printf("   * %d UDP packets\n", udpPacketsFromOCU);
	printf("   * %d other packets\n", otherPacketsFromOCU);
	printf(" * %d IP packets sent to OCU\n", ipPacketsToOCU);
	printf("   * %d TCP packets\n", tcpPacketsToOCU);
	printf("   * %d UDP packets\n", udpPacketsToOCU);
	printf("   * %d other packets\n", otherPacketsToOCU);

	printf("\n");

	printf("Number of total bytes sent: %lld bytes\n", (long long) totalLength);
	printf("Number of data bytes sent: %lld bytes\n",
			(long long) (totalDataBytesFromOCU + totalDataBytesToOCU));
	printf(" * %lld uplink bytes from OCU\n",
			(long long) totalDataBytesFromOCU);
	printf(" * %lld downlink bytes to OCU\n",
			(long long) totalDataBytesToOCU);
}

void count_bits(u_char *args, const struct pcap_pkthdr *header,
		const u_char *packet)
{
	const struct ip_header *ip; /* The IP header */
	const struct tcp_header *tcp; /* The TCP header */
	int size_ip = 0;
	int total_header_length = 0;
	int protocol_header_length = 0;
	uint8_t fromOCUComputer;
#if UPDATE_REDIS_EVERY_X_SECONDS
	uint64_t currentTime;
	redisReply *reply;
#endif

	/*
	 * Increase totalPackets and total byte count
	 */
	totalPackets++;
	totalLength += header->len;

	/* Get ethernet header size */
	total_header_length += SIZE_ETHERNET;

	/* Get IP header size */
	ip = (struct ip_header*) (packet + SIZE_ETHERNET);
	size_ip = IP_HL(ip) * 4;
	if (size_ip < 20)
	{
		/* Not an IPv4 packet, ignore */
		return;
	}

	/* This is an IP Packet, add to number of IP packets */
	total_header_length += size_ip;

	if (ip->ip_src.s_addr == ocuIP.s_addr)
	{
		fromOCUComputer = 1;
		ipPacketsFromOCU++;
	}
	else if (ip->ip_dst.s_addr == ocuIP.s_addr)
	{
		fromOCUComputer = 0;
		ipPacketsToOCU++;
	}
	else
	{
		printf("Invalid src or destination");
	}

	/* Determine protocol */
	switch (ip->ip_p)
	{
	case IPPROTO_TCP:
		/* TCP Header length is not always the same, dissect packet and get the actual header length */
		tcp = (struct tcp_header*) (packet + SIZE_ETHERNET + size_ip);
		protocol_header_length = TH_OFF(tcp) * 4;
		if (protocol_header_length < 20)
		{
			/* invalid header length, count header as data */
			protocol_header_length = 0;
		}
		else
		{
			if (fromOCUComputer)
			{
				tcpPacketsFromOCU++;
			}
			else
			{
				tcpPacketsToOCU++;
			}
		}
		break;
	case IPPROTO_UDP:
		/* UDP header is always 8 bytes */
		protocol_header_length = UDP_HEADER_LENGTH;
		if (fromOCUComputer)
		{
			udpPacketsFromOCU++;
		}
		else
		{
			udpPacketsToOCU++;
		}
		break;
	default:
		/* For all other protocols, count all bits */
		protocol_header_length = 0;
		if (fromOCUComputer)
		{
			otherPacketsFromOCU++;
		}
		else
		{
			otherPacketsToOCU++;
		}
		break;
	}

	/*
	 * Increase totalDataBytes by the payload size
	 */
	total_header_length += protocol_header_length;

	if (fromOCUComputer)
	{
		totalDataBytesFromOCU += (header->len - total_header_length);
	}
	else
	{
		totalDataBytesToOCU += (header->len - total_header_length);
	}
	
	/* Update Redis */
#if UPDATE_REDIS_EVERY_X_SECONDS
	currentTime = getTimeInMilliSeconds();
	if ((currentTime - lastPrintTime) > (UPDATE_REDIS_EVERY_X_SECONDS * 1000))
	{
		print_stats();
		lastPrintTime = currentTime;

	/* Save the bandwidth numbers into redis*/

    	reply = redisCommand(db,"SET %s %" PRIu64 "", REDIS_DOWNLINK_KEY, totalDataBytesToOCU);
    	//printf("SET (binary API): %s\n", reply->str);
    	freeReplyObject(reply);

    	reply = redisCommand(db,"SET %s %" PRIu64 "", REDIS_UPLINK_KEY, totalDataBytesFromOCU);
    	//printf("SET (binary API): %s\n", reply->str);
    	freeReplyObject(reply);
	}
#endif
}

void quit()
{
	print_stats();
	exit(0);
}

int main(int argc, char *argv[])
{
	char *dev = NULL; 				/* Capture device name */
	char errbuf[PCAP_ERRBUF_SIZE];  /* Error buffer */
	pcap_t *handle; 				/* Capture handle */

	char *filter_exp; /* Filter expression */
	struct bpf_program fp; /* Compiled filter expression */

	bpf_u_int32 mask; /* Subnet mask */
	bpf_u_int32 net; /* Net address */

	redisReply *reply; /* Used for executing commands */


	if (argc != 3)
	{
		printf("Usage: bitCounter [device] [ocuIP]\n");
		dev = pcap_lookupdev(errbuf);
		if (dev == NULL )
		{
			fprintf(stderr, "Couldn't find default device: %s\n", errbuf);
			return (EXIT_FAILURE);
		}
		printf("Default device: %s\n", dev);

		return (EXIT_FAILURE);
	}

	dev = argv[1];
	inet_aton(argv[2], &ocuIP);

	filter_exp = malloc(strlen(argv[2]) + 10);

	sprintf(filter_exp, "ip host %s", inet_ntoa(ocuIP));

	// Connect to Redis DB
	db = redisConnect((char*)"127.0.0.1", 6379);
    	if (db == NULL || db->err) {
        	if (db) {
            		printf("Connection error: %s\n", db->errstr);
            		redisFree(db);
        	} else {
            		printf("Connection error: can't allocate redis context\n");
        	}
        	exit(1);
    	}

	/* Query netmask and ip address from interface */
	if (pcap_lookupnet(dev, &net, &mask, errbuf) == -1)
	{
		fprintf(stderr, "Couldn't get netmask for device %s: %s\n", dev,
				errbuf);
		net = 0;
		mask = 0;
	}

	/* Open network device for capture in normal mode */
	handle = pcap_open_live(dev, SNAP_LEN, 0, 1000, errbuf);
	if (handle == NULL )
	{
		fprintf(stderr, "Couldn't open device %s: %s\n", dev, errbuf);
		return (EXIT_FAILURE);
	}

	/* Check if it is an Ethernet device*/
	if (pcap_datalink(handle) != DLT_EN10MB)
	{
		fprintf(stderr, "%s is not an Ethernet device\n", dev);
		exit(EXIT_FAILURE);
	}

	/* Compile the filter expression */
	if (pcap_compile(handle, &fp, filter_exp, 0, net) == -1)
	{
		fprintf(stderr, "Couldn't parse filter %s: %s\n", filter_exp,
				pcap_geterr(handle));
		exit(EXIT_FAILURE);
	}

	/* Apply filter */
	if (pcap_setfilter(handle, &fp) == -1)
	{
		fprintf(stderr, "Couldn't install filter %s: %s\n", filter_exp,
				pcap_geterr(handle));
		exit(EXIT_FAILURE);
	}

	printf("Using device: %s\n", dev);
	printf("OCU IP: %s\n", inet_ntoa(ocuIP));

	// Reset the accounting
	reply = redisCommand(db,"SET %s %" PRIu64 "", REDIS_UPLINK_KEY, 0);
    	//printf("SET (binary API): %s\n", reply->str);
    	freeReplyObject(reply);

	reply = redisCommand(db,"SET %s %" PRIu64 "", REDIS_DOWNLINK_KEY, 0);
	//printf("SET (binary API): %s\n", reply->str);
	freeReplyObject(reply);

	signal(SIGINT, quit);	

	pcap_loop(handle, -1, count_bits, NULL );

	pcap_close(handle);
	return (0);
}
