#include <stdio.h>
#include <time.h>

//#include "struct.h"

#define HELLO 0b10000000 //128
#define DATA   0b00001000 //8
#define ACK   0b10001000

void tx_file_open(FILE* fp, char* filename){
    if((fp = fopen(filename,"w")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    fprintf(fp,"backoff_start_time,backoff_value,tx_time,,Type,srcAddr,destAddr,length,seq,tx_checksum,,srcHop,destHop\n");
    fclose(fp);
}

void output_data_csv_backoff_time(FILE* fp, char* filename,time_t start_backoff_time,double backoff){
    if((fp = fopen(filename,"w")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    char buf[128];
    struct tm *ptm;
    ptm = localtime(&start_backoff_time);
    strftime(buf,sizeof(buf),"%Y/%m/%d %H:%M:%S",ptm);
	fprintf(fp,"%s,%lf,",buf,backoff);
	fclose(fp);
}
void output_data_csv_time_now(FILE* fp,char* filename,time_t now){
	if((fp = fopen(filename,"w")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    char buf[128];
    struct tm *ptm;
    ptm = localtime(&now);
    strftime(buf,sizeof(buf),"%Y/%m/%d %H:%M:%S",ptm);
    fprintf(fp,"%s",buf);
    fclose(fp);
}

char* judge_packet_type_tx(mac_frame_header_t* hdr,char* packet_type){
    if(hdr->type == HELLO){
        strcpy(packet_type,"HELLO");
        return  packet_type;
    }
    else if(hdr->type == DATA){
        strcpy(packet_type,"DATA");
        return  packet_type;
    }else if(hdr->type == ACK){
        strcpy(packet_type,"ACK");
        return packet_type;
    } 
    return NULL;
}

void output_data_csv_tx_hdr(FILE* fp,mac_frame_header_t* hdr,char* filename){
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    char type[20];
    char srcaddr[20];
    char destaddr[20];
    sprintf(srcaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->SourceAddr[0],hdr->SourceAddr[1],hdr->SourceAddr[2],hdr->SourceAddr[3],hdr->SourceAddr[4],hdr->SourceAddr[5]);
    sprintf(destaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->DestAddr[0],hdr->DestAddr[1],hdr->DestAddr[2],hdr->DestAddr[3],hdr->DestAddr[4],hdr->DestAddr[5]);
    
    fprintf(fp,"%s,%s,%s,%u,%u,%u,,"
    ,judge_packet_type(hdr,type),srcaddr,destaddr,hdr->len,hdr->seqNum,hdr->checksum
    );
    fclose(fp);
}

