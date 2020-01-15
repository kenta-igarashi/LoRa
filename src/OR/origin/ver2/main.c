/*******************************************************************************
 *
 * Copyright (c) 2018 Dragino
 *
 * http://www.dragino.com
 *
 *******************************************************************************/

#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <sys/time.h>
#include <signal.h>
#include <stdlib.h>
#include <time.h>

#include <sys/ioctl.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
//mac address取得のためにソケットを開く
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <unistd.h>

#include <sys/time.h>
#include <math.h>
//#include "csv.h"
//#include "struct.h"
// #############################################
// #############################################

#define REG_FIFO                    0x00
#define REG_OPMODE                  0x01
#define REG_FIFO_ADDR_PTR           0x0D
#define REG_FIFO_TX_BASE_AD         0x0E
#define REG_FIFO_RX_BASE_AD         0x0F
#define REG_RX_NB_BYTES             0x13
#define REG_FIFO_RX_CURRENT_ADDR    0x10
#define REG_IRQ_FLAGS               0x12
#define REG_DIO_MAPPING_1           0x40
#define REG_DIO_MAPPING_2           0x41
#define REG_MODEM_CONFIG            0x1D
#define REG_MODEM_CONFIG2           0x1E
#define REG_MODEM_CONFIG3           0x26
#define REG_SYMB_TIMEOUT_LSB  		0x1F
#define REG_PKT_SNR_VALUE			0x19
#define REG_PAYLOAD_LENGTH          0x22
#define REG_IRQ_FLAGS_MASK          0x11
#define REG_MAX_PAYLOAD_LENGTH 		0x23
#define REG_HOP_PERIOD              0x24
#define REG_SYNC_WORD				0x39
#define REG_VERSION	  				0x42

#define PAYLOAD_LENGTH              0x40

// LOW NOISE AMPLIFIER
#define REG_LNA                     0x0C
#define LNA_MAX_GAIN                0x23
#define LNA_OFF_GAIN                0x00
#define LNA_LOW_GAIN		    	0x20

#define RegDioMapping1                             0x40 // common
#define RegDioMapping2                             0x41 // common

#define RegPaConfig                                0x09 // common
#define RegPaRamp                                  0x0A // common
#define RegPaDac                                   0x5A // common

#define SX72_MC2_FSK                0x00
#define SX72_MC2_SF7                0x70
#define SX72_MC2_SF8                0x80
#define SX72_MC2_SF9                0x90
#define SX72_MC2_SF10               0xA0
#define SX72_MC2_SF11               0xB0
#define SX72_MC2_SF12               0xC0

#define SX72_MC1_LOW_DATA_RATE_OPTIMIZE  0x01 // mandated for SF11 and SF12

// sx1276 RegModemConfig1
#define SX1276_MC1_BW_125                0x70
#define SX1276_MC1_BW_250                0x80
#define SX1276_MC1_BW_500                0x90
#define SX1276_MC1_CR_4_5            0x02
#define SX1276_MC1_CR_4_6            0x04
#define SX1276_MC1_CR_4_7            0x06
#define SX1276_MC1_CR_4_8            0x08

#define SX1276_MC1_IMPLICIT_HEADER_MODE_ON    0x01

// sx1276 RegModemConfig2
#define SX1276_MC2_RX_PAYLOAD_CRCON        0x04

// sx1276 RegModemConfig3
#define SX1276_MC3_LOW_DATA_RATE_OPTIMIZE  0x08
#define SX1276_MC3_AGCAUTO                 0x04

// preamble for lora networks (nibbles swapped)
#define LORA_MAC_PREAMBLE                  0x34

#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG1 0x0A
#ifdef LMIC_SX1276
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x70
#elif LMIC_SX1272
#define RXLORA_RXMODE_RSSI_REG_MODEM_CONFIG2 0x74
#endif

// FRF
#define        REG_FRF_MSB              0x06
#define        REG_FRF_MID              0x07
#define        REG_FRF_LSB              0x08

#define        FRF_MSB                  0xD9 // 868.1 Mhz
#define        FRF_MID                  0x06
#define        FRF_LSB                  0x66

// ----------------------------------------
// Constants for radio registers
#define OPMODE_LORA      0x80
#define OPMODE_MASK      0x07
#define OPMODE_SLEEP     0x00
#define OPMODE_STANDBY   0x01
#define OPMODE_FSTX      0x02
#define OPMODE_TX        0x03
#define OPMODE_FSRX      0x04
#define OPMODE_RX        0x05
#define OPMODE_RX_SINGLE 0x06
#define OPMODE_CAD       0x07

// ----------------------------------------
// Bits masking the corresponding IRQs from the radio
#define IRQ_LORA_RXTOUT_MASK 0x80
#define IRQ_LORA_RXDONE_MASK 0x40
#define IRQ_LORA_CRCERR_MASK 0x20
#define IRQ_LORA_HEADER_MASK 0x10
#define IRQ_LORA_TXDONE_MASK 0x08
#define IRQ_LORA_CDDONE_MASK 0x04
#define IRQ_LORA_FHSSCH_MASK 0x02
#define IRQ_LORA_CDDETD_MASK 0x01

// DIO function mappings                D0D1D2D3
#define MAP_DIO0_LORA_RXDONE   0x00  // 00------
#define MAP_DIO0_LORA_TXDONE   0x40  // 01------
#define MAP_DIO1_LORA_RXTOUT   0x00  // --00----
#define MAP_DIO1_LORA_NOP      0x30  // --11----
#define MAP_DIO2_LORA_NOP      0xC0  // ----11--

//----------------------------------------------
//ハローかパケットかの判別
#define HELLO 0b10000000 //128
#define DATA   0b00001000 //8
#define ACK   0b10001000

#define USER_DATA_FRAME_LEN 1500
#define USER_CTRL_FRAME_LEN 100
#define MAX_PAYLOAD_LEN 255

// #############################################
// #############################################
//



typedef bool boolean;
typedef unsigned char byte;

static const int CHANNEL = 0;

char message[256];

bool sx1272 = true;

byte receivedbytes;

//enum sf_t { SF7=7, SF8, SF9, SF10, SF11, SF12 };
//SF6を追加　
//implicitモードをオンにしなければいけない
//DetectionOptimizeのあたいを0x05にする
//
enum sf_t { SF6=6, SF7, SF8, SF9, SF10, SF11, SF12 };
/*******************************************************************************
 *
 * Configure these values!
 *
 *******************************************************************************/

// SX1272 - Raspberry connections
int ssPin = 6;
int dio0  = 7;
int RST   = 0;

// Set spreading factor (SF7 - SF12)
sf_t sf = SF7;

// Set center frequency
uint32_t  freq = 920000000; // in Mhz! (868.1)  868.1->920.0

//byte hello[32] = "world";
int count=0;
//時間計測の構造体
clock_t sender , receiver;
time_t now,later;
//乱数取得用
struct timeval tv_rand;


//パケットヘッダの構造体
typedef struct{
    uint8_t type;//パケットかデータかの識別
    uint8_t SourceAddr[6]={};
    uint8_t DestAddr[6]={};
    uint8_t len;
    uint16_t seqNum;
    uint16_t checksum;
    uint8_t payload[];
}__attribute__ ((packed)) mac_frame_header_t; //パディング処理

//loraのフレームヘッダの構造体
typedef struct{
    uint8_t Hop;
    uint8_t DestAddr[6];
    uint16_t seqNum;
    //uint8_t payload[];
}__attribute__((packed))mac_frame_payload_t;

typedef struct{
    uint8_t srcHop;//現在のホップ数
    uint8_t destHop;
    //uint8_t DestAddr[6];
    //uint16_t seqNum;
    //unsigned char message[100]; 
    uint8_t message[]; 
}__attribute__((packed))or_data_packet_t;

typedef struct routing_table_entry_struct{
    uint8_t hop;
    uint16_t seq;
    uint8_t addr[6];
    //lifetime
    routing_table_entry_struct* next;
}routing_table_entry_t;

typedef struct{
    routing_table_entry_t* head;
    uint8_t size;
}routing_table_t;

typedef struct backoff_entry_struct{
    uint8_t srcAddr[6];
    uint16_t seqNum;
    double backoff;
    time_t backoff_now;
    backoff_entry_struct* next;
}backoff_entry_t;

typedef struct{
    backoff_entry_t* head;
    uint8_t size;
}backoff_t;
    
typedef struct packet_table_entry_struct{
    uint8_t srcAddr[6];
    uint16_t seq;
    uint8_t flag;
    packet_table_entry_struct* next;
    mac_frame_header_t* packet;
}packet_table_entry_t;

typedef struct{
    packet_table_entry_t* head;
    uint8_t size;
}packet_table_t;


/*固定長の配列を宣言しないとバイナリがバグる
 * uint8_tかunsigned charの１バイトで宣言
 * 255はLoRaの最大ペイロードサイズ
 */
uint8_t Hello[255] = {};
uint8_t data[255] = {};
uint8_t Ack[255] = {};

mac_frame_header_t* data_packet = (mac_frame_header_t*)&data;
mac_frame_header_t *Hello_p=(mac_frame_header_t*)&Hello;
mac_frame_header_t* Ack_p = (mac_frame_header_t*)&Ack;


routing_table_t* r_table = NULL;
backoff_t* bo_st = NULL;
packet_table_t* p_table = NULL;

//file open
FILE *fp;
char* rx_filename;
char* tx_filename;
//char* save_data_rx = {"result_rx.csv"}
//自分のMACアドレス
uint8_t my_addr[6]={};

//チェックサム用
uint16_t prev_checksum;
uint16_t current_checksum;

//現在時刻の取得
struct timespec ts_tx,ts_rx;
struct tm tm_rx,tm_tx;
time_t after_backoff;

/*ノードの固有値
 * LoRaの物理層から取得はできないため、有線LANのものを取得して使う
*/
//struct ifreq ifr;
void get_my_mac_addr(){
    int fd;
    struct ifreq ifr;

    fd = socket(AF_INET,SOCK_DGRAM,0);

    ifr.ifr_addr.sa_family = AF_INET;
    strncpy(ifr.ifr_name,"eth0",IFNAMSIZ-1);

   
    ioctl(fd,SIOCGIFHWADDR,&ifr);

    for(int i=0;i<6;i++){
	    my_addr[i] = (unsigned char)ifr.ifr_hwaddr.sa_data[i];
    }
    close(fd);
    //return (my_addr*);
}

void mac_print_addr(uint8_t* addr){
	printf("mac addr:%02x:%02x:%02x:%02x:%02x:%02x\n",addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]);
}

void mac_set_addr(uint8_t*original,uint8_t* target){
    for(int i = 0; i < 6; i++) target[i]=original[i];
}

void mac_set_bc_addr(uint8_t* target){
    for(int i = 0; i < 6; i++) target[i] = 0xff;    
}


/*
 * frame header init function
 */
 
void mac_tx_hello_frame_header_init(mac_frame_header_t* hdr){
    hdr->type = HELLO;
    mac_set_bc_addr(hdr->SourceAddr);
    mac_set_bc_addr(hdr->DestAddr);
    hdr->len = 0;// hdr + payload
    hdr->seqNum = 0;
    hdr->checksum = 0;
}

void mac_tx_ack_frame_header_init(mac_frame_header_t* hdr){
    hdr->type = ACK;
    mac_set_bc_addr(hdr->SourceAddr);
    mac_set_bc_addr(hdr->DestAddr);
    hdr->len = 0;// hdr + payload
    hdr->seqNum = 0;
    hdr->checksum = 0;
}

void mac_tx_data_frame_header_init(mac_frame_header_t* hdr){
    hdr->type = DATA;
    mac_set_bc_addr(hdr->SourceAddr);
    mac_set_bc_addr(hdr->DestAddr);
    hdr->len = 0;
    hdr->seqNum = 0;
    hdr->checksum = 0;
}

void mac_tx_hello_frame_payload_init(mac_frame_header_t* hdr){
    mac_frame_payload_t* payload = (mac_frame_payload_t*)hdr->payload;
    payload->Hop= 0;
    mac_set_bc_addr(payload->DestAddr);
    payload->seqNum = 0;
}
void mac_tx_data_frame_payload_init(mac_frame_header_t* frame){
    or_data_packet_t* data_frame = (or_data_packet_t*)frame->payload;
    data_frame->srcHop = 0;
    data_frame->destHop = 0;
    //mac_set_bc_addr(data_frame->DestAddr);
    //data_frame->seqNum = 0;
    //memset(&(data_frame->message),0,sizeof(message));
    for(unsigned int i = 0;i < 255 - sizeof(mac_frame_header_t) - sizeof(or_data_packet_t);i++){
        data_frame->message[i] = 0x11;
    }
}

void print_mac_frame_header(mac_frame_header_t* data){
    printf("mac packet header   : \n");
    if(data->type == HELLO) printf("type: HELLO\n");
    else if(data->type == DATA) printf("type                : DATA\n");
    else if(data->type == ACK) printf("type                : ACK\n");
    //printf("type: %u\n",data->type);
    printf("Source      ");
    mac_print_addr(data->SourceAddr);
    printf("Destination ");
    mac_print_addr(data->DestAddr);
    printf("length              : %u\n",data->len);
    printf("sequence number     : %u\n",data->seqNum);
    printf("\n");
}

void print_mac_frame_payload(mac_frame_header_t* data){
    mac_frame_payload_t* packet = (mac_frame_payload_t*)data->payload;
    //byte* p = (byte*)packet;
    printf("mac frame payload: \n");
    printf("Hop: %u\n",packet->Hop);
    printf("Destination ");
    mac_print_addr(packet->DestAddr);
    printf("Sequence number: %u\n",packet->seqNum);
    printf("\n");
    /*
    for(int i = 0 ;i<10;i++){
        printf("%02x ",p[i]);
    }
    printf("\n");
    */
}

boolean is_smaller_addr(uint8_t* origin,uint8_t* target){
    for(int i = 0 ;i<6;i++){
        if(origin[i]<target[i]){ //addr1=origin 
            return true;
        }else if(origin[i]>target[i]){
            return false;
        }
    }
    return false;
}

boolean is_same_addr(uint8_t* origin,uint8_t* target){
    for(int i =0;i<6;i++){
        if(origin[i]!=target[i]) return false;
    }
    return true;
}

void delete_routing_table(uint8_t* addr){
    routing_table_entry_t* toFree = NULL;
    routing_table_entry_t* current = r_table->head;
    
    if(!current){
        //no entry in routing table 
    }
    else if(is_same_addr(current->addr,addr)){
        toFree = r_table->head;
        current->next = toFree->next;
        free(toFree);
        --r_table->size;
    }
    else {
        while(current->next && is_smaller_addr(current->addr,addr)){
            current = current->next;
        }
    }
    if(current->next && is_same_addr(current->next->addr,addr)){
        toFree = current->next;
        current->next = toFree->next;
        free(toFree);
        --r_table->size;
    }
}

void insert_routing_table(uint8_t* addr,uint8_t hop,uint8_t seq){

    routing_table_entry_t* current = r_table->head;
    routing_table_entry_t* previous = NULL;
    
    printf("tes\n");
    
    // current
    //while(current && current->next && is_smaller_addr((current->next)->addr, addr))
    //current = current->next;
    while(current && is_smaller_addr(current->addr,addr)){ //current!=NULL && is same addr
    //while(current ){ //current!=NULL && is same addr
        printf("tes1\n");
        previous = current;
        current = current->next;
    }

    
//    if(!current){//routing tableがまだない場合 current==NULL
        if(0){
        //if(!r_table->head){ //r_table->head == NULL
      //insert  
        printf("new entry\n");
        routing_table_entry_t* new_entry = (routing_table_entry_t* )malloc(sizeof(routing_table_entry_t));
        memset(new_entry, 0, sizeof(routing_table_entry_t));
        mac_set_addr(addr, new_entry->addr);
        new_entry->hop = hop;
        new_entry->seq = seq;
        new_entry->next = NULL;
        // new_entry->lifetime;
        (r_table)->size++;
        
        //if(r_table->head){
            // size = 0;
            r_table->head = new_entry;
//        }else{
            // size != 0 && saigo no youso
//            current->next = new_entry;
 //       }            
        //}
    }/*else if(current && is_same_addr(current->addr,my_addr)){
        printf("自身なのでルーティングテーブルに格納しません.\n");
        //delete_routing_table(my_addr);
        return;
        
    }*/
    // else if(current->addr == addr){
    else if(is_same_addr(my_addr,addr)){
        printf("自身なのでルーティングテーブルにいれません.\n");
    }
    else if(current && is_same_addr(my_addr,addr)){
        printf("自身なのでルーティングテーブルにいれません.\n");
    }
    else if(current && is_same_addr(current->addr,addr)){
        //update
        if(current->seq < seq){
            printf("tes3\n");
            current->hop = hop;
            current->seq = seq;
            mac_set_addr(addr,current->addr);
        }
        else if(current->seq==seq && current->hop > hop){
            //updata
            printf("tes4\n");
            current->hop = hop;
            current->seq = seq;
            mac_set_addr(addr,current->addr);
        }
    }
    else{
        //insert
        printf("tes5\n");
        routing_table_entry_t* new_entry = (routing_table_entry_t* )malloc(sizeof(routing_table_entry_t));
        memset(new_entry, 0, sizeof(routing_table_entry_t));

        mac_set_addr(addr, new_entry->addr);
        new_entry->hop = hop;
        new_entry->seq = seq;
        // new_entry->lifetime;
        new_entry->next = NULL;
        (r_table)->size++;
        
        /*
        if(!current){
            printf("tes8\n");
            previous->next = new_entry;
        }
        else if(!current->next){ //current->next==NULL
            printf("tes6\n");
            current->next = new_entry;
           //new_entry->next = NULL;
           //exit(1);
        }
        else{
            printf("tes7\n");
            previous->next = new_entry;
            new_entry->next = current;
        }*/
        if(!previous){
            new_entry->next = r_table->head;
            r_table->head = new_entry;            
        }else{
            new_entry->next = previous->next;
            previous->next = new_entry;
        }
    }
    return ;
} 

int get_routing_table(mac_frame_header_t* hello){
    mac_frame_payload_t* current = (mac_frame_payload_t*) hello->payload;
    routing_table_entry_t* current_entry = r_table->head;
    int i =0;
    while(current_entry){
        current->Hop = current_entry->hop;
        current->seqNum  = current_entry->seq;
        //current->DestAddr = current_entry->addr;
        mac_set_addr(current_entry->addr,current->DestAddr);
        //current = current++;
        ++current;
        i++;
        current_entry = current_entry->next;
    }
    /*
    for(int j = 0;j<i;j++){
        printf("%02x",current[j]);
    }*/
    printf("\n");
   return i;
}

void output_data_csv_rx_time(char* filename){
    char time[50];
    
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    sprintf(time,"%d/%02d/%02d %02d:%02d:%02d",tm_rx.tm_year+1900,tm_rx.tm_mon+1,tm_rx.tm_mday,tm_rx.tm_hour,tm_rx.tm_min,tm_rx.tm_sec);
    fprintf(fp,"%s,%09ld,",time,ts_rx.tv_nsec);
    fclose(fp);
}
void output_data_csv_tx_time(char* filename){
    char time[50];
    
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    sprintf(time,"%d/%02d/%02d %02d:%02d:%02d",tm_tx.tm_year+1900,tm_tx.tm_mon+1,tm_tx.tm_mday,tm_tx.tm_hour,tm_tx.tm_min,tm_tx.tm_sec);
    fprintf(fp,"%s,%09ld,",time,ts_tx.tv_nsec);
    fclose(fp);
}
void output_data_csv(routing_table_entry_t* head, char* filename){
    char address[20];
    
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    sprintf(address,"%02x:%02x:%02x:%02x:%02x:%02x",head->addr[0],head->addr[1],head->addr[2],head->addr[3],head->addr[4],head->addr[5]);
    
    fprintf(fp,"%s,%u,%u,",address,head->hop,head->seq);
    
    /*
    fprintf(fp,"%d,%02d,%02d,%02d,%02d,%02d,%09ld,%02x,%02x,%02x,%02x,%02x,%02x,%u,%u\n"
    ,tm_rx.tm_year+1900,tm_rx.tm_mon+1,tm_rx.tm_mday,tm_rx.tm_hour,tm_rx.tm_min,tm_rx.tm_sec,ts.tv_nsec
    ,head->addr[0],head->addr[1],head->addr[2],head->addr[3],head->addr[4],head->addr[5]
    ,head->hop,head->seq
    );
    * */
    fclose(fp);
}

void output_data_csv_space(char* filename){
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    fprintf(fp,"\n");
    fclose(fp);
}

void print_routing_table(routing_table_t* routing_t){
    routing_table_entry_t* current =routing_t->head;
    printf("----------routing table----------\n");
    //routing_table_entry_t* current = r_table->head;
    int i =1;
    if(!current){ //current==NULL
        printf("data nothing\n");
        return ;
    }
    //output_data_csv_rx_time(rx_filename);
    while(current){//current!=NULL
        printf("Node%d ",i);
        mac_print_addr(current->addr);
        printf("Hop: %u\n",current->hop);
        printf("Sequence Number: %u\n",current->seq);
        printf("table size: %d\n",routing_t->size);
        i++;
        //output_data_csv(current,rx_filename);
        
        current = current->next;
    }
    //output_data_csv_space(rx_filename);
    printf("---------------end----------------\n");
    
}

void print_data_frame(mac_frame_header_t* packet){
    or_data_packet_t* data_frame = (or_data_packet_t*)packet->payload;
    printf("data: \n");
    printf("src hop           : %u\n",data_frame->srcHop);
    printf("dest hop          : %u\n",data_frame->destHop);
    //printf("dest    ");
    //mac_print_addr(data_frame->DestAddr);
    //printf("suequence number: %u\n",data_frame->seqNum);
}

//void add_routing_table(routing_table_t* routing_t){
void add_routing_table(mac_frame_header_t* hdr){
    int len = ((hdr->len) - sizeof(mac_frame_header_t) )/sizeof(mac_frame_payload_t);
    printf("insert routing table length: %d\n",len);
    //routing_table_entry_t* entry = (routing_table_entry_t*)hdr->payload;
    mac_frame_payload_t* entry = (mac_frame_payload_t*)hdr->payload;
    
    //routing_table_entry_t* current =routing_t->head;
    //printf("routing table is:\n");
    //routing_table_entry_t* current = r_table->head;
    /*
    if(!current){ //current==NULL
        printf("data nothing\n");
        return ;
    }
    * */
    if(len == 0){ 
        return;
    }else{
    //int i = 0;
        //while(i < len){
        for(int i = 0;i < len;i++){
            insert_routing_table(entry->DestAddr,entry->Hop+1,entry->seqNum);
            mac_print_addr(entry->DestAddr);
            ++entry;
            
        }
    }
    /*
    while(current){//current!=NULL
        insert_routing_table(current->addr,current->hop+1,current->seq);
        current = current->next;
    }*/
    
}

boolean is_smaller_backoff(double origin,double target){
    if(origin<target)  return true;
    else if(origin>target) return false;
    return false;
}


boolean is_same_seq(uint16_t origin,uint16_t target){
    if(origin!=target) return false;
    return true;
}

void print_packet_table(){
    packet_table_entry_t* current = p_table->head;
    printf("----------packet table----------\n");
    int i =1;
    if(!current){ //current==NULL
        printf("data nothing\n");
        return ;
    }
    while(current){//current!=NULL
        printf("Node%d ",i);
        mac_print_addr(current->srcAddr);
        printf("Sequence Number: %u\n",current->seq);
        printf("flag           : %u\n",current->flag);
        printf("table size     : %d\n",p_table->size);
        i++;
        current = current->next;
    }
    printf("---------------end--------------\n");

}

void print_backoff_table(){
    backoff_entry_t* current = bo_st->head;
    printf("----------backoff table----------\n");
    int i = 1;
    if(!current){
        printf("data nothing\n");
        return;
    }
    while(current){
        printf("Node%d\n",i);
        printf("src ");
        mac_print_addr(current->srcAddr);
        printf("sequence number: %u\n",current->seqNum);
        printf("backoff time   : %lf\n",current->backoff);
        printf("table size     : %d\n",bo_st->size);
        i++;
        current = current->next;
    }
    printf("---------------end---------------\n");
}

char* judge_packet_type(mac_frame_header_t* hdr,char* packet_type){
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

void output_data_csv_hdr(mac_frame_header_t* hdr,char* filename,int pattern){
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    char type[20];
    char srcaddr[20];
    char destaddr[20];
    sprintf(srcaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->SourceAddr[0],hdr->SourceAddr[1],hdr->SourceAddr[2],hdr->SourceAddr[3],hdr->SourceAddr[4],hdr->SourceAddr[5]);
    sprintf(destaddr,"%02x:%02x:%02x:%02x:%02x:%02x",hdr->DestAddr[0],hdr->DestAddr[1],hdr->DestAddr[2],hdr->DestAddr[3],hdr->DestAddr[4],hdr->DestAddr[5]);
    
    fprintf(fp,"%s,%d,%s,%s,%u,%u,%u,%u,,"
    ,judge_packet_type(hdr,type),pattern,srcaddr,destaddr,hdr->len,hdr->seqNum,prev_checksum,current_checksum
    );
    fclose(fp);
}
void output_data_csv_ordata(or_data_packet_t* hdr,char* filename){
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    fprintf(fp,"%u,%u",hdr->srcHop,hdr->destHop);
    fclose(fp);
}

void output_data_csv_tx_hdr(mac_frame_header_t* hdr,char* filename){
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

packet_table_entry_t* insert_packet_table(uint8_t *srcAddr,uint16_t seq,mac_frame_header_t* packet){//,uint8_t flag){
    packet_table_entry_t* current = p_table->head;
    packet_table_entry_t* prev = NULL;

    
    while(current && is_smaller_addr(current->srcAddr,srcAddr)){
        prev = current;
        current = current->next;

    }
    //while(current && is_smaller_seq(current->seq,seq)){
    while(current && is_same_addr(current->srcAddr,srcAddr) && current->seq < seq){
        prev = current;
        current = current->next;
    }

    if(current && is_same_seq(current->seq,seq)){
            //パターン２・・受信済みでACKしてない場合
            //パターン3・・受信済みでACKしてないけど、ACKしない場合
            //パターン4・・受信済みでACK済みの場合
            
        //-----------------------------------------------------
        routing_table_entry_t* r_current = r_table->head;
        while(r_current && !is_same_addr(r_current->addr,packet->DestAddr)){//宛先端末のホップ数を調べる
            r_current = r_current->next;
        }
        or_data_packet_t* data_p = (or_data_packet_t*)packet->payload;
        
        if(current->flag == DATA){
           if(r_current->hop >= data_p->destHop){
                //2..ACKする場合
                //p_entry->flag = ACK;
                current->flag = ACK;
                output_data_csv_hdr(packet,rx_filename,2);
                output_data_csv_ordata(data_p,rx_filename);
            } else{
                //3
                output_data_csv_hdr(packet,rx_filename,3);
                output_data_csv_ordata(data_p,rx_filename);
            }
            
        }else if(current->flag == ACK){
            //4
            output_data_csv_hdr(packet,rx_filename,4);
        }
        //-----------------------------------------------------------
        
        print_packet_table();
        /*
        //update
        printf("data update\n");
        if(current->flag == DATA){
            current->
        }
        */
        output_data_csv_space(rx_filename);
        return current;
    }else{
        //パターン１・・未受信でデータ受信
        //パターン５・・未受信でACK受信
        or_data_packet_t* data_p = (or_data_packet_t*)packet->payload;
        if(packet->type == DATA){
            //1
            output_data_csv_hdr(packet,rx_filename,1);
            output_data_csv_ordata(data_p,rx_filename);
        }
        else{//type==ACK
            //5
            output_data_csv_hdr(packet,rx_filename,5);
        }
        
        //insert
        printf("insert packet table\n");
        //セグメントエラーがデータパケットの中継中に出た
        
        packet_table_entry_t* new_entry = (packet_table_entry_t* )malloc(sizeof(packet_table_entry_t));
       
        memset(new_entry, 0, sizeof(packet_table_entry_t));
        //packet save 要注意！！！！！
        //new_entry->packet = (byte*)malloc((int)packet->len);
        new_entry->packet = (mac_frame_header_t*)malloc((int)packet->len);
        memcpy(new_entry->packet,packet,(int)packet->len);

        mac_set_addr(srcAddr, new_entry->srcAddr);
        new_entry->seq = seq;
        
        if(packet->type == DATA){
            new_entry->flag = DATA ;//1 = data , 2 = ack
        }else if(packet->type == ACK){
            new_entry->flag = ACK;
        }
        
        new_entry->next = NULL;
        (p_table)->size++;

        if(!prev){
            new_entry->next = p_table->head;
            p_table->head = new_entry;            
        }else{
            new_entry->next = prev->next;
            prev->next = new_entry;
        }
        print_packet_table();
        
        output_data_csv_space(rx_filename);
        
        return new_entry;
    }
}

void insert_backoff(uint8_t* srcAddr,uint16_t seq,double backoff,time_t backoff_now){//,uint8_t flag){
    backoff_entry_t* current = bo_st->head;
    backoff_entry_t* previous = NULL;
    while(current && is_smaller_backoff(current->backoff,backoff)){
        previous = current;
        current = current->next;
    }

    //if(1){
        //insert
        backoff_entry_t* new_entry = (backoff_entry_t* )malloc(sizeof(backoff_entry_t));
        memset(new_entry, 0, sizeof(backoff_entry_t));

        new_entry->backoff = backoff;
        mac_set_addr(srcAddr, new_entry->srcAddr);
        new_entry->seqNum = seq;
        new_entry->backoff_now = backoff_now;
        new_entry->next = NULL;
        (bo_st)->size++;
        
        if(!previous){
            new_entry->next = bo_st->head;
            bo_st->head = new_entry;            
        }else{
            new_entry->next = previous->next;
            previous->next = new_entry;
        }
    //}
    print_backoff_table();
    return ;
} 

void dequeue_backoff_table(){
    printf("backoff tableを削除します.\n");
    backoff_entry_t* toFree = NULL;
    
    if(bo_st->head){
       toFree = bo_st->head;
       bo_st->head = bo_st->head->next;
       free(toFree);
       --bo_st->size;
       print_backoff_table();
    }
}


void die(const char *s)
{
    perror(s);
    exit(1);
}

void selectreceiver()
{
    digitalWrite(ssPin, LOW);
}

void unselectreceiver()
{
    digitalWrite(ssPin, HIGH);
}

byte readReg(byte addr)
{
    unsigned char spibuf[2];

    selectreceiver();
    spibuf[0] = addr & 0x7F;
    spibuf[1] = 0x00;
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    unselectreceiver();

    return spibuf[1];
}

void writeReg(byte addr, byte value)
{
    unsigned char spibuf[2];

    spibuf[0] = addr | 0x80;
    spibuf[1] = value;
    selectreceiver();
    wiringPiSPIDataRW(CHANNEL, spibuf, 2);
    
    unselectreceiver();
    //printf("value=%02x\n",value);
    //printf("CR and RW is = %02x\n",readReg(REG_MODEM_CONFIG));
}

static void opmode (uint8_t mode) {
    writeReg(REG_OPMODE, (readReg(REG_OPMODE) & ~OPMODE_MASK) | mode);
}

static void opmodeLora() {
    uint8_t u = OPMODE_LORA;
    if (sx1272 == false)
        u |= 0x8;   // TBD: sx1276 high freq
    writeReg(REG_OPMODE, u);
}


void SetupLoRa()
{
    
    digitalWrite(RST, HIGH);
    delay(100);
    digitalWrite(RST, LOW);
    delay(100);

    byte version = readReg(REG_VERSION);

    if (version == 0x22) {
        // sx1272
        printf("SX1272 detected, starting.\n");
        sx1272 = true;
    } else {
        // sx1276?
        digitalWrite(RST, LOW);
        delay(100);
        digitalWrite(RST, HIGH);
        delay(100);
        version = readReg(REG_VERSION);
        if (version == 0x12) {
            // sx1276
            printf("SX1276 detected, starting.\n");
            sx1272 = false;
        } else {
            printf("Unrecognized transceiver.\n");
            //printf("Version: 0x%x\n",version);
            exit(1);
        }
    }

    opmode(OPMODE_SLEEP);

    // set frequency
    uint64_t frf = ((uint64_t)freq << 19) / 32000000;
    writeReg(REG_FRF_MSB, (uint8_t)(frf>>16) );
    writeReg(REG_FRF_MID, (uint8_t)(frf>> 8) );
    writeReg(REG_FRF_LSB, (uint8_t)(frf>> 0) );

    writeReg(REG_SYNC_WORD, 0x34); // LoRaWAN public sync word

    if (sx1272) {
        if (sf == SF11 || sf == SF12) {
            writeReg(REG_MODEM_CONFIG,0x0B);
        } else {
            writeReg(REG_MODEM_CONFIG,0x0A);
        }
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
    } else {
        if (sf == SF11 || sf == SF12) {
            writeReg(REG_MODEM_CONFIG3,0x0C);
        }else if(sf == SF6){
            /*
            u1_t = mc1 = 0;
            mc1 |= SX1276_MC1_IMPLICIT_HEADER_MODE_ON;
            mc1 |= 0x72;
            * */
             writeReg(REG_MODEM_CONFIG,0x73);
             //RegDetectOptimize
             writeReg(0x31,0xc5);
             //RefDetectionThreshold
             writeReg(0x37,0x0c);
             //
            writeReg(REG_MODEM_CONFIG,0x72);
            writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
        }else {
            writeReg(REG_MODEM_CONFIG3,0x04);
            //
            writeReg(REG_MODEM_CONFIG,0x72);
            writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
        }
        /*
        writeReg(REG_MODEM_CONFIG,0x72);
        writeReg(REG_MODEM_CONFIG2,(sf<<4) | 0x04);
        */
    }

    if (sf == SF10 || sf == SF11 || sf == SF12) {
        writeReg(REG_SYMB_TIMEOUT_LSB,0x05);
    } else {
        writeReg(REG_SYMB_TIMEOUT_LSB,0x08);
    }
    writeReg(REG_MAX_PAYLOAD_LENGTH,0x80);
    writeReg(REG_PAYLOAD_LENGTH,PAYLOAD_LENGTH);
    writeReg(REG_HOP_PERIOD,0xFF);
    writeReg(REG_FIFO_ADDR_PTR, readReg(REG_FIFO_RX_BASE_AD));

    writeReg(REG_LNA, LNA_MAX_GAIN);

}

static void configPower (int8_t pw) {
    if (sx1272 == false) {
        // no boost used for now
        if(pw >= 17) {
            pw = 15;
        } else if(pw < 2) {
            pw = 2;
        }
        // check board type for BOOST pin
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw&0xf)));
        writeReg(RegPaDac, readReg(RegPaDac)|0x4);

    } else {
        // set PA config (2-17 dBm using PA_BOOST)
        if(pw > 17) {
            pw = 17;
        } else if(pw < 2) {
            pw = 2;
        }
        writeReg(RegPaConfig, (uint8_t)(0x80|(pw-2)));
    }
}

void set_txmode(){
    //送信モード
    opmodeLora();
    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);
    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec
    configPower(23);
}

void set_rxmode(){
    //LoRaを受信モードにセット
    //receiverの設定
    SetupLoRa();
    opmodeLora();
    opmode(OPMODE_STANDBY);
    opmode(OPMODE_RX);
}

static void writeBuf(byte addr, byte *value, byte len) {                                                       
    unsigned char spibuf[256];                                                                          
    spibuf[0] = addr | 0x80;                                                                            
    for (int i = 0; i < len; i++) {                                                                         
        spibuf[i + 1] = value[i];                                                                       
    }                                                                                                   
    selectreceiver();                                                                                   
    wiringPiSPIDataRW(CHANNEL, spibuf, len + 1);                                                        
    unselectreceiver();                                                                                 
}

uint16_t checksum(byte *data,int len){
    register uint32_t sum;
    register uint16_t *ptr;
    register int c;
    
    sum = 0;
    ptr = (uint16_t *)data;
    
    for(c = len;c > 1;c -= 2){
        sum += (*ptr);
        if(sum & 0x80000000){
            sum = (sum & 0xFFFF) + (sum >> 16);
        }
        ptr++;
    }
    if(c == 1){
        uint16_t val;
        val = 0;
        memcpy(&val,ptr,sizeof(uint8_t));
        sum += val;
    }
    while(sum >> 16){
        sum = (sum & 0xFFFF) + (sum >> 16);
    }
    return (~sum);
    
}

boolean calc_checksum(mac_frame_header_t* hdr,int len){
    mac_frame_header_t *tmp = hdr;
    uint16_t prev_checksum = 0;
    prev_checksum = hdr->checksum;
    
    tmp->checksum = 0;
    uint16_t current_checksum = checksum((byte*)tmp,len);
    printf("origin(tx):%u\n",prev_checksum);
    printf("rx        :%u\n",current_checksum);
    return current_checksum == prev_checksum;
}
void get_time_now(struct timespec ts ,struct tm tm){
    //struct timespec ts;
    //struct tm tm_tx,tm_rx;
    clock_gettime(CLOCK_REALTIME,&ts);
    localtime_r(&ts.tv_sec,&tm);
    printf("rx time:  %d/%02d/%02d %02d:%02d:%02d.%09ld\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec,ts.tv_nsec);
    //printf("difftime:")
}

void txlora(byte *frame, byte datalen) {

    //checksum function
    ((mac_frame_header_t*)frame)->checksum = 0;//送信するたびに0に初期化しないと値が一致しない
    ((mac_frame_header_t*)frame)->checksum = checksum(frame,(int)datalen);
    printf("checksum: %u\n",((mac_frame_header_t*)frame)->checksum);

    get_time_now(ts_tx,tm_tx);

    // set the IRQ mapping DIO0=TxDone DIO1=NOP DIO2=NOP
    writeReg(RegDioMapping1, MAP_DIO0_LORA_TXDONE|MAP_DIO1_LORA_NOP|MAP_DIO2_LORA_NOP);
    // clear all radio IRQ flags
    writeReg(REG_IRQ_FLAGS, 0xFF);
    // mask all IRQs but TxDone
    writeReg(REG_IRQ_FLAGS_MASK, ~IRQ_LORA_TXDONE_MASK);

    // initialize the payload size and address pointers
    writeReg(REG_FIFO_TX_BASE_AD, 0x00);
    writeReg(REG_FIFO_ADDR_PTR, 0x00);
    writeReg(REG_PAYLOAD_LENGTH, datalen);

    //download buffer to the radio FIFO
    writeBuf(REG_FIFO, frame, datalen);
    // now we actually start the transmission
    opmode(OPMODE_TX);
    
    printf("\n");
    //printf("seq: %d",seqNum);
    //printf("send: %s\n", frame);
    
    //確認
    mac_frame_header_t* p_frame = (mac_frame_header_t*)frame;
    if(p_frame->type == HELLO){
        //print_mac_frame_header(Hello_p);
        //print_mac_frame_payload(Hello_p);
        print_mac_frame_header(p_frame);
        //print_mac_frame_payload(p_frame);
        printf("------------------\n");
    }else if(p_frame->type == DATA){
        print_mac_frame_header(p_frame);
        print_data_frame(p_frame);
        printf("------------------\n");
        output_data_csv_tx_hdr(p_frame,tx_filename);
        output_data_csv_ordata(((or_data_packet_t*)p_frame->payload),tx_filename);
    }else if(p_frame->type == ACK){
        print_mac_frame_header(p_frame);
        printf("------------------\n");
        output_data_csv_tx_hdr(p_frame,tx_filename);
    }
    //printf("length: %d\n",sizeof(Hello));
    

    for(int i = 0;i<datalen;i++){
        printf("%02x ",frame[i]);
    }
    printf("\n");
    
    usleep(250000);
    
}


double sigmoid(double gain, double x){
    return (1.0 / (1.0 + exp(-gain * x)));
}

double OR_calculate_backoff(uint8_t desthop,uint8_t hop){
    //or_data_packet_t* data_p = (mac_frame)
    struct timeval tv;
    gettimeofday(&tv,NULL);
    srand((unsigned int)tv.tv_sec * ((unsigned int)tv.tv_usec + 1));
    double expected = (double)desthop - 1;
    double table = (double)hop;
    double gain = 1.0;
    double alpha = 1.0;
    double beta = 1.0;
    double difference = (table - expected) - alpha;
    
    double base_backoff = 500;//ミリ秒
    double backoff = 0;
    double max_backoff = base_backoff * 2;
    
    double max_random_backoff = (max_backoff) * (sigmoid(gain, difference + beta) - sigmoid(gain,difference));
    backoff += max_backoff * sigmoid(gain,difference);
    printf("difference        :%lf\n",difference);
    printf("max random backoff:%lf\n",max_random_backoff);

    if(max_random_backoff != 0){
        backoff += rand() % (int)max_random_backoff;
    }else{
        backoff += rand() % (10 * 1000);
    }
    printf("backoff           :%lf\n",backoff);
    return backoff;
}

void judge_transfer_data(mac_frame_header_t *packet_p){
    //mac_frame_header_t* packet_p = (mac_frame_header_t*)&packet;
    or_data_packet_t* data_p = (or_data_packet_t*)packet_p->payload;
    routing_table_entry_t* current = r_table->head;
    if(packet_p->type == ACK){
        printf("ACK dataなので転送を中止します.\n");
        insert_packet_table(packet_p->SourceAddr,packet_p->seqNum,packet_p);
        //if(p_entry){//受信済み
        //if(current->hop >= data_p->destHop){
        //p_entry->flag = ACK;
    }
    else if(packet_p->type == DATA){
            print_mac_frame_header(packet_p);
            print_data_frame(packet_p);
            
        //if(is_same_addr(my_addr,packet_p->DestAddr)){//宛先端末が自分の場合
        if(is_same_addr(my_addr,packet_p->DestAddr)){//宛先端末が自分の場合
            
            printf("%s\n",data_p->message);
            //printf("payload message : %s\n",message);
            printf("ACKを送信します.\n");
        
            Ack_p->seqNum = packet_p->seqNum;
            mac_set_addr(packet_p->DestAddr,Ack_p->DestAddr);
            mac_set_addr(packet_p->SourceAddr,Ack_p->SourceAddr);

            mac_print_addr(Ack_p->DestAddr);
            mac_print_addr(Ack_p->SourceAddr);
            printf("seqnum: %u\n",Ack_p->seqNum);
            //送信モード
            set_txmode();
            txlora((byte*)&Ack,(byte)Ack_p->len);
            //sleep(100000);
            //受信モードに切り替え
            set_rxmode();
            
        }else{//自分ではない場合
            printf("transfer data\n");
            
            while(current && !is_same_addr(current->addr,packet_p->DestAddr)){//宛先端末のホップ数を調べる 
                current = current->next;
            }
            if(current){
                packet_table_entry_t* p_entry = insert_packet_table(packet_p->SourceAddr,packet_p->seqNum,packet_p);
                if(p_entry->flag != ACK){//受信済み
               // }
                /*else{//未受信
                    //転送待機時間の算出
                  */  
                    double bo = OR_calculate_backoff(data_p->destHop,current->hop);
                    time(&after_backoff);
                    insert_backoff(packet_p->SourceAddr,packet_p->seqNum,bo,after_backoff);
                }
            }
        }
    }
    else if(packet_p->type == HELLO){
        print_mac_frame_header(packet_p);
        //print_mac_frame_payload(packet_p);
    
        
        insert_routing_table(packet_p->SourceAddr, 1, packet_p->seqNum);
        add_routing_table(packet_p);
        print_routing_table(r_table);
    }
    else{
        printf("ここじゃないよ\n");
    }
}


void output_data_csv_RSSI(int Packet_RSSI,int RSSI,long int SNR,int lora_length,char* filename){
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    fprintf(fp,"%d,%d,%li,%i,,",Packet_RSSI,RSSI,SNR,lora_length);
    fclose(fp);
}

boolean receive(char *payload) {
    // clear rxDone
    writeReg(REG_IRQ_FLAGS, 0x40);

    int irqflags = readReg(REG_IRQ_FLAGS);
    
    //  payload crc: 0x20
    if((irqflags & 0x20) == 0x20)
    {
        printf("CRC error\n");
        writeReg(REG_IRQ_FLAGS, 0x20);
        return false;
    } else {

        byte currentAddr = readReg(REG_FIFO_RX_CURRENT_ADDR);
        byte receivedCount = readReg(REG_RX_NB_BYTES);
        receivedbytes = receivedCount;

        writeReg(REG_FIFO_ADDR_PTR, currentAddr);

        for(int i = 0; i < receivedCount; i++)
        {
            payload[i] = (char)readReg(REG_FIFO);
        }
    }
    return true;
}

void receivepacket() {

    long int SNR;
    int rssicorr;
    
    //printf("%x ",digitalRead(dio0));
    //printf("%d\n",readReg(REG_IRQ_FLAGS));
    
    if(digitalRead(dio0) == 1)
    {
        if(receive(message)) {
            mac_frame_header_t* p = (mac_frame_header_t*)message;
            get_time_now(ts_rx,tm_rx);
            output_data_csv_rx_time(rx_filename);
            time(&after_backoff);
            //checksum function
            
                for(int i = 0;i < (int)receivedbytes;i++){
                    printf("%02x ",message[i]);
                }
                printf("\n");
            
            if(!calc_checksum(p,(int)receivedbytes)){
                printf("Length: %i", (int)receivedbytes);
                printf("パケットを破棄します.\n");
                
            }else{
                
                byte value = readReg(REG_PKT_SNR_VALUE);
                if( value & 0x80 ) // The SNR sign bit is 1
                {
                    // Invert and divide by 4
                    value = ( ( ~value + 1 ) & 0xFF ) >> 2;
                    SNR = -value;
                }
                else
                {
                    // Divide by 4
                    SNR = ( value & 0xFF ) >> 2;
                }
                
                if (sx1272) {
                    rssicorr = 139;
                } else {
                    rssicorr = 157;
                }
                count++;
                printf("rx count: %d\n",count);
                
                int packetrssi = readReg(0x1A)-rssicorr;
                int rssi = readReg(0x1B)-rssicorr;
                //mac_frame_payload_t* pay_p = (mac_frame_payload_t*)p;
                printf("\n");
                printf("------------------\n");
                printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);


                printf("Packet RSSI: %d, ", packetrssi);
                printf("RSSI: %d, ",rssi);
                printf("SNR: %li, ", SNR);
                printf("Length: %i", (int)receivedbytes);
                printf("\n");
                //printf("Payload: %s\n", message);
                
                if(p->type == DATA || p->type == ACK){
                    output_data_csv_RSSI(packetrssi,rssi,SNR,(int)receivedbytes,rx_filename);
                }
                // sousinsitayatu
                judge_transfer_data(p);

                // payload no entry list.
                /*num_entry = p->len - sizeof(mac_frame_header_t) / sizeof(mac_lora_frame_header_t);

                for(int i = 0; i < num_entry; i++){
                    mac_lora_frame_header_t* ptr = p->payload+sizeof(mac_lora_frame_header_t)*num_entry;
                    insert_routing_table(ptr->addr, ptr->hop, ptr->seq);
                }*/
                for(int i = 0;i < (int)receivedbytes;i++){
                    printf("%02x ",message[i]);
                }
                printf("\n");
                
            } // received a message
        }
    }  //dio0=1;
}

void routing_table_init(){
    r_table = (routing_table_t *) malloc(sizeof(routing_table_t));
    r_table->size=0;
    r_table->head = NULL;
}
void backoff_struct_init(){
    bo_st = (backoff_t*)malloc(sizeof(backoff_t));
    bo_st->size=0;
    bo_st->head=NULL;
}
void packet_table_init(){
    p_table = (packet_table_t*)malloc(sizeof(packet_table_t));
    p_table->size=0;
    p_table->head = NULL;
}

packet_table_entry_t* check_packet_table(uint8_t *srcAddr,uint16_t seq){//,uint8_t flag){
    packet_table_entry_t* current = p_table->head;
    //packet_table_entry_t* prev = NULL;
    
    while(current && is_smaller_addr(current->srcAddr,srcAddr)){
       // prev = current;
        current = current->next;
    }
    //while(current && is_smaller_seq(current->seq,seq)){
    while(current && is_same_addr(current->srcAddr,srcAddr) && current->seq < seq){
        //prev = current;
        current = current->next;
    }
    if(current && is_same_seq(current->seq,seq)){
        return current;
    }
    return NULL;
}
void output_data_csv_backoff_time(char* filename,time_t start_backoff_time,double backoff){
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
/*
void output_data_csv_time_now(char* filename,struct tm now){
	if((fp = fopen(filename,"w")) == NULL){
        printf("can't open %s\n",filename);
        exit(1);
    }
    char buf[128];
    struct tm *ptm = now;
    //ptm = localtime(&now);
    strftime(buf,sizeof(buf),"%Y/%m/%d %H:%M:%S",ptm);
    fprintf(fp,"%s",buf);
    fclose(fp);
}
*/

double return_backoff_time(double backoff,time_t backoff_now){
    //printf("backoff time = %lf\n",backoff);
    double return_backoff;
    //printf("backoff: %lf\n",backoff);
    return_backoff = backoff_now + backoff * pow(10.0,-3.0);
    //printf("now time: %ld\n",now);
    //printf("backoff time: %lf\n",backoff_now);
    return return_backoff;
}

void rx_file_open(char* file_name){//file_name = 〇〇.csv
    //create result file csv
    //FILE *fp;
    if((fp = fopen(file_name,"w")) == NULL){
        printf("can't open %s\n",file_name);
        exit(1);
    }
    fprintf(fp,"rx_time,nsec,PacketRSSI,RSSI,SNR,lora_length,,Type,pattern,srcAddr,destAddr,length,seq,tx_checksum,rx_checksum,,srcHop,destHop\n");
    fclose(fp);
}

void tx_file_open(char* file_name){
    if((fp = fopen(file_name,"w")) == NULL){
        printf("can't open %s\n",file_name);
        exit(1);
    }
    fprintf(fp,"backoff_start_time,backoff_value,tx_time,,Type,srcAddr,destAddr,length,seq,tx_checksum,,srcHop,destHop\n");
    fclose(fp);
}

int main (int argc, char *argv[]) {
    if(argv[1]){
        rx_filename = argv[1];
        rx_file_open(rx_filename);
    }else if(!argv[1]){
        printf("パケット受信保存ファイルを設定してください.\n");
        exit(1);
    }
    if(argv[2]){
        tx_filename = argv[2];
        tx_file_open(tx_filename);
    }else if(!argv[1]){
        printf("パケット送信保存ファイルを設定してください.\n");
        exit(1);
    }/*
    else if(!argv[3]){
        printf("data packetのサイズを指定してください.\n");
        exit(1);
    }else if(argv[3]){
        data_size = atoi(argv[3]);
    }
    */
    /*
    if (argc < 2) {
        printf ("Usage: argv[0] sender|rec [message]\n");
        exit(1);
    }
    */
  
    //routing_table init function
    routing_table_init();
    
    //backoff init function
    backoff_struct_init();
    
    //packet table init function (backoff)
    packet_table_init();
    
    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    wiringPiSPISetup(CHANNEL, 500000);
    
    //LoRaを受信モードにセット
    //receiverの設定
    set_rxmode();

    //printf("%d\n",sizeof(mac_frame_header_t));
    //printf("value is=%02x\n",readReg(0x31));
    
    //init tx frame function
    memset(&Hello[0],0,sizeof(Hello));
    memset(&data[0],0,sizeof(data));
    memset(&Ack[0],0,sizeof(Ack));
    mac_tx_hello_frame_header_init(Hello_p);
    mac_tx_hello_frame_payload_init(Hello_p);
    mac_tx_data_frame_header_init(data_packet);
    mac_tx_data_frame_payload_init(data_packet);
    mac_tx_ack_frame_header_init(Ack_p);
   
   
    //自身のmacアドレスの取得
	get_my_mac_addr();
    printf("my ");
	mac_print_addr(my_addr);
    //printf("-----------------------------\n");
    
    //データ挿入
    //Hello
    Hello_p->type = HELLO;
    mac_set_addr(my_addr,Hello_p->SourceAddr);
    //ACK
    Ack_p->type = ACK;
    //mac_set_addr(my_addr,Ack_p->SourceAddr);
    Ack_p->len = sizeof(mac_frame_header_t);
    //DATA
    data_packet->type = DATA;
    mac_set_addr(my_addr,data_packet->SourceAddr);
    uint8_t dest_addr[6] = {0xb8,0x27,0xeb,0x60,0xf7,0xbf};
    mac_set_addr(dest_addr,data_packet->DestAddr);
    
    //mac_print_addr(hello->SourceAddr);
    
    //時刻を一秒追加
    time(&later);
    later+=5;
    //while
    while(1){
        time(&now);
        if(now<=later){
            //time(&now);
        //if(1){
            //時間計測
            //sender=clock();
            
            /*
            printf("\n");
            printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
            printf("------------------\n");
            */
            //while(flag==true) {
                //printf("rec\n");
                
                receivepacket(); 
                
           // }
           
            
            //if(bo_st->head && (now <= bo_st->head->backoff)){
            //if(bo_st->head && (now > return_backoff_time(bo_st->head->backoff))){
            if(bo_st->head ){
                if(now > return_backoff_time(bo_st->head->backoff,bo_st->head->backoff_now)){
                    //printf("debugだよ\n");
                    //return_backoff_time(bo_st->head->backoff);
                    packet_table_entry_t* p_entry = check_packet_table(bo_st->head->srcAddr,bo_st->head->seqNum);
                    if(p_entry->flag != ACK){
                        routing_table_entry_t* r_entry = r_table->head;
                        while(r_entry && !is_same_addr(r_entry->addr,p_entry->packet->DestAddr)){//宛先端末のホップ数を調べる
                            r_entry = r_entry->next;
                        }
                        //再送する場合は要修正
                        ((or_data_packet_t*)p_entry->packet->payload)->srcHop ++;
                        ((or_data_packet_t*)p_entry->packet->payload)->destHop = r_entry->hop;
                        printf("データを転送します.\n");
                        //送信モード
                        set_txmode();
                        
                        output_data_csv_backoff_time(tx_filename,bo_st->head->backoff,bo_st->head->backoff_now);
                        output_data_csv_tx_time(tx_filename);
                        txlora((byte*)p_entry->packet,(byte)p_entry->packet->len);
                        
                        //受信モードに切り替え
                        set_rxmode();
                    }
                    dequeue_backoff_table();
                }
            }
        }else{//else if(r_table_size == 5 ){
            //printf("%d\n",count);
            count=0;
            //時間測定
            sender=clock();
            //送信モードにセット
            set_txmode();
            
            //printf("value is=%02x\n",readReg(0x31));
            //printf("\n");
            
            printf("------------------\n");
            printf("Send packets at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
            //printf("------------------\n");
            /*
            if (argc > 2){
                strncpy((char *)hello, argv[2], sizeof(hello));
            }
        */
            //txlora(hello, strlen((char *)hello));/

            //txlora((byte *)&hello, sizeof(hello));
            
            
            //確認
            //print_mac_frame(Hello_p);
            //print_lora_frame_header(Hello_p);
            //printf("------------------\n");
            
            
            //txlora((byte *)&hello,strlen((char*)hello));
            //txlora((byte*)&hello_packet, sizeof(mac_frame_header_t));
            
            //////
            //mac_lora_frame_header_t hello_payload[256];
            //
            /*
            int num = get_routing_table(hello); { 
                mac_lora_frame_header_t* current = (mac_lora_frame_header_t*) hello->payload;
                routing_table_entry_t* current_entry = rt_table->head;
                int i =0;
                while(current_rt){
                    ccurrent->hop = current_entry->hop;
                    current->seq  = ->seq;
                    current->addr = ->addr;
                    current = current++;
                    i++;
                    current-rt = current_rt->next;
                }
               return i;
            }
            * 
            */
            // header_len = sizeof(mac_frame_header_t);
            // payload_len = num * sizeof(mac_lora_frame_header_t);
            // total_len = header_len + payload_len;
            //Hello_p->len = total_len;
            //txlora((byte*)&Hello, total_len);
            //////
            Hello_p->seqNum++;
            
            int num = get_routing_table(Hello_p);
            int header_len = sizeof(mac_frame_header_t);
            int payload_len = num * sizeof(mac_frame_payload_t);
            printf("%d %d\n",header_len,payload_len);
            int total_len = header_len + payload_len;
            printf("total length: %d\n",total_len);
            
            if(total_len >= 255){
                printf("最大ペイロードサイズを超えています.\n");
                return 0;
            }else{
                Hello_p->len = total_len;
                /*
                for(int i = 53;i<255;i++){
                    Hello[i] =0x11;
                } */
                txlora((byte*)&Hello, (byte)total_len);
            }
            
                
            //受信モードに切り替え
            set_rxmode();
            
            //時間計測
            receiver = clock();
            printf("%f秒\n",(double)(receiver-sender)/CLOCKS_PER_SEC);
            
            //時間の更新
            time(&later);
            gettimeofday(&tv_rand,NULL);
            srand(tv_rand.tv_sec * (tv_rand.tv_usec + 1));

            later= later + 4.9 + (rand() % 200) * pow(10.0,-3.0) ;
            //delay(5000);
        
        }
        
    }

/*
    if (!strcmp("sender", argv[1])) {
        opmodeLora();
        // enter standby mode (required for FIFO loading))
        opmode(OPMODE_STANDBY);

        writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

        configPower(23);

        printf("Send packets at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
        printf("------------------\n");

        if (argc > 2)
            strncpy((char *)hello, argv[2], sizeof(hello));

        while(1) {
            txlora(hello, strlen((char *)hello ));
            delay(5000);
            * //delay(50)
            *         }
    } else {

        // radio init
        opmodeLora();
        opmode(OPMODE_STANDBY);
        opmode(OPMODE_RX);
        printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);
        printf("------------------\n");
        while(1) {
            receivepacket(); 
            delay(1);
        }
*/
    
    //free(r_table);
    return (0);
}
