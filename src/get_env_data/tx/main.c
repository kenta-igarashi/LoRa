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

#define USER_DATA_FRAME_LEN 1500
#define USER_CTRL_FRAME_LEN 100

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
time_t current,later;

//現在時刻の取得
struct timespec ts;
struct tm tm_tx;

//パケットヘッダの構造体
typedef struct{
    uint8_t type;//パケットかデータかの識別
    uint8_t SourceAddr[6]={};
    uint8_t DestAddr[6]={};
    uint8_t len;
    uint16_t seqNum;
    struct timespec time;
    uint16_t checksum;
    uint8_t payload[4];
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
    uint8_t DestAddr[6];
    uint16_t seqNum;
    //unsigned char message[100]; 
    char message[100]; 
}__attribute__((packed))or_data_packet_t;

typedef struct routing_table_entry_struct{
    uint8_t hop;
    uint8_t seq;
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
    uint8_t flag;
    backoff_entry_struct* next;
}backoff_entry_t;

typedef struct{
    backoff_entry_t* head;
    uint8_t size;
}backoff_t;

/*固定長の配列を宣言しないとバイナリがバグる
 * uint8_tかunsigned charの１バイトで宣言
 * 255はLoRaの最大ペイロードサイズ
 */
uint8_t Hello[255] = {};
uint8_t data[255] = {};
mac_frame_header_t* data_packet = (mac_frame_header_t*)&data;
mac_frame_header_t *Hello_p=(mac_frame_header_t*)&Hello;

routing_table_t* r_table = NULL;
backoff_t* b_st = NULL;

uint8_t my_addr[6]={};

char* filename;
FILE *fp;
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

/*mac frameとlora frameの初期化
 */

void mac_tx_frame_header_init(mac_frame_header_t* hello_pack){
    mac_frame_header_t* ctrl_hdr =hello_pack;

    ctrl_hdr->type = HELLO;
    mac_set_bc_addr(ctrl_hdr->SourceAddr);
    mac_set_bc_addr(ctrl_hdr->DestAddr);
    //ctrl_hdr->len = USER_CTRL_FRAME_LEN;// hdr + payload
    ctrl_hdr->len = 0;// hdr + payload
    ctrl_hdr->seqNum = 0;
    ctrl_hdr->checksum = 0;

}

void mac_tx_frame_payload_init(mac_frame_header_t* frame){
    printf("debug\n");
    mac_frame_payload_t* ctrl_org_hdr = (mac_frame_payload_t*)frame->payload;
    //byte * p = (byte*)ctrl_org_hdr;
    ctrl_org_hdr->Hop= 0;
    mac_set_bc_addr(ctrl_org_hdr->DestAddr);
    ctrl_org_hdr->seqNum = 0;
    /*
    for(int i = 0 ;i<10;i++){
        printf("%02x ",p[i]);
    }
    printf("\n");
    */
}
void mac_tx_frame_data_init(mac_frame_header_t* frame){
    or_data_packet_t* data_frame = (or_data_packet_t*)frame->payload;
    data_frame->srcHop = 0;
    data_frame->destHop = 0;
    mac_set_bc_addr(data_frame->DestAddr);
    data_frame->seqNum = 0;
    memset(&(data_frame->message),0,sizeof(message));
}

void print_mac_frame_header(mac_frame_header_t* data){
    printf("mac packet header: \n");
    if(data->type == HELLO) printf("type: HELLO\n");
    else printf("type: DATA\n");
    //printf("type: %u\n",data->type);
    printf("Source ");
    mac_print_addr(data->SourceAddr);
    printf("Destination ");
    mac_print_addr(data->DestAddr);
    printf("length: %u\n",data->len);
    printf("sequence number: %u\n",data->seqNum);
    printf("payload: %s\n",data->payload);
    printf("\n");
}

void print_mac_frame_payload(mac_frame_header_t* data){
    mac_frame_payload_t* packet = (mac_frame_payload_t*)data->payload;
    //byte* p = (byte*)packet;
    printf("lora packet header: \n");
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

boolean is_smaller_addr(uint8_t* addr1,uint8_t* addr2){
    for(int i = 0 ;i<6;i++){
        if(addr1[i]<addr2[i]){ //addr1=origin 
            return true;
        }else if(addr1[i]>addr2[i]){
            return false;
        }
    }
    return false;
}

boolean is_same_addr(uint8_t* origin,uint8_t* target){
    for(int i =0;i<6;i++){
        //mac_print_addr(origin);
        //mac_print_addr(target);
        if(origin[i]!=target[i]) return false;
    }

    return true;
}
void insert_routing_table(uint8_t* addr,uint8_t hop,uint8_t seq){

    routing_table_entry_t* current = r_table->head;
    routing_table_entry_t* previous = NULL;
    
    //if(!r_table->head){
      //insert  
    //  return ;
    //}
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
    }
    // else if(current->addr == addr){
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

int get_routing_table(mac_frame_header_t* hello){
    mac_frame_payload_t* current = (mac_frame_payload_t*) hello->payload;
    routing_table_entry_t* current_entry = r_table->head;
    int i =0;
    while(current_entry){
        current->Hop = current_entry->hop;
        current->seqNum  = current_entry->seq;
        //current->DestAddr = current_entry->addr;
        mac_set_addr(current_entry->addr,current->DestAddr);
        current = current++;
        i++;
        current_entry = current_entry->next;
    }
   return i;
}

void print_routing_table(routing_table_t* routing_t){
    routing_table_entry_t* current =routing_t->head;
    printf("routing table is:\n");
    //routing_table_entry_t* current = r_table->head;
    int i =1;
    if(!current){ //current==NULL
        printf("data nothing\n");
        return ;
    }
    while(current){//current!=NULL
        printf("Node%d ",i);
        mac_print_addr(current->addr);
        printf("Hop: %u\n",current->hop);
        printf("Sequence Number: %u\n",current->seq);
        printf("table size: %d\n",routing_t->size);
        i++;
        current = current->next;
    }
    
}

void print_data_frame(mac_frame_header_t* packet){
    or_data_packet_t* data_frame = (or_data_packet_t*)packet->payload;
    printf("data: \n");
    printf("src hop: %u\n",data_frame->srcHop);
    printf("dest hop: %u\n",data_frame->destHop);
    printf("Destination ");
    mac_print_addr(data_frame->DestAddr);
    printf("sequence number: %u\n",data_frame->seqNum);
}

void add_routing_table(routing_table_t* routing_t){
    routing_table_entry_t* current =routing_t->head;
    printf("routing table is:\n");
    //routing_table_entry_t* current = r_table->head;
    if(!current){ //current==NULL
        printf("data nothing\n");
        return ;
    }
    while(current){//current!=NULL
        insert_routing_table(current->addr,current->hop+1,current->seq);
        current = current->next;
    }
    
}

boolean is_smaller_backoff(double origin,double target){
    if(origin<target)  return true;
    else if(origin>target) return false;
    return false;
}
void add_queue_backoff(uint8_t* srcAddr,uint16_t seq,double backoff,uint8_t flag){
    backoff_entry_t* current = b_st->head;
    backoff_entry_t* previous = NULL;
    
    while(current && is_smaller_backoff(current->backoff,backoff)){
        previous = current;
        current = current->next;
    }

    if(1){
        //insert
        backoff_entry_t* new_entry = (backoff_entry_t* )malloc(sizeof(backoff_entry_t));
        memset(new_entry, 0, sizeof(backoff_entry_t));

        new_entry->backoff = backoff;
        mac_set_addr(srcAddr, new_entry->srcAddr);
        new_entry->seqNum = seq;
        //new_entry->flag = ;
        new_entry->next = NULL;
        (b_st)->size++;
        
        if(!previous){
            new_entry->next = b_st->head;
            b_st->head = new_entry;            
        }else{
            new_entry->next = previous->next;
            previous->next = new_entry;
        }
    }
    return ;
} 

void insert_backoff(uint8_t* srcAddr,uint16_t seq,double backoff,uint8_t flag){
    backoff_entry_t* current = b_st->head;
    backoff_entry_t* previous = NULL;
    
    while(current && is_smaller_backoff(current->backoff,backoff)){
        previous = current;
        current = current->next;
    }

    if(1){
        //insert
        backoff_entry_t* new_entry = (backoff_entry_t* )malloc(sizeof(backoff_entry_t));
        memset(new_entry, 0, sizeof(backoff_entry_t));

        new_entry->backoff = backoff;
        mac_set_addr(srcAddr, new_entry->srcAddr);
        new_entry->seqNum = seq;
        //new_entry->flag = ;
        new_entry->next = NULL;
        (b_st)->size++;
        
        if(!previous){
            new_entry->next = b_st->head;
            b_st->head = new_entry;            
        }else{
            new_entry->next = previous->next;
            previous->next = new_entry;
        }
    }
    return ;
    
}

double sigmoid(double gain, double x){
    return (1.0 / (1.0 + exp(-gain * x)));
}

double OR_calculate_backoff(or_data_packet_t* hdr,uint8_t hop){
    //or_data_packet_t* data_p = (mac_frame)
    struct timeval tv;
    gettimeofday(&tv,NULL);
    srand((unsigned int)tv.tv_sec * ((unsigned int)tv.tv_usec + 1));
    double expected = (double)hdr->destHop - 1;
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
    
    if(max_random_backoff != 0){
        backoff += rand() % (int)max_random_backoff;
    }else{
        backoff += rand() % (10 * 1000);
    }
    return backoff;
}

void judge_tansfer_data(mac_frame_header_t* packet_p){
    //mac_frame_header_t* packet_p = (mac_frame_header_t*)&packet;
    or_data_packet_t* data_p = (or_data_packet_t*)packet_p->payload;
    routing_table_entry_t* current = r_table->head;
    if(packet_p->type == DATA){
        
            print_mac_frame_header(packet_p);
            print_data_frame(packet_p);
        
        //if(is_same_addr(my_addr,packet_p->DestAddr)){//宛先端末が自分の場合
        if(is_same_addr(my_addr,data_p->DestAddr)){//宛先端末が自分の場合
            for(int i = 0;i<sizeof(data_p->message)/sizeof(data_p->message[0]);i++){
                message[i] =data_p->message[i];
            }
            printf("payload message: %s\n",message);
        }else{//自分ではない場合
            printf("transfer data\n");
            //転送待機時間の算出
            while(current && is_same_addr(current->addr,data_p->DestAddr)){//宛先端末のホップ数を調べる
                current = current->next;
            }
            insert_backoff(packet_p->SourceAddr,data_p->seqNum,OR_calculate_backoff(data_p,current->hop),1);
            //delay(OR_calculate_backoff(data_p,current->hop));//待機
            //再送制御

        }
    }
    else if(packet_p->type == HELLO){
        print_mac_frame_header(packet_p);
        print_mac_frame_payload(packet_p);
        
        insert_routing_table(packet_p->SourceAddr, 1, packet_p->seqNum);
        add_routing_table(r_table);
        print_routing_table(r_table);
    }
}

void print_payload_data(){
    printf("message: %s\n",message);
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
            printf("%d\n",count);
            
            mac_frame_header_t* p = (mac_frame_header_t*)message;
            //mac_frame_payload_t* pay_p = (mac_frame_payload_t*)p;
            printf("\n");
            printf("------------------\n");
            printf("Listening at SF%i on %.6lf Mhz.\n", sf,(double)freq/1000000);


            printf("Packet RSSI: %d, ", readReg(0x1A)-rssicorr);
            printf("RSSI: %d, ", readReg(0x1B)-rssicorr);
            printf("SNR: %li, ", SNR);
            printf("Length: %i", (int)receivedbytes);
            printf("\n");
            //printf("Payload: %s\n", message);
            
            //print_mac_frame_header(p);
            //print_data_frame(p);
    
            // sousinsitayatu
            judge_tansfer_data(p);
            /*
            insert_routing_table(p->SourceAddr, 1, p->seqNum);
            add_routing_table(r_table);
            print_routing_table(r_table);
            */
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

    }  //dio0=1;
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

void get_time_now(mac_frame_header_t* hdr){
    //struct timespec ts;
    //struct tm tm;
    clock_gettime(CLOCK_REALTIME,&ts);
    localtime_r(&ts.tv_sec,&tm_tx);
    hdr->time = ts;
    //printf("sizeof ts:%d",sizeof(ts));
    //printf("sizeof ts:%10ld",hdr->time.tv_sec);
    //printf("time %10ld.%09ld CLOCK_REALTIME\n",ts.tv_sec,ts.tv_nsec);
    //printf("%d/%02d/%02d/ %02d:%02d:%02d.%09ld\n",tm.tm_year+1900,tm.tm_mon+1,tm.tm_mday,tm.tm_hour,tm.tm_min,tm.tm_sec,ts.tv_nsec);
}

char* judge_packet_type(mac_frame_header_t* hdr,char* packet_type){
    if(hdr->type == HELLO){
        strcpy(packet_type,"HELLO");
        return  packet_type;
    }
    else if(hdr->type == DATA){
        strcpy(packet_type,"DATA");
        return  packet_type;
    }
    return 0;
}

void output_data_csv(mac_frame_header_t* hdr){
    char packet_type[20];
    //memset(
    if((fp = fopen(filename,"a+")) == NULL){
        printf("can't open file");
        exit(1);
    }
    fprintf(fp,"%u,,%s,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%02x,%u,%d,%02d,%02d,%02d,%02d,%02d,%09ld,%u\n"
    ,hdr->seqNum
    ,judge_packet_type(hdr,packet_type)
    ,hdr->SourceAddr[0],hdr->SourceAddr[1],hdr->SourceAddr[2],hdr->SourceAddr[3],hdr->SourceAddr[4],hdr->SourceAddr[5]
    ,hdr->DestAddr[0],hdr->DestAddr[1],hdr->DestAddr[2],hdr->DestAddr[3],hdr->DestAddr[4],hdr->DestAddr[5]
    ,hdr->len
    ,tm_tx.tm_year+1900,tm_tx.tm_mon+1,tm_tx.tm_mday,tm_tx.tm_hour,tm_tx.tm_min,tm_tx.tm_sec,ts.tv_nsec
    ,hdr->checksum
    );
    fclose(fp);
}

void txlora(byte *frame, byte datalen) {

    get_time_now(Hello_p);
    
    //checksum function
    Hello_p->checksum = 0;
    Hello_p->checksum = checksum(frame,(int)datalen);
    printf("checksum: %u\n",Hello_p->checksum);
    
    output_data_csv((mac_frame_header_t*)frame);
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
    printf("send: %s\n", frame);
    
    //確認
    print_mac_frame_header(Hello_p);
    //print_mac_frame_payload(Hello_p);
    printf("------------------\n");

    
    //printf("length: %d\n",sizeof(Hello));
    
    
    for(int i = 0;i<datalen;i++){
        printf("%02x ",frame[i]);
    }
    printf("\n");
    
/*
    for(int i = 0;i<datalen;i++){
        printf("%02x ",frame[i]);
    }
    printf("\n");
  */  
}

void file_open(char* file_name){
    if((fp = fopen(file_name,"w")) == NULL){
        printf("can't open %s",file_name);
        exit(1);
    }
    fprintf(fp,"tx_seq,,TYPE,srcAddress,srcAddress,srcAddress,srcAddress,srcAddress,srcAddress,destAddress,destAddress,destAddress,destAddress,destAddress,destAddress,length,年,月,日,時,分,秒,nsec,tx checksum\n");
    fclose(fp);
}

int main (int argc, char *argv[]) {
    /*
    if (argc < 2) {
        printf ("Usage: argv[0] sender|rec [message]\n");
        exit(1);
    }
    */
    if(!argv[1]){
        printf("送信回数を設定してください.\n");
        exit(1);
    }else if(!argv[2]){
        printf("保存ファイルを指定してください.\n");
        exit(1);
    }else{
        filename = argv[2];
        file_open(filename);
    }

    //hello.SourceAddr=0;
    //mac_frame_header_t hello_packet;
    
    //mac_packet_header_t *hello=&ctrl_frame;
  /*
    byte hello_packet[1000];
    for(int i =0;i<1000;i++){
        hello_packet[i]=0;
    }
  */
    //mac_frame_header_t *Hello_p=(mac_frame_header_t*)&Hello;
    
    //routing_table init function
    r_table = (routing_table_t *) malloc(sizeof(routing_table_t));
    r_table->size=0;
    r_table->head = NULL;
 
    wiringPiSetup () ;
    pinMode(ssPin, OUTPUT);
    pinMode(dio0, INPUT);
    pinMode(RST, OUTPUT);

    wiringPiSPISetup(CHANNEL, 500000);
    
    /*
    //LoRaを受信モードにセット
    //receiverの設定
    SetupLoRa();
    opmodeLora();
    opmode(OPMODE_STANDBY);
    opmode(OPMODE_RX);
    */
    //送信モードに設定
    
    //送信モードにセット
    opmodeLora();
    // enter standby mode (required for FIFO loading))
    opmode(OPMODE_STANDBY);

    writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

    configPower(23);
    
    
    //mac_frame_header_t *hello=(mac_frame_header_t*)&hello_packet;
    

    //printf("%d\n",sizeof(mac_frame_header_t));
    //printf("value is=%02x\n",readReg(0x31));
    
    //init tx frame function
    memset(&Hello[0],0,sizeof(Hello));
    memset(&data[0],0,sizeof(data));
    mac_tx_frame_header_init(Hello_p);
    //mac_tx_frame_payload_init();
    //mac_tx_frame_payload_init(Hello_p);
    memset(&(Hello_p->payload[0]),0,sizeof(Hello_p->payload));
    
    //自身のmacアドレスの取得
	get_my_mac_addr();
    printf("my ");
	mac_print_addr(my_addr);
    //printf("-----------------------------\n");
     
    //仮のデータ挿入
    Hello_p->type = HELLO;
    mac_set_addr(my_addr,Hello_p->SourceAddr);
    Hello_p->seqNum = 0;
    //mac_print_addr(hello->SourceAddr);
    
    while(Hello_p->seqNum < atoi(argv[1])){
    //while(1){
    //時刻を一秒追加
    time(&later);
    later+=1;
    //while
    while(current<=later){
        time(&current);
        //if(current<=later){
        if(0){
      
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
                
                
                //delay(100);
                //delay(1);
           // }
            /*
            if(current>b_st->head->backoff){
                int num = get_routing_table(Hello_p);
                int header_len = sizeof(mac_frame_header_t);
                int payload_len = num * sizeof(or_data_packet_t);
                printf("%d %d\n",header_len,payload_len);
                int total_len = header_len + payload_len;
                Hello_p->len = total_len;
                txlora((byte*)&Hello, total_len);
            }
            */
            
            
        }//else if(1){
        if(current>later){
            //printf("%d\n",count);
            count=0;
            //時間測定
            sender=clock();
            /*
            //送信モードにセット
            opmodeLora();
            // enter standby mode (required for FIFO loading))
            opmode(OPMODE_STANDBY);

            writeReg(RegPaRamp, (readReg(RegPaRamp) & 0xF0) | 0x08); // set PA ramp-up time 50 uSec

            configPower(23);
            //printf("value is=%02x\n",readReg(0x31));
            //printf("\n");
            */
            
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
            
            //int num = get_routing_table(Hello_p);
            int header_len = sizeof(mac_frame_header_t);
            //int payload_len = num * sizeof(mac_frame_payload_t);
            //printf("%d %d\n",header_len,payload_len);
            //int total_len = header_len + payload_len;
            printf("total length: %d\n",header_len);
            if(header_len>255){
                printf("最大ペイロードサイズを超えています\n");
                break;
            }/*else if(Hello_p->seqNum >atoi(argv[1])+1){
                printf("end tx\n");
                exit(1);
            }*/
            else {
                Hello_p->len = header_len;
                txlora((byte*)&Hello, (byte)header_len);
            //txlora((byte*)&Hello,40);
            }
            /*
            //受信モードに切り替え
            SetupLoRa();
            opmodeLora();
            opmode(OPMODE_STANDBY);
            opmode(OPMODE_RX);
            */
            //時間計測
            receiver = clock();
            printf("%f秒\n",(double)(receiver-sender)/CLOCKS_PER_SEC);
            
            //時間の更新
            //time(&later);
            //later+=1;
            //delay(5000);
        
        }
        
    }

}
printf("ent tx\n");


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
