#include <stdio.h>

typedef struct{
    uint8_t type;//パケットかデータかの識別
    uint8_t SourceAddr[6]={};
    uint8_t DestAddr[6]={};
    uint8_t SenderAddr[6]={};
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
