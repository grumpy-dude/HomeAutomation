#include "RFM69/RFM69.h"

enum Protocol_Version {
    First_Protocol_Version  = 1
};

const Protocol_Version The_Current_Protocol_Version = First_Protocol_Version;


enum Data_Type {
    Test_Packet_Data_Type = 1,
    Temperature_Data_Type
};


struct Test_Packet_Data {
    uint8_t packet_Sequence_Number;
} __attribute__((packed));


struct Temperature_Data {
    uint8_t device_ID;
    uint8_t packet_Sequence_Number;
    float   temperature;
} __attribute__((packed));


struct Packet_Data {
    uint8_t protocol_Version = First_Protocol_Version;
    uint8_t source_Node_ID;
    uint8_t target_Node_ID;
    uint8_t data_Type;

    union The_Data {
        Test_Packet_Data test_Packet;
        Temperature_Data temperature;
    } the_Data;
} __attribute__((packed));

static_assert( (sizeof(Packet_Data) <= RF69_MAX_DATA_LEN), "\"Packet_Data\" struct too large." );
