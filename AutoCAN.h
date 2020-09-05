typedef enum {
    CAN_RX_ERROR,       //did not receive expected message
    CAN_NODE_OFFLINE,   //CAN heartbeat check failed
    CAN_EPAS_1,         //epasuino entered steering mode 1
    CAN_EPAS_2,         //epasuino entered steering mode 2
    CAN_EPAS_3,         //epasuino entered steering mode 3
    CAN_EPAS_4,         //epasuino entered steering mode 4
    CAN_EPAS_5,         //epasuino entered steering mode 5
    CAN_EPAS_6          //epasuino entered steering mode 6
} canSystemMessage;