#define CAN_INFO_MSG_ID 411   //message id for messages on can bus
#define CAN_ERROR_MSG_ID 911    //message id for errors on can bus

typedef struct {
    int messageID;
    byte contentID;
    byte extraInfo;
    char* messageText;
} canSystemMessage;

canSystemMessage canRxError =       {CAN_ERROR_MSG_ID, 1, NULL, "CAN RX ERROR"};
canSystemMessage canNodeOffline =   {CAN_ERROR_MSG_ID, 2, NULL, "CAN NODE OFFLINE" };
canSystemMessage canEpas1 =         {CAN_INFO_MSG_ID, 3, NULL, "Manual Steering"};
canSystemMessage canEpas2 =         {CAN_INFO_MSG_ID, 4, NULL, "Firm Steering"};
canSystemMessage canEpas3 =         {CAN_INFO_MSG_ID, 5, NULL, "Sport+ Steering"};
canSystemMessage canEpas4 =         {CAN_INFO_MSG_ID, 6, NULL, "Sport Steering"};
canSystemMessage canEpas5 =         {CAN_INFO_MSG_ID, 7, NULL, "Touring Steering"};
canSystemMessage canEpas6 =         {CAN_INFO_MSG_ID, 8, NULL, "Comfort Steering"};