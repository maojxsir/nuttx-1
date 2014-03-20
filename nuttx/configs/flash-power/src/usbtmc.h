#ifndef __FL_POWER_USBTMC_H 
#define __FL_POWER_USBTMC_H 

/* USBTMC request,see USBTMC spec 4.2.1*/
// 0 reserved
#define INITIATE_ABORT_BULK_OUT         (1)
#define CHECK_ABORT_BULK_OUT_STATUS     (2)
#define INITIATE_ABORT_BULK_IN          (3)
#define CHECK_ABORT_BULK_IN_STATUS      (4)
#define INITIATE_CLEAR                  (5)
#define CHECK_CLEAR_STATUS              (6)
#define GET_CAPABILITIES                (7)
// 8~63 reserved
#define INDICATIOR_PULSE                (64)
// 65~127 Reserved for use by USBTMC specification
// 128~191 Reserved for use by USBTMC subclass
// 192~255 Reserved for VISA specification

/* USBTMC status value*/
// 0 reserved
#define STATUS_SUCCESS                  (0x01)        //Success
#define STATUS_PENDING                  (0x02)        //Warning
// 0x03~0x1F Reserved Warning,use by USBTMC
// 0x20~0x3F Reserved Warning,use by USBTMC subclass
// 0x40~0x7F Reserved Warning,use by VISA 
#define STATUS_FAILED                   (0x80)        //Failure,unspecified reason
#define STATUS_TRANSFER_NOT_IN_PROGRESS (0x81)
#define STAUS_SPLIT_NOT_IN_PROGRESS     (0x82)        //Failure
#define STATUS_SPLIT_IN_PROGRESS        (0x83)        //Failure
// 0x84~0x9F Reserved Failure for USBTMC use
// 0xA0~0xBF Reserved Failure for USBTMC subclass use
// 0xC0~0xFF Reserved Failure for VISA

/* USBTMC MsgID*/
//                                  Value     Dir  Description
// 0 reserved 
#define DEV_DEP_MSG_OUT             (1)    // OUT  USBTMC device dependent command message
#define REQUEST_DEV_DEP_MSG_IN      (2)    // OUT  USBTMC command message, need reponse in BUKL-IN endpoint
#define DEV_DEP_MSG_IN              (2)    // IN   USBTMS reponse message to REQUEST_DEV_DEP_MSG_IN
// 3~125 Reserved
#define VENDOR_SPECIFIC_OUT         (126)  // OUT  USBTMC vendor spcific command message
#define REQUEST_VENDOR_SPECIFIC_IN  (127)  // OUT  USBTMC vendor spcific command message need reponse
#define VENDOR_SPECIFIC_IN          (127)  // IN   Reponse to REQUEST_VENDOR_SPECIFIC_IN
// 128~191 Reserved for USBTMC subclass
// 192~255 Reserved for VISA        

struct usbtmc_bulkout_header_s {
  uint8_t msgID;                 // 0
  uint8_t bTag;                  // 1
  uint8_t bTagInverse;           // 2
  uint8_t reserved;              // 3  must be zero
  uint32_t transferSize          // 4~7
  union {
    struct header_DevDepMsgOut DevDepMsgOut;
    struct header_ReqDevDepMsgIn ReqDevDepMsgIn;
    struct header_VendorOut VendorOut;
    struct header_ReqVendorOut ReqVendorOut;
    struct header_DevDepMsgIn DevDepMsgIn;
    struct  header_VendorIn VendorIn;
  } header;
};
typedef struct usbtmc_bulkout_header_s usbtmc_bulkout_header;
//Bulk-IN header is same to Bulk-OUT header
typedef struct usbtmc_bulkout_header_s usbtmc_bulkin_header;

// DEV_DEP_MSG_OUT 
struct header_DevDepMsgOut {
  union {  // 8
    uint8_t value;
    uint8_t EOM:1;   //End Of Message
  } bmTransferAttr;
  uint8_t reserved[3]; //9~11
};

// REQUEST_DEV_DEP_MSG_IN
struct header_ReqDevDepMsgIn {
  union {  //8
    uint8_t value;
    uint8_t TermCharEnabled:1;  //Terminal Character Enable
    uint8_t reserved:1;
  } bmTransferAttr;
  uint8_t TermChar; // 9 Terminal Character
  uint8_t reserved[2] //10~11 MBZ(Must Be Zero)
};

// VENDOR_SPECIFIC_OUT
struct header_VendorOut {
  uint8_t reserved[4]; //8~11 MBZ(Must Be Zero)
};

// REQUSET_VENDOR_SPECIFIC_IN
struct header_ReqVendorOut {
  uint8_t reserved[4]; //8~11 MBZ(Must Be Zero)
};

// DEV_DEP_MSG_IN
struct header_DevDepMsgIn {
  union {
    uint8_t value;
    uint8_t TermChar:1;
    uint8_t EOM:1;
  } bmTransferAttr;
  uint8_t reserved[3]; //9~11
};

// VENDOR_SPECIFIC_IN
struct  header_VendorIn {
  uint8_t reserved[4];
};

//Interrupt In notification
struct usbtmc_intin_header {
  union {
    uint8_t value;
    uint8_t subclassBit:1; //D7
    uint8_t vendorBit:1;   //D6
    uint8_t bit5:1;
    uint8_t bit4:1;
    uint8_t bit3:1;
    uint8_t bit2:1;
    uint8_t bit1:1;
    uint8_t bit0:1;
  } flag;
  uint8_t notify[0];
};


#endif