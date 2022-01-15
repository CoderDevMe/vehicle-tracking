/*
 * FS1000A RF Transmitter
 * Vcc --- 5V
 * Gnd --- Gnd
 * Data --- D12
 * 
 * 
 * Ublox GPS
 * Vcc --- 3.3V
 * Gnd --- Gnd
 * Tx , Rx
 */


#include <RH_ASK.h>
#include <SPI.h> // Not actually used but needed to compile

enum DYNAMIC_MODEL {
  
  PORTABLE  = 0,
  STATIONARY = 2,
  PEDESTRIAN  = 3,
  AUTOMOTIVE = 4,
  SEA = 5,
  AIRBORNE_1G = 6,//airborne with < 1g acceleration
  AIRBORNE_2G = 7,//airborne with < 2g acceleration
  AIRBORNE_4G = 8,//airborne with < 4g acceleration
  WRIST = 9 //worn watch
  
};


enum MSG_TYPE {
    nav_posllh = 0,
    nav_status ,
    nav_solution,
    cfg_tar_pos,
    cfg_tar_settings,
    mon_request,
    
    undefined=99,
};


struct UBX_NAV_STATUS {
  
  uint8_t  cls;
  uint8_t id;
  uint16_t len;
  uint32_t time;                                  // GPS msToW
  uint8_t fix_type;
  uint8_t fix_status;
  uint8_t differential_status;
  uint8_t res;
  uint32_t time_to_first_fix;
  uint32_t uptime;                                // milliseconds
};

struct POS_INFO {

  unsigned long lastupdate;
  unsigned long gps_time;
  double lat;
  double lon;
  double last_lat;
  double last_lon;
  float height;
  uint8_t fix_type;
  uint8_t used_sats;
  bool fix_ok;
  float dt;
  bool flag_update_pos;
};


struct UBX_NAV_SOL {
  
  uint8_t  cls;
  uint8_t id;
  uint16_t len;
  uint32_t time;
  int32_t time_nsec;
  uint16_t week;
  uint8_t fix_type;
  uint8_t fix_status;
  int32_t ecef_x;
  int32_t ecef_y;
  int32_t ecef_z;
  uint32_t position_accuracy_3d;
  int32_t ecef_x_velocity;
  int32_t ecef_y_velocity;
  int32_t ecef_z_velocity;
  uint32_t speed_accuracy;
  uint16_t position_DOP;
  uint8_t res;
  uint8_t satellites;
  uint32_t res2;
};


enum UBS_PROTOCOL_BYTES {
  
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    PREAMBLE3 = 0x0A,
    PREAMBLE4 = 0x0B,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    CLASS_MON = 0x0A,
    CLASS_RXM = 0x02,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_DOP = 0x4,
    MSG_SOL = 0x6,
    MSG_PVT = 0x7,
    MSG_VELNED = 0x12,
    MSG_CFG_CFG = 0x09,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_MSG = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_SBAS = 0x16,
    MSG_CFG_GNSS = 0x3E,
    
    MSG_MON_HW = 0x09,
    MSG_MON_HW2 = 0x0B,
    MSG_MON_VER = 0x04,
    MSG_NAV_SVINFO = 0x30,
    MSG_RXM_RAW = 0x10,
    MSG_RXM_RAWX = 0x15,
    
    MSG_POS_INFO = 0x50,
    MSG_CFG_TARGET_POS = 0x51,
    MSG_CFG_TARGET_SETTINGS = 0x52,
    MSG_MON_REQUEST = 0x53,
   
};


struct UBX_NAV_POSLLH  {
  uint8_t  cls;
  uint8_t id;
  uint16_t len;
  uint32_t time;                                  // GPS msToW
  int32_t longitude;
  int32_t latitude;
  int32_t altitude_ellipsoid;
  int32_t altitude_msl;
  uint32_t horizontal_accuracy;
  uint32_t vertical_accuracy;
}__attribute__((packed));

struct UBX_CFG_RATE {
  uint8_t cls = CLASS_CFG;
  uint8_t id = MSG_CFG_RATE;
  uint16_t len = 6;
  uint16_t meas_rate = 100; // 100 ms -> 10 HZ
  uint16_t nav_rate  = 1;
  uint16_t time_ref  = 0;  //UTC time
  
};

struct UBX_CFG_MSG_RATE {

   uint8_t cls = CLASS_CFG;
  uint8_t id = MSG_CFG_MSG;
  uint16_t len = 3;
  
  uint8_t msg_cls ;
  uint8_t msg_id ;
  uint8_t rate;
};

struct UBX_CFG_NAV_SETTINGS {
  uint8_t cls = CLASS_CFG;
  uint8_t id = MSG_CFG_NAV_SETTINGS;
  uint16_t len = 36;
  uint16_t mask;
  uint8_t dynModel;
  uint8_t fixMode;
  int32_t fixedAlt=0;
  uint32_t fixedAltVar=1;
  int8_t minElev=5;
  uint8_t drLimit=0;
  uint16_t pDop=25;
  uint16_t tDop=25;
  uint16_t pAcc=10000;
  uint16_t tAcc=300;
  uint8_t staticHoldThresh=0;
  uint8_t res1;
  uint32_t res2;
  uint32_t res3;
  uint32_t res4;
};

struct MSG_TO_SEND{
  double lat;
  double lon;
};



void calcChecksum(uint8_t* CK ,int _size,unsigned char* buff ) {
  memset(CK, 0, 2);
  
  for (int i = 0; i < _size; i++) {

    CK[0] +=buff[i];
    CK[1] += CK[0];
  }
}


void sendMessage(Stream *port ,uint8_t* HEADER,uint8_t* msg_buff , int _size,bool debug=false){
  
  uint8_t checksum[2];
  // calc checsum of cfg_rate      
  calcChecksum(checksum ,_size,msg_buff);
  // send config
  port->write(HEADER,2);
  port->write(msg_buff,_size);
  port->write(checksum,2);
  
  #if DEBUG
  if (!debug)// msg_buff[0] == CLASS_MON &&  msg_buff[1] == MSG_POS_INFO)
    return ;
  Serial.print("\nSEND BUFF:");
  Serial.print(" ");
  Serial.print(HEADER[0],HEX);
  Serial.print(" ");
  Serial.print(HEADER[1],HEX);
  Serial.print(" ");
  for (int i=0;i<_size;i++){
    Serial.print(msg_buff[i],HEX);
    Serial.print(" ");
  }
  Serial.print(checksum[0],HEX);
  Serial.print(" ");
  Serial.println(checksum[1],HEX);
  
  #endif
  
}

UBX_NAV_POSLLH navposllh;
UBX_NAV_STATUS navstatus ;
UBX_NAV_SOL navsolution;
MSG_TO_SEND msg_to_send;

MSG_TYPE receive_pos_message(Stream *port,uint8_t* header){
  
  static int fpos = 0;
  static uint8_t checksum[2];
  static int payloadSize =0;
  static unsigned char cls=0x00;
  static unsigned char id = 0x00;
  static MSG_TYPE msg_type = undefined;
  static unsigned char* buf ;
  

  while ( port->available() ) {
    byte c = port->read();
//    Serial.print(c);Serial.print(",");
    if ( fpos < 2 ) {
      if ( c == header[fpos] )
        fpos++;
      else
        fpos = 0;
        
       msg_type = undefined;
    }
    else if (fpos == 2){
      cls = c;
      fpos++;
    }
    else if (fpos == 3){
      id = c;
      msg_type = undefined;
     if (cls == CLASS_NAV && id == MSG_POSLLH){
        msg_type =  nav_posllh ;
        payloadSize = sizeof(UBX_NAV_POSLLH);
        buf = ((unsigned char*)(&navposllh));
     }
     else if (cls == CLASS_NAV && id == MSG_STATUS){
       msg_type = nav_status ;
       payloadSize = sizeof(UBX_NAV_STATUS);
       buf = ((unsigned char*)(&navstatus));
     }
     else if (cls == CLASS_NAV && id == MSG_SOL){
        msg_type = nav_solution;
        payloadSize = sizeof(UBX_NAV_SOL);
        buf = ((unsigned char*)(&navsolution));
        
     }
     if (msg_type != undefined)
     {
      buf[0] = cls;
      buf[1] = id;
      fpos++;
     }
     else 
      fpos=0;
      
    }
    else if (msg_type != undefined) {
      if ( (fpos-2) < payloadSize )
        buf[fpos-2] = c;

      fpos++;

      if ( fpos == (payloadSize+2) ) {
        //calcChecksum(checksum);
        calcChecksum(checksum,payloadSize,buf);
        
      }
      else if ( fpos == (payloadSize+3) ) {
        if ( c != checksum[0] )
          fpos = 0;
      }
      else if ( fpos == (payloadSize+4) ) {
        fpos = 0;
        if ( c == checksum[1] ) {
          return msg_type;
        }
      }
      else if ( fpos > (payloadSize+4) ) {
        fpos = 0;
      }
    }
  }
  return undefined;
}

#define GPS_SERIAL Serial1

const uint8_t UBX_HEADER[] = { PREAMBLE1, PREAMBLE2 }; 
POS_INFO pos_info;

RH_ASK driver;

void setup() {
  Serial.begin(9600);
  GPS_SERIAL.begin(9600);

  //Initialize RF_Transmitter
   if (!driver.init())
         Serial.println("init failed");
         
  // config GPS_rate
//  Serial.print("\nconfigure gps: set rate to 10 HZ");
//  UBX_CFG_RATE cfg_rate;
//  cfg_rate.meas_rate = 100; // 100 ms -> 10 HZ
//  cfg_rate.nav_rate  = 1;
//  cfg_rate.time_ref  = 0;  //UTC time
  
//  sendMessage(&GPS_SERIAL,UBX_HEADER,((uint8_t*)(&cfg_rate)), sizeof(UBX_CFG_RATE));
  
//  delay(100);
//  
//  UBX_CFG_MSG_RATE cfg_msg_rate;
//
//  // Enable nav posllh msg with rate 1 cycl
//  Serial.print("\nconfigure gps:enable ubx_nav_posllh message");
//  cfg_msg_rate.msg_cls = CLASS_NAV;
//  cfg_msg_rate.msg_id = MSG_POSLLH;
//  cfg_msg_rate.rate = 1;
  
//  sendMessage(&GPS_SERIAL,UBX_HEADER,((uint8_t*)(&cfg_msg_rate)), sizeof(UBX_CFG_MSG_RATE));
  
//  delay(100);
//
// // Enable nav status msg with rate 1 cycl
//  Serial.print("\nconfigure gps:enable ubx_nav_status message");
//  cfg_msg_rate.msg_cls = CLASS_NAV;
//  cfg_msg_rate.msg_id = MSG_STATUS;
//  cfg_msg_rate.rate = 1;

//  sendMessage(&GPS_SERIAL,UBX_HEADER,((uint8_t*)(&cfg_msg_rate)), sizeof(UBX_CFG_MSG_RATE));
  
  // Set dynamic model 
//  delay(100);
//  Serial.print("\nconfigure gps: set dynamic model to automotive");
//  UBX_CFG_NAV_SETTINGS nav_settings;
// 
//  nav_settings.mask = 1;
//  nav_settings.dynModel = AUTOMOTIVE;
//  nav_settings.fixMode = 3; // AUTO
  
//  sendMessage(&GPS_SERIAL,UBX_HEADER,((uint8_t*)(&nav_settings)), sizeof(UBX_CFG_NAV_SETTINGS));
  

  delay(100);
}
void loop() {

  unsigned long now_millis = millis();

  MSG_TYPE posmtype = receive_pos_message(&GPS_SERIAL,UBX_HEADER);
// Serial.println(posmtype);
  switch(posmtype)
    {
      case nav_posllh:
      
        pos_info.gps_time = navposllh.time;
        
        pos_info.lat = (double)navposllh.latitude/10000000.0d;
        pos_info.lon = (double)navposllh.longitude/10000000.0d;
        pos_info.height = (float)navposllh.altitude_ellipsoid/1000.0f;
        pos_info.dt = 0.001 * (float)(now_millis-pos_info.lastupdate);
        pos_info.lastupdate = now_millis;
        pos_info.last_lat = pos_info.lat;
        pos_info.last_lon = pos_info.lon;
        pos_info.flag_update_pos = true;

        break;
      case nav_status:
        pos_info.fix_type = navstatus.fix_type;
        pos_info.fix_ok = 1<<0 & navstatus.fix_status; 
        break;
      case nav_solution:
        pos_info.fix_type = navsolution.fix_type;
        pos_info.used_sats = navsolution.satellites;
        pos_info.fix_ok = 1<<0 & navsolution.fix_status; 
        break;
    }
    char msg[25];
    String str_message = "";

    str_message += navposllh.latitude;
    str_message += ",";
    str_message += navposllh.longitude;

    strcpy(msg, str_message.c_str());
    
    driver.send( (uint8_t *)msg, strlen(msg));
    driver.waitPacketSent();

    if(pos_info.flag_update_pos){
      pos_info.flag_update_pos = false;
      Serial.print("Lat:");Serial.print( navposllh.latitude,DEC);
      Serial.print("\tlon:");Serial.print( navposllh.longitude,DEC);
      Serial.print("\theight:");Serial.print( pos_info.height,DEC);
      Serial.println();
    }
}
