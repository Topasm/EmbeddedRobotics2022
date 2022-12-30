
#define TX_Enable_pin (12)  // 송신 활성화 핀
#define RX_Enable_pin (13)  // 수신 활성화 핀
#define DX_ID (0x07)        // 서보모터 아이디

#include <SoftwareSerial.h>

String RCVdata;
String CMDdataDEG[3];
int CMDdataVal[3] = { 2047, 2047, 2047 };
SoftwareSerial mySerial(2, 3);

typedef enum _com_mode {
  RX_MODE,
  TX_MODE
} com_mode;

// 통신 모드(송/수신)에 따른 버퍼칩 설정
void set_com_mode(com_mode mode) {

  if (mode == RX_MODE) {
    // 비활성화 먼저 수행하여 동시에 활성화 되는 순간을 방지
    digitalWrite(TX_Enable_pin, LOW);   // TX disable
    digitalWrite(RX_Enable_pin, HIGH);  // RX Enable
  } else {
    // 비활성화 먼저 수행하여 동시에 활성화 되는 순간을 방지
    digitalWrite(RX_Enable_pin, LOW);   // RX disable
    digitalWrite(TX_Enable_pin, HIGH);  // TX Enable
  }
}

// 통신 프로토콜에 체크섬 삽입
void dx_insert_checksum_byte(unsigned char *packet) {

  unsigned char i;
  unsigned char checksum_pt;
  unsigned char checksum;
  unsigned char packet_length;

  packet_length = packet[3];  // 3번 바이트에 패킷 길이가 저장되어 있음
  checksum_pt = packet_length + 3;

  checksum = 0x00;
  for (i = 2; i < checksum_pt; i++) {
    checksum += packet[i];
  }
  packet[checksum_pt] = ~checksum;
}

// id 설정을 위한 통신 패킷 조립
void dx_set_id(unsigned char id) {

  unsigned char packet[8];
  unsigned char i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = 0xFD;
  packet[3] = 0x00;
  packet[4] = 254;
  packet[5] = 0x04;
  packet[6] = 0x00;
  packet[7] = 0x03;
  packet[8] = id;

  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE);  // 송신 모드로 설정
  for (i = 0; i < 8; i++) {
    Serial.write(packet[i]);
  }

  // 송신완료시까지 블록
  while (!(UCSR0A & (1 << TXC0))) {
    __asm__("nop\n\t");  // no operation
  }

  set_com_mode(RX_MODE);  // 수신 모드로 설정
}

// 제어 모드 설정
// 정방향과 역방향 최대 각도에 따라 바퀴모드나 관절모드로 설정됨
// 바퀴모드: 모두 0으로 설정
// 관절모드: 0~360(0xfff)로 설정
void dx_set_control_mode(unsigned char id,
                         unsigned char cw_angle_limit[2],
                         unsigned char ccw_angle_limit[2]) {

  unsigned char packet[11];
  unsigned char i;

  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x07;
  packet[4] = 0x03;
  packet[5] = 0x06;
  packet[6] = cw_angle_limit[0];
  packet[7] = cw_angle_limit[1];
  packet[8] = ccw_angle_limit[0];
  packet[9] = ccw_angle_limit[1];
  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE);  // 송신 모드로 설정
  for (i = 0; i < 11; i++) {
    Serial.write(packet[i]);
  }

  // 송신완료시까지 블록
  while (!(UCSR0A & (1 << TXC0))) {
    __asm__("nop\n\t");  // no operation
  }

  set_com_mode(RX_MODE);  // 수신 모드로 설정
}


// 위치제어용 패킷 조립
void dx_tx_packet_for_position_control(unsigned char id, unsigned int goal_pos) {

  unsigned char packet[11];
  unsigned char i;
  packet[0] = 0xFF;
  packet[1] = 0xFF;
  packet[2] = id;
  packet[3] = 0x07;
  packet[4] = 0x03;
  packet[5] = 0x1E;
  packet[6] = byte(goal_pos);
  packet[7] = byte((goal_pos & 0x0F00) >> 8);
  packet[8] = 0x00;
  packet[9] = 0x00;
  dx_insert_checksum_byte(packet);

  set_com_mode(TX_MODE);  // 송신 모드로 설정
  for (i = 0; i < 11; i++) {
    mySerial.write(packet[i]);
  }


  delay(10);
  set_com_mode(RX_MODE);  // 수신 모드로 설정
}




// 송신완료시까지 블록



void setup() {

  // put your setup code here, to run once:
  Serial.begin(115200);            // 통신 속도
                                   //  Serial.begin(115200);  // 통신 속도
                                   //  Serial.begin(19200);  // 통신 속도
  // pinMode(TX_Enable_pin, OUTPUT);  //TX Enable
  // pinMode(RX_Enable_pin, OUTPUT);  //RX Enable
  mySerial.begin(57600);

  // unsigned char cw_angle_limit[2];
  // unsigned char ccw_angle_limit[2];

  //  dx_set_id(DX_ID);
  //  delay(1);

  // 관절 모드로 설정
  // cw_angle_limit[0] = 0x00;
  // cw_angle_limit[1] = 0x00;
  // ccw_angle_limit[0] = 0xFF;
  // ccw_angle_limit[1] = 0x0F;
  // //delay(1);

  // dx_set_control_mode(8, cw_angle_limit, ccw_angle_limit);
  // dx_set_control_mode(9, cw_angle_limit, ccw_angle_limit);
  // dx_set_control_mode(10, cw_angle_limit, ccw_angle_limit);
  delay(1000);
}


void loop() {

  // Serial.read()
  if (Serial.available() <= 0) {
    return;
  }
  char ch = Serial.read();
  RCVdata += ch;
  if (ch == '\n') {

    if (RCVdata.length() > 0) {
      int index;
      int tmpcnt = 0;
      String tmpString = RCVdata;
      tmpString.trim();
      Serial.print("command in deg   ");
      Serial.println(tmpString);

      while (tmpString.length() > 0) {
        index = tmpString.indexOf(",");
        if (index == -1) {
          CMDdataDEG[tmpcnt] = tmpString;
          CMDdataDEG[tmpcnt].trim();
          tmpcnt++;
          break;
        }

        CMDdataDEG[tmpcnt] = tmpString.substring(0, index);
        tmpString = tmpString.substring(index + 1);
        tmpString.trim();
        CMDdataDEG[tmpcnt].trim();
        tmpcnt++;
      }
    }
    int i = 0;
  for (i = 0; i < 3; i++) {

    CMDdataVal[i] = map(CMDdataDEG[i].toInt(), 0, 360, 0, 4095);
    if (CMDdataVal[i] > 4095) {
      CMDdataVal[i] = 4095;
    } else if (CMDdataVal[i] < 0) {
      CMDdataVal[i] = 0;
    }
    Serial.print("motor input");
    Serial.println(i);
    Serial.println(CMDdataVal[i]);
  }
    RCVdata = "";
  }

  dx_tx_packet_for_position_control(8, CMDdataVal[0]);
  dx_tx_packet_for_position_control(9, CMDdataVal[1]);
  dx_tx_packet_for_position_control(10, CMDdataVal[2]);

}