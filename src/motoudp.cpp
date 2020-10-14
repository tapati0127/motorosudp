#include "../include/motorosudp/motoudp.h"
#include <QUdpSocket>
#include <QString>
#include <QByteArray>
#include <QFile>


MotoUDP::MotoUDP(QHostAddress h,quint16 p)
{
    _HostAddress = h;
    _port = p;
    rx_buffer = new QByteArray;
    rx_file_buffer = new QByteArray;
    tx_file_buffer = new QByteArray;
}
MotoUDP::~MotoUDP()
{
  delete rx_buffer;
  delete rx_file_buffer;
  delete tx_file_buffer;
}
struct MotoUDP::MotoUDP::TxData {
    char identifier[4] = {'Y','E','R','C'};
    u_int16_t header_size = 32;
    u_int16_t data_size;
    u_int8_t reserve1 = 3;
    u_int8_t processing_division = 1;
    u_int8_t ack = 0;
    u_int8_t id;
    u_int32_t block_no = 0;
    char reserve2[8] = {'9','9','9','9','9','9','9','9'};
    u_int16_t command_no;
    u_int16_t instance;
    u_int8_t attribute;
    u_int8_t service;
    const u_int16_t padding = 0;
   // QByteArray data;
};
struct MotoUDP::MotoUDP::TxDataWritePosition {
  const u_int32_t control_group_robot = 1;
  const u_int32_t control_group_station = 0;
  u_int32_t classification_in_speed = 0;
  u_int32_t speed;
  const uint32_t coordinate = 0x10;
  int32_t x;
  int32_t y;
  int32_t z;
  int32_t rx;
  int32_t ry;
  int32_t rz;
  const u_int32_t reservation1 = 0;
  const u_int32_t reservation2 = 0;
  const u_int32_t type = 0;
  const u_int32_t expanded_type = 0;
  const u_int32_t tool_no = 0;
  const u_int32_t user_coordinate_no = 0;
  const u_int32_t base_1_position = 0;
  const u_int32_t base_2_position = 0;
  const u_int32_t base_3_position = 0;
  const u_int32_t station_1_position = 0;
  const u_int32_t station_2_position = 0;
  const u_int32_t station_3_position = 0;
  const u_int32_t station_4_position = 0;
  const u_int32_t station_5_position = 0;
  const u_int32_t station_6_position = 0;
};
struct MotoUDP::MotoUDP::TxDataWritePulse {
  const u_int32_t control_group_robot = 1;
  const u_int32_t control_group_station = 0;
  u_int32_t classification_in_speed = 0;
  u_int32_t speed;
  int32_t r1;
  int32_t r2;
  int32_t r3;
  int32_t r4;
  int32_t r5;
  int32_t r6;
  int32_t r7;
  int32_t r8;
  const u_int32_t tool_no = 0;
  const u_int32_t base_1_position = 0;
  const u_int32_t base_2_position = 0;
  const u_int32_t base_3_position = 0;
  const u_int32_t station_1_position = 0;
  const u_int32_t station_2_position = 0;
  const u_int32_t station_3_position = 0;
  const u_int32_t station_4_position = 0;
  const u_int32_t station_5_position = 0;
  const u_int32_t station_6_position = 0;
};
struct MotoUDP::MotoUDP::TxDataWriteVariablePosition
{
  u_int32_t data_type;
  const u_int32_t figure = 0;
  const u_int32_t tool_no = 0;
  const u_int32_t user_coodirnate_no = 0;
  const u_int32_t extended_figure = 0;
  int32_t first_axis_position;
  int32_t second_axis_position;
  int32_t third_axis_position;
  int32_t fourth_axis_position;
  int32_t fifth_axis_position;
  int32_t sixth_axis_position;
  const int32_t seventh_axis_position = 0;
  const int32_t eighth_axis_position = 0;
};


bool MotoUDP::MotoUDP::ConnectMotoman()
{
    client = new QUdpSocket;
    client->bind();
    return 1;
}
bool MotoUDP::MotoUDP::CloseMotoman()
{
    client->close();
    delete  client;
    return 1;
}
QString MotoUDP::MotoUDP::SendCommand(QByteArray buffer)
{
    isDataReceive = false;
    client->writeDatagram(buffer,buffer.length(),_HostAddress,_port);
    return ByteArray2Hex(buffer);
}

QString MotoUDP::MotoUDP::ByteArray2Hex(QByteArray buffer)
{
    QString s;
    for (int i = 0; i < buffer.size(); i++)
    {
        QString c = QString::number(buffer.at(i),16);
        if(c.size()<2)
        {
            s.push_back('0');
            s.push_back(c);
        }
        else{
            s.push_back(c.at(c.size()-2));
            s.push_back(c.at(c.size()-1));
        }
        s.push_back(' ');
    }
    s = s.toUpper();
    return s;
}

QByteArray MotoUDP::MotoUDP::Hex2ByteArray (QString s)
{
    QByteArray buffer;
    for (int i = 0; i < s.size(); i = i+3)
    {
        QString temp;
        temp.push_back(s.at(i));
        temp.push_back(s.at(i+1));
        bool ok;
        char x = temp.toInt(&ok,16);
        if(ok == 1)
        {
            buffer.push_back(x);
        }
        else
            return 0;
    }
    return buffer;
}

QByteArray MotoUDP::MotoUDP::Int32ToByteArray (int32_t value)
{
    QByteArray buffer;
    for (uint i = 0;i < sizeof (int32_t); i++) {
        char u = (value>>8*i) & 0xFF;
        buffer.push_back(u);
    }
    return buffer;
}
int32_t MotoUDP::MotoUDP::ByteArray2Int32 (QByteArray* buffer,int start, int number)
{
    int32_t value = 0;
    for (int i = start;i < start+number; i++) {
        int32_t temp = (unsigned char) buffer->at(i) ;
        value += temp << 8*(i-start);
    }
    return value;

}
bool MotoUDP::MotoUDP::SendData (char* buffer, int lenght)
{
  isDataReceive = false;
  client->writeDatagram(buffer,lenght,_HostAddress,_port);
  return 1;
}
bool  MotoUDP::MotoUDP::TurnOnServo()
{
    TxData sent_data;
    u_int32_t data = 1;
    sent_data.id = RECEIVE_TYPE::ON_SERVO;
    sent_data.command_no = 0x83;
    sent_data.instance = 2;
    sent_data.attribute = 1;
    sent_data.service = 0x10;
    u_int16_t data_length = sizeof (data);
    u_int16_t total_length = 32 + data_length;
    sent_data.data_size = data_length;
    char buffer [total_length];
    memcpy(buffer,&sent_data,32);
    memcpy(buffer+32,&data,data_length);
    SendData(buffer,total_length);
    return 1;
}
bool MotoUDP::MotoUDP::TurnOffServo()
{
  TxData sent_data;
  u_int32_t data = 2;
  sent_data.id = RECEIVE_TYPE::OFF_SERVO;
  sent_data.command_no = 0x83;
  sent_data.instance = 2;
  sent_data.attribute = 1;
  sent_data.service = 0x10;
  u_int16_t data_length = sizeof (data);
  u_int16_t total_length = 32 + data_length;
  sent_data.data_size = data_length;
  char buffer[total_length];
  memcpy(buffer,&sent_data,32);
  memcpy(buffer+32,&data,data_length);
  SendData(buffer,total_length);
    return 1;
}
bool MotoUDP::MotoUDP::GetPosition()
{
  TxData sent_data;
  char buffer [sizeof (sent_data)];
  sent_data.id = RECEIVE_TYPE::GET_POSITION;
  sent_data.command_no = 0x75;
  sent_data.instance = 0x65;
  sent_data.attribute = 0;
  sent_data.service = 0x01;
  sent_data.data_size = 0;
  memcpy(buffer,&sent_data,sizeof (sent_data));
  SendData(buffer,sizeof (sent_data));
  return 1;
}
bool MotoUDP::MotoUDP::GetPulsePosition()
{
  TxData sent_data;
  char buffer [sizeof (sent_data)];
  sent_data.id = RECEIVE_TYPE::GET_PULSE;
  sent_data.command_no = 0x75;
  sent_data.instance = 0x01;
  sent_data.attribute = 0;
  sent_data.service = 0x01;
  sent_data.data_size = 0;
  memcpy(buffer,&sent_data,sizeof (sent_data));
  SendData(buffer,sizeof (sent_data));
  return 1;
}
bool MotoUDP::MotoUDP::WritePosition(u_int16_t type,u_int32_t classification_in_speed,  u_int32_t speed,int32_t* pos)
{
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::WRITE_POSITION;
  sent_data.command_no = 0x8A;
  sent_data.instance = type;
  sent_data.attribute = 01;
  sent_data.service = 0x02;

  TxDataWritePosition position;
  position.classification_in_speed = classification_in_speed;
  position.x = *pos;
  position.y = *(pos+1);
  position.z = *(pos+2);
  position.rx = *(pos+3);
  position.ry = *(pos+4);
  position.rz = *(pos+5);
  position.speed = speed;


  sent_data.data_size = sizeof(position);
  char buffer[sizeof(sent_data)+sizeof (position)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof (sent_data),&position,sizeof (position));
  SendData(buffer,sizeof(sent_data)+sizeof (position));
  return 1;
}
bool MotoUDP::MotoUDP::WritePulse(u_int16_t type,u_int32_t classification_in_speed, u_int32_t speed,int32_t* pos)
{
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::WRITE_PUSLE;
  sent_data.command_no = 0x8B;
  sent_data.instance = type;
  sent_data.attribute = 01;
  sent_data.service = 0x02;

  TxDataWritePulse position;
  position.classification_in_speed = classification_in_speed;
  position.r1 = *pos;
  position.r2 = *(pos+1);
  position.r3 = *(pos+2);
  position.r4 = *(pos+3);
  position.r5 = *(pos+4);
  position.r6 = *(pos+5);
  position.speed = speed;
  sent_data.data_size = sizeof(position);
  char buffer [sizeof(sent_data)+sizeof (position)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof (sent_data),&position,sizeof (position));
  SendData(buffer,sizeof(sent_data)+sizeof (position));
  return 1;
}
bool MotoUDP::MotoUDP::GetVarPosition(u_int16_t index)
{
  TxData sent_data;
  char buffer [sizeof (sent_data)];
  sent_data.id = 06;
  sent_data.command_no = 0x7f;
  sent_data.instance = index;
  sent_data.attribute = 0;
  sent_data.service = 0x0e;
  sent_data.data_size = 0;

  memcpy(buffer,&sent_data,sizeof (sent_data));
  SendData(buffer,sizeof (sent_data));
  return true;
}
bool MotoUDP::MotoUDP::WriteVarPosition(u_int16_t index, int32_t X,int32_t Y,int32_t Z,int32_t RX,int32_t RY,int32_t RZ)
{
  TxData sent_data;
  sent_data.id = 07;
  sent_data.command_no = 0x7f;
  sent_data.instance = index;
  sent_data.attribute = 0;
  sent_data.service = 0x10;
  TxDataWriteVariablePosition position;
  sent_data.data_size = sizeof (position);
  position.data_type = 17;
  position.first_axis_position = X;
  position.second_axis_position = Y;
  position.third_axis_position = Z;
  position.fourth_axis_position = RX;
  position.fifth_axis_position = RY;
  position.sixth_axis_position = RZ;
  char buffer[sizeof(sent_data)+ sizeof (position)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),&position,sizeof(position));
  SendData(buffer,sizeof(sent_data)+ sizeof (position));
  return 1;
}
bool MotoUDP::MotoUDP::SelectJob(char* jobname){
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::JOB_SELLECT;
  sent_data.command_no = 0x87;
  sent_data.instance = 1;
  sent_data.attribute = 0;
  sent_data.service = 0x02;
  sent_data.data_size = 36;
  u_int32_t line = 0;
  char buffer [sizeof(sent_data)+ 36];

  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),jobname,32);
  memcpy(buffer+sizeof(sent_data)+32,&line,4);
  SendData(buffer,sizeof(sent_data)+36);
  return 1;
}
bool MotoUDP::MotoUDP::StartJob(){
  TxData sent_data;
  sent_data.id = 9;
  sent_data.command_no = 0x86;
  sent_data.instance = 1;
  sent_data.attribute = 1;
  sent_data.service = 0x10;
  u_int32_t data = 1;
  sent_data.data_size = sizeof (data);
  char buffer [sizeof(sent_data)+ sizeof(data)];
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),&data,sizeof(data));
  SendData(buffer,sizeof(sent_data)+sizeof(data));
  return 1;
}
void MotoUDP::MotoUDP::ReceiveData()
{
    rx_buffer->resize(client->pendingDatagramSize());
    client->readDatagram(rx_buffer->data(),rx_buffer->size());

    qDebug() << rx_buffer->toHex();
    if(GetReceiveType() == GET_POSITION && rx_buffer->at(26)==0)
    {
      memcpy(current_position,rx_buffer->data()+52,24);
    }
    else if (GetReceiveType() == GET_PULSE && rx_buffer->at(26)==0) {
      memcpy(current_pulse,rx_buffer->data()+52,24);
    }
    else if (GetReceiveType() == FILE_RECEIVE && rx_buffer->at(26)==0) {
      for (int i = 32;i<rx_buffer->size();i++) {
        rx_file_buffer->push_back(rx_buffer->at(i));
//        qDebug() << rx_buffer->toHex();
      }
      TxData header;
      memcpy(&header,rx_buffer,32);
      header.command_no = 0;
      header.instance = 0;
      header.attribute = 0;
      header.service = 0x16;
      header.data_size = 0;
      char data[32];
      memcpy(data,&header,32);
      client->writeDatagram(data,32,_HostAddress,_port+1);
    }
    else if (GetReceiveType() == FILE_TRANSMIT&& rx_buffer->at(26)==0) {
      if(last_data==true)
      {
        return;
      }
      index_file_transmit++;
      TxData header;
      header.processing_division = 2;
      header.command_no = 0;
      header.instance = 0;
      header.attribute = 0;
      header.service = 0x15;
      header.ack = 1;
      header.id = FILE_TRANSMIT;
      header.block_no = index_file_transmit;
      char data[64];

      u_int byte_number = 32;
      if(index_file_transmit==max_index_file_transmit){
        byte_number = last_byte_number;
        header.block_no = index_file_transmit+0x80000000;
        last_data = true;
      }
      header.data_size = byte_number;
      memcpy(data,&header,32);
      memcpy(data+32,tx_file_buffer->data()+(index_file_transmit-1)*32,byte_number);
//      for (int i = 0;i<32+byte_number;i++) {
//        qDebug() <<(data[i]);
//      }

      client->writeDatagram(data,32+byte_number,_HostAddress,_port+1);
    }

    //rx_data = ByteArray2Hex(*rx_buffer);
    //isDataReceive = true;
}
bool MotoUDP::MotoUDP::FileReceiveCommand(char name[]){
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::FILE_RECEIVE;
  sent_data.processing_division = 2;
  sent_data.command_no = 0;
  sent_data.instance = 0;
  sent_data.attribute = 0;
  sent_data.service = 0x16;
  sent_data.data_size = 12;//
  char buffer[64];//
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),name,12);//

  client->writeDatagram(buffer,32+12,_HostAddress,_port+1);
  rx_file_buffer->resize(0);
}
bool MotoUDP::MotoUDP::FileTransmitCommand(char name[]){
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::FILE_TRANSMIT;
  sent_data.processing_division = 2;
  sent_data.command_no = 0;
  sent_data.instance = 0;
  sent_data.attribute = 0;
  sent_data.service = 0x15;
  sent_data.data_size = 13;//
  char buffer[64];//
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),name,13);//
  client->writeDatagram(buffer,32+13,_HostAddress,_port+1);//
  max_index_file_transmit = tx_file_buffer->size()/32;
  if(tx_file_buffer->size()%32!=0)
  {
    max_index_file_transmit ++;
    last_byte_number=tx_file_buffer->size()%32;
  }
  index_file_transmit = 0;
  last_data = false;
}
bool MotoUDP::MotoUDP::FileDeleteCommand(char name[]){
  TxData sent_data;
  sent_data.id = RECEIVE_TYPE::FILE_DELETE;
  sent_data.processing_division = 2;
  sent_data.command_no = 0;
  sent_data.instance = 0;
  sent_data.attribute = 0;
  sent_data.service = 0x9;
  sent_data.data_size = 13;//
  char buffer[64];//
  memcpy(buffer,&sent_data,sizeof (sent_data));
  memcpy(buffer+sizeof(sent_data),name,13);//
  client->writeDatagram(buffer,32+13,_HostAddress,_port+1);//
}
bool MotoUDP::MotoUDP::GetJobFile(QString path){
  //"/home/tapati/motoman_ws/src/motorosudp/TEST0407.JBI"
  QFile file(path);
  file.open(QIODevice::ReadWrite);
  file.write(*rx_file_buffer);
  file.close();
}
bool MotoUDP::MotoUDP::JobFile2ByteArray(QString path)
{
  //"/home/tapati/motoman_ws/src/motorosudp/TEST0407.JBI"
  QFile file(path);
  file.open(QIODevice::ReadWrite);
  *tx_file_buffer = file.readAll();
  file.close();
}
QByteArray* MotoUDP::MotoUDP::Get_rx_buffer()
{
    return rx_buffer;
}
MotoUDP::MotoUDP::RECEIVE_TYPE MotoUDP::MotoUDP::GetReceiveType ()
{
    return RECEIVE_TYPE(rx_buffer->at(11));
}
bool MotoUDP::MotoUDP::CheckReceivedData(QByteArray buffer)
{
    QByteArray abuffer;
    bool check_number_data, check_header_1, check_header_2, check_header_3 ;
    abuffer = SplitArray(buffer,6,2);
    check_number_data = ByteArray2Int32(&abuffer,0,2)==Get_rx_buffer()->size()-32;
    check_header_1 = ByteArray2Hex(SplitArray(buffer,0,6)) == "59 45 52 43 20 00 ";
    check_header_2 = ByteArray2Hex(SplitArray(buffer,8,3)) == "03 01 01 ";
    check_header_3 = ByteArray2Hex(SplitArray(buffer,12,12)) == "00 00 00 80 39 39 39 39 39 39 39 39 ";
    return  check_number_data&&check_header_1&&check_header_2&&check_header_3;
}
QByteArray MotoUDP::MotoUDP::SplitArray(QByteArray array,int start, int count)
{
    QByteArray temp_array;
    for (int i = start;i<start+count;i++) {
        temp_array.push_back(array.at(i));
    }
    return temp_array;
}
int32_t MotoUDP::MotoUDP::Joint2Pulse(double joint, int i)
{

    if(i==0)
    {
      return int32_t(PULSE_PER_DEGREE_S*joint);
    }
    else if (i==1) {
      return int32_t(PULSE_PER_DEGREE_L*joint);
    }
    else if (i==2) {
      return int32_t(PULSE_PER_DEGREE_U*joint);
    }
    else {
      return int32_t(PULSE_PER_DEGREE_RBT*joint);
    }
}
double MotoUDP::MotoUDP::Pulse2Joint(int32_t pulse, int i)
{
  if(i==0)
  {
    return double(pulse)/PULSE_PER_DEGREE_S;
  }
  else if (i==1) {
    return double(pulse)/PULSE_PER_DEGREE_L;
  }
  else if (i==2) {
    return double(pulse)/PULSE_PER_DEGREE_U;
  }
  else {
    return double(pulse)/PULSE_PER_DEGREE_RBT;
  }
}

std::vector<double> MotoUDP::MotoUDP::ByteArray2Joint(QByteArray *pulse_buffer)
{
  std::vector<double> pulse;
  for (int i = 0; i < 6;i++) {
    pulse.push_back(Pulse2Joint(ByteArray2Int32(pulse_buffer,4*i+52,4),i)*M_PI/180);
  }
  return pulse;
}
int32_t*  MotoUDP::MotoUDP::GetCurrentPosition(){
  return current_position;
}
int32_t*  MotoUDP::MotoUDP::GetCurrentPulse(){
  return current_pulse;
}
bool MotoUDP::MotoUDP::ConnectToPLC(QHostAddress host, u_int port,uint16_t adr,uint16_t no_reg,std::vector<uint16_t> data)
{
  struct tx{
        uint8_t Command_type = 0x11;
        uint8_t Identification_number = 0;
        uint16_t channel_number = 0;
        uint16_t not_use_1 = 0;
        uint16_t total_lenght;
        uint32_t not_use_2 = 0;
        uint16_t data_lenght;
        uint16_t MFC_SFC = 0x0B20;
        uint16_t CPU_number = 16;
        uint16_t reference_address ;
        uint16_t number_register ;
        uint16_t data_reg[];
      } tx_data;
      tx_data.reference_address = adr;
      tx_data.number_register = no_reg;
      memcpy(tx_data.data_reg,data.data(),data.size());
      tx_data.data_lenght = 8 + 2*data.size();
      tx_data.total_lenght = 14+tx_data.data_lenght;
      char buffer[tx_data.total_lenght];
      memcpy(buffer,&tx_data,sizeof(tx_data));
      client->writeDatagram(buffer,sizeof(tx_data),host,port);
      return true;
}

const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_S = 34816/30;
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_L = 102400/90;
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_U = 51200/90;
const double MotoUDP::MotoUDP::PULSE_PER_DEGREE_RBT = 10204/30;

