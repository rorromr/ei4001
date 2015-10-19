#include <stdio.h>
#include <stdint.h>

// Mesage reception state
uint8_t msgState_=0;
uint8_t msgParamIdx_=2;
uint8_t msgLen_=0;
uint8_t msgFinish_=0;
// Buffer
uint8_t rxMsgBuf_[15];
// ID
uint8_t ID = 20;

void print_buffer()
{
  int i;
  printf("[");
  for (i = 0; i < 14; ++i)
  {
    printf("%i,", rxMsgBuf_[i]);
  }
  printf("%i]\n", rxMsgBuf_[14]);
}

void process(uint8_t data)
{
  switch(msgState_)
  {
    case 0: // 0xFF
      msgState_ = data == 0xFF ? 1 : 0;
      break;
      
    case 1: // 0XFF
      msgState_ = data == 0xFF ? 2 : 1;
      break;
      
    case 2: // ID
      // Check error
      msgState_ = data == ID ? 3 : 0;
      break;
      
    case 3: // Length
      msgLen_ = data;
      // Save length in the RX message buffer
      rxMsgBuf_[0] = data;
      msgState_ = 4;
      break;

    case 4: // Instruction
      // Save instruction in the RX message buffer
      rxMsgBuf_[1] = data;
      // Check for short message
      msgState_ = msgLen_ <= 2 ? 6 : 5;
      break;
    
    case 5: // Parameters
      rxMsgBuf_[msgParamIdx_++] = data;
      // Check message length
      msgState_ = msgParamIdx_ >= msgLen_ ? 6 : msgState_;
      break;

    case 6: // Checksum
      // @TODO Checksum
      rxMsgBuf_[msgParamIdx_] = data;
      msgFinish_ = 1;
      msgState_ = 0;
      msgParamIdx_ = 2;
      msgLen_ = 0;
      break;
  }
}

void reset()
{
  msgState_=0;
  msgParamIdx_=2;
  msgLen_=0;
  msgFinish_=0;
}

void init_buffer()
{
  int i;
  for (i = 0; i < 15; ++i)
  {
    rxMsgBuf_[i] = 0;
  }
}

void exec_test(uint8_t *test, uint8_t length)
{
  int i;
  for (i = 0; i < length; ++i)
  {
    process(test[i]);
  }
}

int main() {
  // Init buffer
  uint8_t test1[] = {0Xff, 0xff, ID, 4, 1, 2, 3, 5, 6, 8};
  uint8_t test2[] = {0Xff, 0xff, ID, 4, 1, 2, 3, 5, 6, 8};

  printf("\nTEST 1\n");
  init_buffer();
  print_buffer();
  exec_test(test1, 8);
  print_buffer();
  //reset();

  printf("\nTEST 2\n");
  init_buffer();
  print_buffer();
  exec_test(test2, 8);
  print_buffer();

  return(0);
}