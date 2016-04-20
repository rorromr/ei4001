#include <SerialDXL.h>

MMap::UInt8NV foo_nv(0, 0, MMap::Access::RW, 10, 60, 13);

MMap::UInt8 foo(1, MMap::Access::RW, 10, 200, 25);
MMap::UInt8 bar(2, MMap::Access::RW, 10, 200, 50);


uint8_t mmap_buf[] = {0x00,0x00,0x00,0x00};
MMap::VariablePtr var_list[3];

MMap::MMap mem_map(3);


void setup() {
  Serial.begin(9600);
  mem_map.init(mmap_buf, var_list);
  foo_nv.save();// Save default
}

void loop() {
  uint8_t offset = 0;
  offset += foo.serialize(mmap_buf+offset);
  offset += bar.serialize(mmap_buf+offset);
  offset += foo_nv.serialize(mmap_buf+offset);
  Serial.println(mmap_buf[0],DEC);
  Serial.println(mmap_buf[1],DEC);
  Serial.println(mmap_buf[2],DEC);
  Serial.print("Offset: "); Serial.println(offset);
  Serial.print("EEPROM: "); 
  Serial.println(eeprom_read_byte( (uint8_t*)(uint16_t) 5),DEC);
  Serial.println("---");
  delay(1000);
}
