const unsigned short CHAR = 8;
const unsigned short xdim = 16;
const unsigned short ydim = 3;

const int GATES[] = {10, 11, 12, 13, A0, A1};
const int SOURCES[] = {2, 3, 4, 5, 6, 7, 8, 9};
const unsigned short gates = sizeof(GATES) / sizeof(GATES[0]);
const unsigned short sources = sizeof(SOURCES) / sizeof(SOURCES[0]);

class G {
public:
  G (unsigned int gate, unsigned int source) : gate(gate), source(source) {};
  unsigned int gate;
  unsigned int source;
};

const G MAPPING[xdim][ydim] = {{G(4, 5), G(5, 7), G(4, 4)}, {G(5, 6), G(5, 4), G(5, 5)}, {G(3, 7), G(3, 5), G(3, 6)}, {G(3, 4), G(4, 6), G(4, 7)}, {G(1, 5), G(2, 7), G(1, 4)}, {G(2, 6), G(2, 4), G(2, 5)}, {G(0, 7), G(0, 5), G(0, 6)}, {G(0, 4), G(1, 6), G(1, 7)}, {G(4, 1), G(5, 3), G(4, 0)}, {G(5, 2), G(5, 0), G(5, 1)}, {G(3, 3), G(3, 1), G(3, 2)}, {G(3, 0), G(4, 2), G(4, 3)}, {G(1, 1), G(2, 3), G(1, 0)}, {G(2, 2), G(2, 0), G(2, 1)}, {G(0, 3), G(0, 1), G(0, 2)}, {G(0, 0), G(1, 2), G(1, 3)}};

const unsigned short bits_per_part = CHAR;
const unsigned short gate_map_parts = xdim * ydim / bits_per_part;

byte GATE_MAPS[gates][gate_map_parts];
unsigned short SOURCE_MAPS[gates][sources][2];
unsigned short SOURCE_STATES[gates];

void generate_maps () {
  for (unsigned int g = 0; g < gates; g++) {
    unsigned short n = 0;
    for (unsigned short part = 0; part < gate_map_parts; part++) {    
      GATE_MAPS[g][part] = 0;
      for (byte b = 0; b < bits_per_part; b++) {
        const unsigned short c = b + (part * bits_per_part);
        const unsigned short x = c % xdim;
        const unsigned short y = c / xdim;
  
        if (MAPPING[x][y].gate == g) {
          GATE_MAPS[g][part] |= 0x1 << b;
          SOURCE_MAPS[g][n][0] = c;
          SOURCE_MAPS[g][n][1] = MAPPING[x][y].source;
          n++;
        }
      }
    }
  }
}

void calc_source_states (const byte* image) {
  for (unsigned short g = 0; g < gates; g++) {
    SOURCE_STATES[g] = 0;
    unsigned int next_pixel = 0;
    
    for (byte part = 0; part < gate_map_parts; part++) {
      if (SOURCE_MAPS[g][next_pixel][0] >= (part + 1) * bits_per_part) {
        continue;
      }
      const byte masked = image[part] & GATE_MAPS[g][part];
      for (unsigned int n = next_pixel; n < sources; n++) {
        const unsigned short img_idx = SOURCE_MAPS[g][n][0];       
        if (img_idx >= (part + 1) * bits_per_part) {
          break;
        }

        const unsigned short source = SOURCE_MAPS[g][n][1];
        SOURCE_STATES[g] |= (0x1 & (masked >> (img_idx % bits_per_part))) << source;
        next_pixel++;
      }
    }
  }
}

void update_port_registers (const unsigned short gate, const bool disable = false) {
  // Direct output register access to save cycles
  PORTD = (PORTD & B00000011) | (B11111100 & ~(SOURCE_STATES[gate] << 2));
  PORTB = (~SOURCE_STATES[gate] >> 6) | B11111100;
  PORTC = PORTC | B00000011;

  if (!disable) {
    // This should be about as fast as manually shifting gate pins above
    digitalWrite(GATES[gate], LOW);
  }
}

void parse_serial_buffer (const byte* buffer) {
  static const unsigned short len = xdim * ydim / bits_per_part;
  byte parsed[len];

  for (unsigned int i = 0; i < len; i++) {
    for (unsigned int j = 0; j < bits_per_part; j++) {
      bool val = buffer[i * bits_per_part + j] == '1';
      parsed[i] |= ((byte) val) << j;
    }
  }
  
  calc_source_states(parsed);
}

const byte imgdata_ones[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

void setup () {  
  Serial.begin(115200);
  Serial.setTimeout(1); // We use this as the sleep in the loop. Hopefully it kinda works
    
  for (auto& g : GATES) {
    pinMode(g, OUTPUT);
    digitalWrite(g, HIGH);
  }

  for (auto& s : SOURCES) {
    pinMode(s, OUTPUT);
    digitalWrite(s, HIGH);
  }

  generate_maps();
  calc_source_states(imgdata_ones); 
}


// Test data
/*
const byte imgdata_zeros[] = {B00000000, B00000000, B00000000, B00000000, B00000000, B00000000};
const byte imgdata_pattern[] = {B01100101, B01100101, B01100101, B01100101, B01100101, B01100101};
*/

int gate = 0;
byte serial_buffer[48];

void loop () { 
  update_port_registers(gate);
  gate = (gate + 1) % 8;
  
  auto res = Serial.readBytesUntil('\n', serial_buffer, 48);
  if (res != 0) {
    parse_serial_buffer(serial_buffer);
  }
}
