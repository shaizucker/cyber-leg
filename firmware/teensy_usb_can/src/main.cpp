// USB CDC SLCAN adapter, classic CAN only. CAN3 TX=31, RX=30.
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "slcan_frame.h"

FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> canBus;
static uint32_t bitrate = 1000000;
static bool opened = false, listenOnly = false, connected = false;
static uint8_t stickyStatus = 0;
static char command[48];
static size_t commandLength = 0;
static bool discardCommand = false;

static void reply(bool success) {
    Serial.write(success ? '\r' : '\a');
    Serial.send_now();
}

static void closeCan() {
    // Only this mailbox is used for TX; discard a still-pending transmission.
    canBus.setMB(MB8, TX, STD);
    canBus.FLEXCAN_EnterFreezeMode();
    opened = false;
    listenOnly = false;
    digitalWrite(LED_BUILTIN, LOW);
}

static void openCan(bool passive) {
    canBus.begin();
    canBus.setBaudRate(bitrate, passive ? LISTEN_ONLY : TX);
    canBus.setMaxMB(16);
    canBus.enableFIFO();
    canBus.setFIFOFilter(ACCEPT_ALL);
    canBus.setMB(MB8, TX, STD);
    opened = true;
    listenOnly = passive;
    digitalWrite(LED_BUILTIN, HIGH);
}

static bool transmit(const char *line) {
    slcan::Frame parsed;
    if (!opened || listenOnly || !slcan::parse(line, parsed)) return false;
    // Do not call write on a busy mailbox: FlexCAN otherwise queues the frame.
    if (FLEXCAN_get_code(FLEXCANb_MBn_CS(CAN3, 8)) != FLEXCAN_MB_CODE_TX_INACTIVE) {
        stickyStatus |= 0x02;
        return false;
    }
    CAN_message_t frame = {};
    frame.id = parsed.id;
    frame.len = parsed.len;
    frame.flags.extended = parsed.extended;
    frame.flags.remote = parsed.remote;
    memcpy(frame.buf, parsed.data, parsed.len);
    return canBus.write(MB8, frame) == 1;
}

static void handle(const char *line) {
    const size_t size = strlen(line);
    if (size == 1 && line[0] == 'C') { closeCan(); reply(true); return; }
    if (size == 1 && (line[0] == 'O' || line[0] == 'L')) {
        if (opened) { reply(false); return; }
        openCan(line[0] == 'L'); reply(true); return;
    }
    if (size == 2 && line[0] == 'S' && !opened) {
        static const uint32_t speeds[] = {10000,20000,50000,100000,125000,250000,500000,800000,1000000};
        const int index = line[1]-'0';
        if (index >= 0 && index <= 8) { bitrate = speeds[index]; reply(true); return; }
    }
    if (size == 1 && line[0] == 'V') { Serial.print("V0101"); reply(true); return; }
    if (size == 1 && line[0] == 'N') { Serial.print("N0041"); reply(true); return; }
    if (size == 1 && line[0] == 'F') {
        const uint32_t esr = FLEXCANb_ESR1(CAN3), ecr = FLEXCANb_ECR(CAN3);
        if (esr & 0x300) stickyStatus |= 0x04;
        if ((ecr & 0xFF) >= 128 || ((ecr>>8)&0xFF) >= 128) stickyStatus |= 0x20;
        if ((esr & 0x30) >= 0x20) stickyStatus |= 0x80;
        Serial.printf("F%02X", stickyStatus); stickyStatus = 0; reply(true); return;
    }
    if (size == 1 && line[0] == 'D') {
        // Read-only bench diagnostics; not needed by the standard SLCAN host.
        Serial.printf("D MCR=%08lX CTRL1=%08lX ESR1=%08lX ECR=%08lX MB8=%08lX CLOCKSEL=%08lX",
            (unsigned long)FLEXCANb_MCR(CAN3), (unsigned long)FLEXCANb_CTRL1(CAN3),
            (unsigned long)FLEXCANb_ESR1(CAN3), (unsigned long)FLEXCANb_ECR(CAN3),
            (unsigned long)FLEXCANb_MBn_CS(CAN3,8), (unsigned long)CCM_CSCMR2);
        reply(true); return;
    }
    // No timestamps or acceptance-filter emulation: reject unsupported requests.
    if (size == 2 && line[0] == 'Z' && line[1] == '0' && !opened) { reply(true); return; }
    if (line[0] == 't' || line[0] == 'T' || line[0] == 'r' || line[0] == 'R') {
        reply(transmit(line)); return;
    }
    reply(false);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(115200); // USB CDC transport, not a physical UART baud rate.
    canBus.begin();
    canBus.setBaudRate(bitrate, LISTEN_ONLY);
    closeCan(); // No startup banner and no CAN transmission at boot.
}

void loop() {
    const bool hostConnected = bool(Serial);
    if (!hostConnected) {
        if (connected) {
            closeCan(); commandLength = 0; discardCommand = false;
        }
        connected = false;
        return;
    }
    connected = true;
    // Bound USB command work so incoming CAN is also serviced under load.
    for (unsigned count=0; count<64 && Serial.available(); ++count) {
        const char value = Serial.read();
        if (value == '\r') {
            if (discardCommand) reply(false);
            else if (commandLength) {
                command[commandLength] = '\0'; handle(command);
            }
            commandLength = 0; discardCommand = false;
        } else if (value != '\n' && !discardCommand) {
            if (value < 32 || value > 126 || commandLength >= sizeof(command)-1) discardCommand = true;
            else command[commandLength++] = value;
        }
    }
    if (!opened) return;
    CAN_message_t received;
    for (unsigned count=0; count<32 && canBus.read(received); ++count) {
        if (received.len > 8) { stickyStatus |= 0x08; continue; }
        if (received.flags.overrun) stickyStatus |= 0x08;
        slcan::Frame frame;
        frame.id = received.id; frame.len = received.len;
        frame.extended = received.flags.extended; frame.remote = received.flags.remote;
        memcpy(frame.data, received.buf, frame.len);
        char output[32];
        const size_t length = slcan::format(frame, output);
        if (Serial.availableForWrite() < int(length)) { stickyStatus |= 0x01; continue; }
        Serial.write(reinterpret_cast<const uint8_t *>(output), length);
        Serial.send_now(); // Flush each frame promptly, avoiding idle batching.
    }
}
