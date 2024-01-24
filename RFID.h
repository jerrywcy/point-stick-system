#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#define MAX_GPIO 39

class CLRC663 {
  private:
    uint8_t CS, RST, IRQ;

    const uint8_t REG_COMMAND = 0x00;

    const uint8_t COMMAND_IDLE = 0x00;

    void begin_communication() {
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWrite(CS, LOW);
    }

    void end_communication() {
        digitalWrite(CS, HIGH);
        SPI.endTransaction();
    }

  public:
    CLRC663(uint8_t cs, uint8_t rst = -1, uint8_t irq = -1) {
        CS = cs;
        pinMode(CS, OUTPUT);
        if ((rst >= 0) && (rst <= MAX_GPIO)) {
            RST = rst;
            pinMode(rst, OUTPUT);
            digitalWrite(rst, LOW);
        } else
            RST = -1;

        if ((irq >= 0) && (irq <= MAX_GPIO)) {
            IRQ = irq;
            pinMode(irq, INPUT);
        } else
            IRQ = -1;
    }

    enum class Register : uint8_t {
        Command = 0x00,     // Starts and stops command execution
        HostCtrl = 0x01,    // Host control register
        FIFOControl = 0x02, // Control register of the FIFO
        WaterLevel = 0x03,  // Level of the FIFO underflow and overflow warning
        FIFOLength = 0x04,  // Length of the FIFO
        FIFOData = 0x05,    // Data In/Out exchange register of FIFO buffer
        IRQ0 = 0x06,        // Interrupt register 0
        IRQ1 = 0x07,        // Interrupt register 1
        IRQ0En = 0x08,      // Interrupt enable register 0
        IRQ1En = 0x09,      // Interrupt enable register 1
        Error = 0x0A,       // Error bits showing the error status of the last
                            // command execution
        Status = 0x0B,      // Contains status of the communication
        RxBitCtrl = 0x0C,   // Control register for anticollision adjustments
                            // for bit oriented protocols
        RxColl = 0x0D,      // Collision position register
        TControl = 0x0E,    // Control of Timer 0..3
        T0Control = 0x0F,   // Control of Timer0
        T0ReloadHi = 0x10,  // High register of the reload value of Timer0
        T0ReloadLo = 0x11,  // Low register of the reload value of Timer0
        T0CounterValHi = 0x12, // Counter value high register of Timer0
        T0CounterValLo = 0x13, // Counter value low register of Timer0
        T1Control = 0x14,      // Control of Timer1
        T1ReloadHi = 0x15,     // High register of the reload value of Timer1
        T1ReloadLo = 0x16,     // Low register of the reload value of Timer1
        T1CounterValHi = 0x17, // Counter value high register of Timer1
        T1CounterValLo = 0x18, // Counter value low register of Timer1
        T2Control = 0x19,      // Control of Timer2
        T2ReloadHi = 0x1A,     // High byte of the reload value of Timer2
        T2ReloadLo = 0x1B,     // Low byte of the reload value of Timer2
        T2CounterValHi = 0x1C, // Counter value high byte of Timer2
        T2CounterValLo = 0x1D, // Counter value low byte of Timer2
        T3Control = 0x1E,      // Control of Timer3
        T3ReloadHi = 0x1F,     // High byte of the reload value of Timer3
        T3ReloadLo = 0x20,     // Low byte of the reload value of Timer3
        T3CounterValHi = 0x21, // Counter value high byte of Timer3
        T3CounterValLo = 0x22, // Counter value low byte of Timer3
        T4Control = 0x23,      // Control of Timer4
        T4ReloadHi = 0x24,     // High byte of the reload value of Timer4
        T4ReloadLo = 0x25,     // Low byte of the reload value of Timer4
        T4CounterValHi = 0x26, // Counter value high byte of Timer4
        T4CounterValLo = 0x27, // Counter value low byte of Timer4
        DrvMode = 0x28,        // Driver mode register
        TxAmp = 0x29,          // Transmitter amplifier register
        DrvCon = 0x2A,         // Driver configuration register
        Txl = 0x2B,            // Transmitter register
        TxCrcPreset = 0x2C,    // Transmitter CRC control register, preset value
        RxCrcPreset = 0x2D,    // Receiver CRC control register, preset value
        TxDataNum = 0x2E,      // Transmitter data number register
        TxModWidth = 0x2F,     // Transmitter modulation width register
        TxSym10BurstLen =
            0x30, // Transmitter symbol 1 + symbol 0 burst length register
        TXWaitCtrl = 0x31,  // Transmitter wait control
        TxWaitLo = 0x32,    // Transmitter wait low
        FrameCon = 0x33,    // Transmitter frame control
        RxSofD = 0x34,      // Receiver start of frame detection
        RxCtrl = 0x35,      // Receiver control register
        RxWait = 0x36,      // Receiver wait register
        RxThreshold = 0x37, // Receiver threshold register
        Rcv = 0x38,         // Receiver register
        RxAna = 0x39,       // Receiver analog register
        LPCD_Option =
            0x3A, // No function implemented for CLRC66301 and CLRC66302
                  // CLRC66303: Options for LPCD configuration
        SerialSpeed = 0x3B, // Serial speed register
        LFO_Trimm = 0x3C,   // Low-power oscillator trimming register
        PLL_Ctrl = 0x3D,    // IntegerN PLL control register, for
                            // microcontroller clock output adjustment
        PLL_DivOut = 0x3E,  // IntegerN PLL control register, for
                            // microcontroller clock output adjustment
        LPCD_QMin =
            0x3F, // Low-power card detection Q channel minimum threshold
        LPCD_QMax =
            0x40, // Low-power card detection Q channel maximum threshold
        LPCD_IMin =
            0x41, // Low-power card detection I channel minimum threshold
        LPCD_I_Result =
            0x42, // Low-power card detection I channel result register
        LPCD_Q_Result =
            0x43,          // Low-power card detection Q channel result register
        PadEn = 0x44,      // PIN enable register
        PadOut = 0x45,     // PIN out register
        PadIn = 0x46,      // PIN in register
        SigOut = 0x47,     // Enables and controls the SIGOUT Pin
        TxBitMod = 0x48,   // Transmitter bit mode register
        RFU = 0x49,        // -
        TxDataCon = 0x4A,  // Transmitter data configuration register
        TxDataMod = 0x4B,  // Transmitter data modulation register
        TxSymFreq = 0x4C,  // Transmitter symbol frequency
        TxSym0H = 0x4D,    // Transmitter symbol 0 high register
        TxSym0L = 0x4E,    // Transmitter symbol 0 low register
        TxSym1H = 0x4F,    // Transmitter symbol 1 high register
        TxSym1L = 0x50,    // Transmitter symbol 1 low register
        TxSym2 = 0x51,     // Transmitter symbol 2 register
        TxSym3 = 0x52,     // Transmitter symbol 3 register
        TxSym10Len = 0x53, // Transmitter symbol 1 + symbol 0 length register
        TxSym32Len = 0x54, // Transmitter symbol 3 + symbol 2 length register
        TxSym10BurstCtrl =
            0x55, // Transmitter symbol 1 + symbol 0 burst control register
        TxSym10Mod =
            0x56, // Transmitter symbol 1 + symbol 0 modulation register
        TxSym32Mod =
            0x57,        // Transmitter symbol 3 + symbol 2 modulation register
        RxBitMod = 0x58, // Receiver bit modulation register
        RxEofSym = 0x59, // Receiver end of frame symbol register
        RxSyncValH = 0x5A, // Receiver synchronisation value high register
        RxSyncValL = 0x5B, // Receiver synchronisation value low register
        RxSyncMod = 0x5C,  // Receiver synchronisation mode register
        RxMod = 0x5D,      // Receiver modulation register
        RxCorr = 0x5E,     // Receiver correlation register
        FabCal = 0x5F,     // Calibration register of the receiver, calibration
                           // performed at production
        Version = 0x7F,    // Version and subversion register
    };

    enum class Command : uint8_t {
        idle = 0x00,      // This command indicates that the CLRC663 is in idle
                          // mode. This command is also used to terminate the
                          // actual command.
        lpcd = 0x01,      // This command activates the low-power card detection
                          // mode.
        load_key = 0x02,  // This command is used to load a key into the
                          // CLRC663. The key is stored in the EEPROM of the
                          // CLRC663.
        MFAuthnet = 0x03, // This command performs the MIFARE standard
                          // authentication.
        ackreq = 0x04,    // This command is used to perform an Acknowledge
                          // request.
        receive = 0x05,   // This command activates the receive circuitry.
        transmit = 0x06,  // This command transmits data from FIFO buffer.
        transceive =
            0x07,        // This command transmits data from FIFO buffer and
                         // automatically activates the receiver after a
                         // transmission is finished. Each transmission process
                         // starts by writing the command into CommandReg.
        write_e2 = 0x08, // This command is used to write data into the
                         // EEPROM of the CLRC663.
        write_e2_page = 0x09, // This command is used
                              // to write data into
                              // the EEPROM of the
                              // CLRC663.
        read_e2 = 0x0A,       // This command is used to read data from the
                              // EEPROM of the CLRC663.
        load_reg = 0x0C,      // This command is used to load a complete
                              // register set from the EEPROM into the
                              // CLRC663. The register set is stored in the
                              // EEPROM of the CLRC663.
        load_protocol = 0x0D, // This command is used to load a protocol
                              // into the CLRC663. The protocol is stored in
                              // the EEPROM of the CLRC663.
        load_key_e2 = 0x0E,   // This command is used to load a key into the
                              // CLRC663. The key is stored in the EEPROM of the
                              // CLRC663.
        store_key_e2 = 0x0F,  // This command is used to store a key into the
                              // EEPROM of the CLRC663.
        get_random = 0x1C,    // This command is used to get a random number.
        soft_reset = 0x1F, // This command is performing a soft reset. Triggered
                           // by this command all the default values for the
                           // register setting will be read from the EEPROM and
                           // copied into the register set.
    };

    void begin() {
        SPI.begin();
        reset();
    }

    void end() { SPI.end(); }

    void reset() {
        if (RST != -1) {
            hard_reset();
        } else {
            soft_reset();
        }
    }

    void hard_reset() {
        if (RST != -1) {
            digitalWrite(RST, HIGH);
            digitalWrite(RST, LOW);
            digitalWrite(RST, HIGH);
            digitalWrite(RST, LOW);
            delay(10);
        }
    }

    void soft_reset() {
        run_command(Command::soft_reset);
        delay(10);
    }

    void write_reg(Register reg, uint8_t value) {
        begin_communication();
        SPI.transfer((uint8_t(reg) << 1) | 0x00);
        SPI.transfer(value);
        end_communication();
    }

    void write_regs(Register reg, uint8_t *values, uint8_t len) {
        uint8_t instruction_tx[len + 1];
        instruction_tx[0] = ((uint8_t(reg) << 1) | 0x00);
        for (uint8_t i = 0; i < len; i++) {
            instruction_tx[i + 1] = values[i];
        }
        // SPI transport
        begin_communication();
        for (uint8_t i = 0; i < (len + 1); i++) {
            SPI.transfer(instruction_tx[i]);
        }
        end_communication();
    }

    void write_fifo(uint8_t *data, uint8_t len) {
        for (uint8_t i = 0; i < len; i++) {
            write_reg(Register::FIFOData, data[i]);
        }
    }

    uint8_t read_reg(Register reg) {
        uint8_t instruction_tx[2] = {uint8_t((uint8_t(reg) << 1) | 0x01), 0};
        uint8_t res = 0;
        begin_communication();
        SPI.transfer(instruction_tx[0]);
        res = SPI.transfer(instruction_tx[1]);
        end_communication();
        return res;
    }

    uint8_t fifo_length() { return read_reg(Register::FIFOLength); }

    void read_fifo(uint8_t *data, uint8_t len) {
        for (uint8_t i = 0; i < len; i++) {
            data[i] = read_reg(Register::FIFOData);
        }
    }

    uint8_t read_irq0() { return read_reg(Register::IRQ0); }

    void run_command(Command command) {
        write_reg(Register::Command, uint8_t(command));
    }

    void load_register(uint8_t EEPROM_address_high, uint8_t EEPROM_address_low,
                       uint8_t register_address, uint8_t length) {
        uint8_t values[4] = {EEPROM_address_high, EEPROM_address_low,
                             register_address, length};
        write_fifo(values, 4);
        run_command(Command::load_reg);
    }

    void flush_fifo() { write_reg(Register::FIFOControl, 0xB0); }

    void clear_irq0() { write_reg(Register::IRQ0, ~(1 << 7)); }

    void clear_irq1() { write_reg(Register::IRQ1, ~(1 << 7)); }

    void load_protocol() {
        uint8_t values[2] = {0x0A, 0x0A};
        write_regs(Register::FIFOData, values, 2);
        run_command(Command::load_protocol);
    }

    void inventory(uint8_t flags, uint8_t mask_length = 0x00,
                   uint8_t mask = 0x00) {
        uint8_t data[4] = {flags, 0x01, mask_length, mask};
        write_fifo(data, 4);
        run_command(Command::transceive);
    }

    uint16_t read_tag(uint8_t *data, uint32_t timeout = 51) {
        run_command(Command::idle);
        flush_fifo();
        load_protocol();
        flush_fifo();
        write_reg(Register::DrvMode, 0x8E);
        load_register(0x01, 0x94, 0x28, 0x12);
        run_command(Command::idle);
        clear_irq0();
        inventory(0x06);
        uint32_t start_time = millis();
        uint8_t uid[10];
        for (int count = 0; count < 16; count++) {
            if (read_irq0() & 0x01) {
                clear_irq0();
                read_fifo(uid, fifo_length());
            }
            write_reg(Register::FrameCon, 0x0C);
            write_reg(Register::TxDataNum, 0x00);
            flush_fifo();
            run_command(Command::transceive);
        }
        write_reg(Register::DrvMode, 0x86);
    }
};
