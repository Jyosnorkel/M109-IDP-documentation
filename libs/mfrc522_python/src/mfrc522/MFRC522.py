class MFRC522:
    MFRC522_addr = 0x28  # =40

    MAX_LEN = 16

    # Proximity Coupling Device
    PCD_IDLE = 0x00
    PCD_AUTHENT = 0x0E
    PCD_RECEIVE = 0x08
    PCD_TRANSMIT = 0x04
    PCD_TRANSCEIVE = 0x0C
    PCD_RESETPHASE = 0x0F
    PCD_CALCCRC = 0x03

    # Proximity Integrated Circuit Card
    PICC_REQIDL = 0x26
    PICC_REQALL = 0x52
    PICC_ANTICOLL = 0x93
    PICC_SElECTTAG = 0x93
    PICC_AUTHENT1A = 0x60
    PICC_AUTHENT1B = 0x61
    PICC_READ = 0x30
    PICC_WRITE = 0xA0
    PICC_DECREMENT = 0xC0
    PICC_INCREMENT = 0xC1
    PICC_RESTORE = 0xC2
    PICC_TRANSFER = 0xB0
    PICC_HALT = 0x50

    # Status
    MI_OK = 0
    MI_NOTAGERR = 1
    MI_ERR = 2

    # MFRC522 Registers Addresses
    Reserved00 = 0x00
    CommandReg = 0x01
    CommIEnReg = 0x02
    DivlEnReg = 0x03
    CommIrqReg = 0x04
    DivIrqReg = 0x05
    ErrorReg = 0x06
    Status1Reg = 0x07
    Status2Reg = 0x08
    FIFODataReg = 0x09
    FIFOLevelReg = 0x0A
    WaterLevelReg = 0x0B
    ControlReg = 0x0C
    BitFramingReg = 0x0D
    CollReg = 0x0E
    Reserved01 = 0x0F

    Reserved10 = 0x10
    ModeReg = 0x11
    TxModeReg = 0x12
    RxModeReg = 0x13
    TxControlReg = 0x14
    TxAutoReg = 0x15
    TxSelReg = 0x16
    RxSelReg = 0x17
    RxThresholdReg = 0x18
    DemodReg = 0x19
    Reserved11 = 0x1A
    Reserved12 = 0x1B
    MifareReg = 0x1C
    Reserved13 = 0x1D
    Reserved14 = 0x1E
    SerialSpeedReg = 0x1F

    Reserved20 = 0x20
    CRCResultRegM = 0x21
    CRCResultRegL = 0x22
    Reserved21 = 0x23
    ModWidthReg = 0x24
    Reserved22 = 0x25
    RFCfgReg = 0x26
    GsNReg = 0x27
    CWGsPReg = 0x28
    ModGsPReg = 0x29
    TModeReg = 0x2A
    TPrescalerReg = 0x2B
    TReloadRegH = 0x2C
    TReloadRegL = 0x2D
    TCounterValueRegH = 0x2E
    TCounterValueRegL = 0x2F

    Reserved30 = 0x30
    TestSel1Reg = 0x31
    TestSel2Reg = 0x32
    TestPinEnReg = 0x33
    TestPinValueReg = 0x34
    TestBusReg = 0x35
    AutoTestReg = 0x36
    VersionReg = 0x37
    AnalogTestReg = 0x38
    TestDAC1Reg = 0x39
    TestDAC2Reg = 0x3A
    TestADCReg = 0x3B
    Reserved31 = 0x3C
    Reserved32 = 0x3D
    Reserved33 = 0x3E
    Reserved34 = 0x3F
    serNum = []

    def __init__(self, i2c_bus=0, i2c_address=MFRC522_addr, debugLevel='WARNING'):
        """Initialize the RFID reader."""
        # Store I2C
        self.i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        if type(i2c_bus) == "<class 'SoftI2C'>":
            self.i2c_bus.start()

        # TODO do we need this??
        self.StopAuth = self.StopCrypto1
        
        # Initialize MFRC522 RFID reader
        self.Init()

    def Reset(self):
        """Reset the RFID reader."""

        self.WriteReg(self.CommandReg, self.PCD_RESETPHASE)

    def WriteReg(self, addr, val):
        """Write a value to a reader register."""
        self.i2c_bus.writeto(self.i2c_address, bytearray([addr, val]))

    def ReadReg(self, addr):
        """Read a value from a reader register."""

        self.i2c_bus.writeto(self.i2c_address, bytearray([addr]))
        val = self.i2c_bus.readfrom(self.i2c_address, 1)
        return int.from_bytes(val)

    def Close(self):
        """Close the reader interface."""
        #self.spi.close()
        #GPIO.cleanup()
        pass

    def SetBitMask(self, reg, mask):
        """Set bits in a reader register."""
        # Read the current value of the register
        tmp = self.ReadReg(reg)

        # Set the desired bits using a bitwise OR operation
        self.WriteReg(reg, tmp | mask)

    def ClearBitMask(self, reg, mask):
        """Clear bits in a reader register."""
        # Read the current value of the register
        tmp = self.ReadReg(reg)

        # Clear the desired bits using a bitwise AND operation
        self.WriteReg(reg, tmp & (~mask))

    def AntennaOn(self):
        """Turn the RFID antenna on."""
        # Read the current value of the TxControlReg register
        temp = self.ReadReg(self.TxControlReg)

        # Check if the least significant two bits are already set
        if (temp & 0x03) != 0x03:
            # If not, turn on the antenna by setting the bits using a bit mask
            self.SetBitMask(self.TxControlReg, 0x03)

    def AntennaOff(self):
        """
        Turns off the antenna of an MFRC522 RFID module by clearing the TxControlReg register.

        """
        # Clear the least significant two bits of the TxControlReg register to turn off the antenna
        self.ClearBitMask(self.TxControlReg, 0x03)

    def MFRC522_ToCard(self, command, sendData):
        """Send a command to a card and return its response."""
        backData = []  # List to store response data
        backLen = 0  # Length of response data
        status = self.MI_ERR  # Default status
        irqEn = 0x00  # Interrupt request enable flag
        waitIRq = 0x00  # Wait for interrupt request flag
        lastBits = None  # Number of valid bits in last byte
        n = 0  # Number of bytes received

        # Set interrupt request and wait flags based on command
        if command == self.PCD_AUTHENT:
            irqEn = 0x12
            waitIRq = 0x10
        if command == self.PCD_TRANSCEIVE:
            irqEn = 0x77
            waitIRq = 0x30

        # Enable interrupts and reset FIFO buffer
        self.WriteReg(self.CommIEnReg, irqEn | 0x80)
        self.ClearBitMask(self.CommIrqReg, 0x80)
        self.SetBitMask(self.FIFOLevelReg, 0x80)

        # Put MFRC522 into idle state
        self.WriteReg(self.CommandReg, self.PCD_IDLE)

        # Write data to FIFO buffer
        for i in range(len(sendData)):
            self.WriteReg(self.FIFODataReg, sendData[i])

        # Start command execution
        self.WriteReg(self.CommandReg, command)

        # Set bit framing if command is transceive
        if command == self.PCD_TRANSCEIVE:
            self.SetBitMask(self.BitFramingReg, 0x80)

        # Wait for command execution (timeout)
        i = 2000
        while True:
            n = self.ReadReg(self.CommIrqReg)
            i -= 1
            # Break if interrupt request received or timeout
            if i == 0 or (n & 0x01) or (n & waitIRq):
                break
        # print(f"i={i}, n={n} (n&0x01={n&0x01}), waitIrq={waitIRq} (n&waitIRq={n&waitIRq})")

        # Clear bit framing if command is transceive
        self.ClearBitMask(self.BitFramingReg, 0x80)

        # Check for errors and update status accordingly
        if i != 0:
            errorReg = self.ReadReg(self.ErrorReg)
            if (errorReg & 0x1B) == 0x00:
                status = self.MI_OK

                if n & irqEn & 0x01:
                    # print(f"n={n}, irqEn={irqEn}")
                    status = self.MI_NOTAGERR

                # Read response data if command is transceive
                if command == self.PCD_TRANSCEIVE:
                    n = self.ReadReg(self.FIFOLevelReg)
                    lastBits = self.ReadReg(self.ControlReg) & 0x07
                    if lastBits != 0:
                        backLen = (n - 1) * 8 + lastBits
                    else:
                        backLen = n * 8

                    if n == 0:
                        n = 1
                    if n > self.MAX_LEN:
                        n = self.MAX_LEN

                    for i in range(n):
                        backData.append(self.ReadReg(self.FIFODataReg))
            else:
                print(f"errorReg={errorReg}")
                status = self.MI_ERR

        # Return response data, length, and status
        return (status, backData, backLen)

    def Request(self, reqMode):
        """Request a response from a nearby card."""
        status = None
        backBits = None
        TagType = []

        # Set the bit framing register to 0x07
        self.WriteReg(self.BitFramingReg, 0x07)

        # Append the request mode to the TagType list
        TagType.append(reqMode)

        # Send the request to the card using the MFRC522_ToCard method
        (status, backData, backBits) = self.MFRC522_ToCard(
            self.PCD_TRANSCEIVE, TagType)

        # If the status is not MI_OK or the back bits are not 0x10, set status to MI_ERR
        if ((status != self.MI_OK) | (backBits != 0x10)):
            #print(f"Request() Error, status={status}, backBits={backBits:02x}")
            status = self.MI_ERR

        # Return a tuple containing the status, back bits and tag type
        return (status, backBits)

    def Anticoll(self):
        """Resolve card collisions and read the card UID."""
        backData = []
        serNumCheck = 0

        serNum = []

        # Set the BitFramingReg to 0x00
        self.WriteReg(self.BitFramingReg, 0x00)

        # Append the PICC_ANTICOLL command and 0x20 to the serNum list
        serNum.append(self.PICC_ANTICOLL)
        serNum.append(0x20)

        # Call the MFRC522_ToCard method with PCD_TRANSCEIVE command and serNum data
        (status, backData, backBits) = self.MFRC522_ToCard(
            self.PCD_TRANSCEIVE, serNum)

        # Check if the operation was successful
        if (status == self.MI_OK):
            i = 0
            # Check if the backData has the expected length of 5 bytes
            if len(backData) == 5:
                # Calculate the XOR checksum of the first 4 bytes of the backData
                for i in range(4):
                    serNumCheck = serNumCheck ^ backData[i]
                # Check if the calculated checksum matches the 5th byte of backData
                if serNumCheck != backData[4]:
                    print(f"serNumCheck={serNumCheck} != backData[4]={backData[4]}")
                    # If not, set the status to MI_ERR
                    status = self.MI_ERR
            else:
                # If backData doesn't have 5 bytes, set the status to MI_ERR
                print(f"len(backData)={len(backData) != 5}")
                status = self.MI_ERR

        # Return the status and backData
        return (status, backData)

    def CalulateCRC(self, pIndata):
        """Calculate a CRC for the supplied data."""

        # Clear the CRC IRQ flag and set the FIFO level to maximum.
        self.ClearBitMask(self.DivIrqReg, 0x04)
        self.SetBitMask(self.FIFOLevelReg, 0x80)

        # Write the input data to the FIFO.
        for i in range(len(pIndata)):
            self.WriteReg(self.FIFODataReg, pIndata[i])

        # Start the CRC calculation command.
        self.WriteReg(self.CommandReg, self.PCD_CALCCRC)

        # Wait for the CRC calculation to complete.
        i = 0xFF
        while True:
            n = self.ReadReg(self.DivIrqReg)
            i -= 1
            if not ((i != 0) and not (n & 0x04)):
                break

        # Read the calculated CRC value from the chip.
        pOutData = []
        pOutData.append(self.ReadReg(self.CRCResultRegL))
        pOutData.append(self.ReadReg(self.CRCResultRegM))
        return pOutData

    def SelectTag(self, serNum):
        """Select a card by its UID."""
        # Initialize empty lists for the response data and the data buffer
        backData = []
        buf = []

        # Add the command byte and tag type to the buffer
        buf.append(self.PICC_SElECTTAG)
        buf.append(0x70)

        # Add the serial number of the tag to the buffer
        for i in range(5):
            buf.append(serNum[i])

        # Calculate the CRC values for the buffer and add them to the buffer
        pOut = self.CalulateCRC(buf)
        buf.append(pOut[0])
        buf.append(pOut[1])

        # Send the buffer to the tag and receive the response
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_TRANSCEIVE, buf)

        # Check if the response is successful and has the expected length
        if (status == self.MI_OK) and (backLen == 0x18):
            # Log the size of the response and return the first byte of the response
            #self.logger.debug("Size: " + str(backData[0]))
            return backData[0]
        else:
            # Return 0 if the response is not successful or has an unexpected length
            print(f"SelectTag error: status={status}, backlen={backLen}")
            return 0

    def Authenticate(self, authMode, BlockAddr, Sectorkey, serNum):
        """Authenticate a card block."""
        buff = []

        # First byte should be the authMode (A or B)
        buff.append(authMode)

        # Second byte is the trailerBlock (usually 7)
        buff.append(BlockAddr)

        # Now we need to append the authKey which usually is 6 bytes of 0xFF
        for i in range(len(Sectorkey)):
            buff.append(Sectorkey[i])

        # Next we append the first 4 bytes of the UID
        for i in range(4):
            buff.append(serNum[i])

        # Now we start the authentication itself
        (status, backData, backLen) = self.MFRC522_ToCard(self.PCD_AUTHENT, buff)
        
        # Check if an error occurred
        if not (status == self.MI_OK):
            raise ValueError(f"AUTH ERROR status={status}")
        if not (self.ReadReg(self.Status2Reg) & 0x08) != 0:
            raise ValueError(f"AUTH ERROR(status2reg={self.Status2Reg} & 0x08) != 0")

        # Return the status
        return status

    def StopCrypto1(self):
        """
        Stops the authentication process.
        """
        self.ClearBitMask(self.Status2Reg, 0x08)

    def ReadTag(self, blockAddr):
        """Read data from a card block."""

        # create an array containing the READ command and the block address to be read
        recvData = []
        recvData.append(self.PICC_READ)
        recvData.append(blockAddr)
        # calculate the CRC checksum for the command and block address
        pOut = self.CalulateCRC(recvData)
        # append the calculated checksum to the command and block address array
        recvData.append(pOut[0])
        recvData.append(pOut[1])
        # send the command and block address array to the RFID card and receive response
        (status, backData, backLen) = self.MFRC522_ToCard(
            self.PCD_TRANSCEIVE, recvData)
        # if response status is not OK, raise error
        if not (status == self.MI_OK):
            raise ValueError(f"Error while reading, status={status}")

        # if response data has length 16, print debug message and return data
        if len(backData) == 16:
            # print("Sector " + str(blockAddr) + " " + str(backData))
            return backData
        # if response data length is not 16, return None
        else:
            raise ValueError(f"Invalid length {len(BackData)}")

    def WriteTag(self, blockAddr, writeData):
        """Write data to a card block."""

        # The buffer to be sent to the tag for writing data
        buff = []
        buff.append(self.PICC_WRITE)
        buff.append(blockAddr)

        # Calculate the CRC checksum for the buffer
        crc = self.CalulateCRC(buff)
        buff.append(crc[0])
        buff.append(crc[1])

        # Send the buffer to the tag and receive the response
        (status, backData, backLen) = self.MFRC522_ToCard(
            self.PCD_TRANSCEIVE, buff)

        # Check if the write operation was successful or not
        if not (status == self.MI_OK) or not (backLen == 4) or not ((backData[0] & 0x0F) == 0x0A):
            status = self.MI_ERR

        #self.logger.debug("%s backdata &0x0F == 0x0A %s" %
        #                  (backLen, backData[0] & 0x0F))

        # If the initial write operation was successful, write the actual data to the tag
        if status == self.MI_OK:
            buf = []
            for i in range(16):
                buf.append(writeData[i])
            # Calculate the CRC checksum for the data to be written
            crc = self.CalulateCRC(buf)
            buf.append(crc[0])
            buf.append(crc[1])
            # Send the data buffer to the tag and receive the response
            (status, backData, backLen) = self.MFRC522_ToCard(
                self.PCD_TRANSCEIVE, buf)
            # Check if the write operation was successful or not
            if not (status == self.MI_OK) or not (backLen == 4) or not ((backData[0] & 0x0F) == 0x0A):
                raise ValueError(f"Error while writing, status={status}, backLen={backLen}, backData={backData}")
            # If the write operation was successful, log it
            if status == self.MI_OK:
                #self.logger.debug("Data written")
                pass

    def Init(self):
        """
        Initializes the MFRC522 RFID reader by resetting it and configuring its registers.
        """

        # Reset the MFRC522
        self.Reset()

        # Set the timer mode and prescaler
        self.WriteReg(self.TModeReg, 0x8D)
        self.WriteReg(self.TPrescalerReg, 0x3E)
        self.WriteReg(self.TReloadRegL, 30)
        self.WriteReg(self.TReloadRegH, 0)

        # Enable the auto-timer for transmission and set the mode
        self.WriteReg(self.TxAutoReg, 0x40)
        self.WriteReg(self.ModeReg, 0x3D)

        # Turn on the antenna
        self.AntennaOn()
