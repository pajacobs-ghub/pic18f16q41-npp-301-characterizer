# rs485_npp301.py
# Functions to interact with the PIC18F16Q41 NPP-301 characterization node
# through a serial RS485 cable.
#
# PJ 2025-02-22: Begin by adapting rs485-edaq.py
#    2025-02-24: Report calculated resistances
#
import argparse
import serial
import time
import serial.tools.list_ports as list_ports
import re
import struct

# -----------------------------------------------------------------------------
# The RS485 communication happens through a standard serial port.
# Use pySerial to handle this connection.

def serial_ports():
    return [p.device for p in list_ports.comports()]

def openPort(port='/dev/ttyUSB0'):
    '''
    Returns a handle to the opened serial port, or None.
    '''
    ser = None
    try:
        ser = serial.Serial(port, 115200, rtscts=0, timeout=0.5)
    except serial.serialutil.SerialException:
        print(f'Did not find serial port: {port}')
        print(f'Serial ports that can be seen: {serial_ports()}')
        return None
    return ser

# -----------------------------------------------------------------------------
# Each data-recording node on the RS485 bus will be represented in this program
# by and instance of the following class.

class NPP301Node(object):
    __slots__ = ['id_char', 'serial_port']

    def __init__(self, id_char, serial_port):
        '''
        Each node on the RS485 bus should listen to all messages but
        accept and act only on the messages addressed to their id.

        The controlling node (master) has id character b'0'.
        Other nodes may be '1', '2', ... 'A' .. 'Z', 'a' .. 'z'.
        '''
        self.id_char = id_char
        self.serial_port = serial_port
        return

    #---------------------------------------------------------
    # Fundamentally, it's all messages on the RS485 bus.

    def send_RS485_command(self, cmd_txt):
        '''
        Send the wrapped command text on the RS485 bus.

        For notes, see PJ's workbook page 76, 2024-01-09.
        '''
        self.serial_port.reset_input_buffer()
        cmd_bytes = f'/{self.id_char}{cmd_txt}!\n'.encode('utf-8')
        # print("cmd_bytes=", cmd_bytes)
        self.serial_port.write(cmd_bytes)
        self.serial_port.flush()
        return

    def get_RS485_response(self):
        '''
        Returns the unwrapped response text that comes back
        from a previously sent command.

        For notes, see PJ's workbook page 76, 2024-01-09.
        '''
        txt = self.serial_port.readline().strip().decode('utf-8')
        if txt.startswith('/0'):
            if txt.find('#') < 0:
                print('Incomplete RS485 response:', txt)
            else:
                txt = re.sub('/0', '', txt).strip()
                txt = re.sub('#', '', txt).strip()
        else:
            raise RuntimeError(f'Invalid RS485 response: {txt}')
        return txt

    #-----------------------------------------------------------
    # PIC18F16Q41 service functions are built on RS485 messages.

    def command_PIC(self, cmd_txt):
        '''
        Sends the text of a command to the RS485 send function.
        Returns the text of the RS485 return message.

        Each command to the PIC MCU is encoded as the first character
        of the command text. Any required data follows that character.

        A return message should start with the same command character
        and may have more text following that character.
        A command that is not successful should send back a message
        with the word "error" in it, together with some more information.
        '''
        cmd_char = cmd_txt[0]
        self.send_RS485_command(cmd_txt)
        txt = self.get_RS485_response()
        if not txt.startswith(cmd_char):
            raise RuntimeError(f'Unexpected response: {txt}')
        txt = re.sub(cmd_char, '', txt, count=1).strip()
        if txt.find('error') >= 0:
            print("Warning: error return for command to PIC MCU.")
            print(f"  cmd_txt: {cmd_txt}")
            print(f"  response: {txt}")
        return txt

    def get_PIC_version(self):
        return self.command_PIC('v')

    def set_PIC_LED(self, val):
        txt = self.command_PIC(f'L{val}')
        return

    def set_PIC_VREF_on(self, level):
        '''
        Enable the analog-voltage output of the PIC MCU.
        level is an 8-bit integer 0-255.
        The output is set at (level/256 * 4.096) Volts.
        '''
        level = int(level)
        if level < 0: level = 0
        if level > 255: level = 255
        txt = self.command_PIC(f'w {level} 1')
        return

    def set_PIC_VREF_off(self):
        '''
        Disable the analog-voltage output of the PIC MCU.
        '''
        txt = self.command_PIC(f'w 0 0')
        return

    def read_PIC_ADC(self):
        '''
        Get the current ADC values.
        '''
        txt = self.command_PIC('a')
        return txt

if __name__ == '__main__':
    # A basic test to see if the NPP301-characterization node is attached and awake.
    # Assuming that you have node 'N', typical use on a Linux box:
    # $ python3 rs485_npp301.py
    import argparse
    parser = argparse.ArgumentParser(description="NPP301-characterization program")
    parser.add_argument('-p', '--port', dest='port', help='name for serial port')
    parser.add_argument('-i', '--identity', dest='identity', help='single-character identity')
    args = parser.parse_args()
    port_name = '/dev/ttyUSB0'
    if args.port: port_name = args.port
    node_id = 'N'
    if args.identity: node_id = args.identity
    sp = openPort(port_name)
    if sp:
        node1 = NPP301Node(node_id, sp)
        #
        print("First, see that board is alive.")
        node1.set_PIC_LED(1)
        print(node1.get_PIC_version())
        time.sleep(1.0)
        node1.set_PIC_LED(0)
        node1.set_PIC_VREF_on(255)
        print("Now, report the NPP-301 resistances.")
        print("Press Control-C to finish.")
        try:
            while True:
                response = node1.read_PIC_ADC()
                adcs = [int(item) for item in response.strip().split()]
                a8, a2, a4, a5, a6 = adcs
                Rref = 1000.0
                r1 = (a8-a2)/a4 * Rref
                r2 = (a2-a4)/a4 * Rref
                r3 = (a8-a6)/a5 * Rref
                r4 = (a6-a5)/a5 * Rref
                print(f"adc readings a8={a8} a2={a2} a4={a4} a5={a5} a6={a6}"
                      f"  resistances r1={r1:.1f} r2={r2:.1f} r3={r3:.1f} r4={r4:.1f}")
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
    else:
        print("Did not find the serial port.")
    print("Done.")
