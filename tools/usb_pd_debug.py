import serial
from serial.tools.list_ports import *


class UsbPdBoard(serial.Serial):
    def __init__(self, comport):
        super().__init__(comport)
        self.baudrate = 115200
        self.timeout = 0.25
        if not self.is_open:
            self.open()
        print("Putting board in developer mode...")
        self.send_str('DEV')
        print(self.read_str_until_timeout())

    def send_str(self, s: str):
        if not s.endswith('\n'):
            self.write(f"{s}\n".encode('utf-8'))
        else:
            self.write(s.encode('utf-8'))

    def read_str_until_timeout(self) -> str:
        s = ""
        add_s = self.read(1).decode('utf-8')
        while len(add_s) > 0:
            s += add_s
            add_s = self.read(1).decode('utf-8')
        return s

    def print_src_pdos(self):
        self.send_str('PRINT_SRC_PDOS')
        print(self.read_str_until_timeout())

    def set_pdo_voltage(self, volts):
        self.send_str(f'REQ_VOLTAGE={volts:.2f}\n')
        print(self.read_str_until_timeout())

    def set_pdo_current(self, amps):
        self.send_str(f'REQ_CURRENT={amps:.2f}\n')
        print(self.read_str_until_timeout())

    def negotiate(self, v, amps):
        self.set_pdo_voltage(v)
        self.set_pdo_current(amps)
        self.send_str('NEGO_NEW_PDO\n')
        print(self.read_str_until_timeout())
    
    def negotiate_number(self, num):
        self.send_str(f'REQ_NUMBER={num}\n')
        print(self.read_str_until_timeout())
        self.send_str('NEGO_NEW_PDO\n')
        print(self.read_str_until_timeout())


if __name__ == '__main__':
    ports =  comports()
    index = 0
    for port in ports:
        print(f"{index}: ", end='')
        print(port)
        index += 1
    try:
        comport = ports[int(input("Choose comport by line number: "))].device
    except IndexError:
        exit("Wrong answer you fucking donkey")
    except ValueError:
        exit("Enter a number next time you fucken donkey")

    print(f"Using {comport} as the debug device.")

    usbpd = UsbPdBoard(comport)
    usbpd.print_src_pdos()
    done = False
    while not done:
        try:
            voltstr = input("Negotiate Voltage: ")
            if '#' in voltstr:
                pdo_num = int(voltstr.lstrip('#'))
                print(f"Requesting PDO number {pdo_num}...")
                usbpd.negotiate_number(pdo_num)
            else:
                volts = float(voltstr)
                curr  = float(input("Negotiate Current: "))
                usbpd.negotiate(volts, curr)

        except ValueError:
            pass
        except KeyboardInterrupt:
            done = True
    
