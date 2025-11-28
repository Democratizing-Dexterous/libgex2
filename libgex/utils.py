import serial
import serial.tools
import serial.tools.list_ports

def search_ports():
    """
    search for serial ports, COM* for Windows, ttyACM* or ttyUSB* for Linux
    """
    ports = serial.tools.list_ports.comports()

    if len(ports) == 0:
        print('No COM ports found!')
        return None
    ports_info = {}
    
    for p in ports:
        print(p.device, p.serial_number)
        ports_info[p.serial_number] = p.device

    return ports_info

def get_port_by_serial_number(ports_info, serial_number):
    if serial_number in ports_info:
        return ports_info[serial_number]
    else:
        return None

if __name__ == "__main__":
    ports_info = search_ports()