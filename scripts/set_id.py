import sys
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
old, new = int(sys.argv[1]), int(sys.argv[2])
port = PortHandler("/dev/ttyACM0"); pk = PacketHandler(1.0)
port.openPort(); port.setBaudRate(1000000)
pk.write1ByteTxRx(port, old, 24, 0)            # 토크 OFF (EEPROM 쓰기 조건)
res, err = pk.write1ByteTxRx(port, old, 3, new) # 주소 3 = ID
print("변경 결과:", pk.getTxRxResult(res))
_, res, _ = pk.ping(port, new)
print(f"ID {new} ping:", "OK" if res == COMM_SUCCESS else "실패")
port.closePort()
