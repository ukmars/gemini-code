from BLEModule import *
import gc
import time


ble = bluetooth.BLE()
BLEP = BLESimplePeripheral(ble)

_DEBUG_BUF_SIZE = 256
_debug_buf = bytearray(_DEBUG_BUF_SIZE)


def writeDebug(*args):
    '''
    This should be a heap-safe debug function that can be called from anywhere.
    It will send the debug message to the console and over BLE if connected.
    You should not use .format() as it will allocate memory.
    '''
    global _debug_buf
    idx = 0
    # Iterate over each argument
    for i, arg in enumerate(args):
        s = str(arg)  # Convert argument to string
        for c in s:
            if idx < _DEBUG_BUF_SIZE - 2:  # reserve space for \r\n
                _debug_buf[idx] = ord(c)
                idx += 1
            else:
                break  # stop if buffer full
        # Add a space if not the last argument
        if i < len(args) - 1 and idx < _DEBUG_BUF_SIZE - 2:
            _debug_buf[idx] = ord(' ')
            idx += 1
    # Append CRLF at the end if space permits
    if idx < _DEBUG_BUF_SIZE - 2:
        _debug_buf[idx] = 13  # '\r'
        _debug_buf[idx + 1] = 10  # '\n'
        idx += 2

    # Output to the USB/console
    print(*args)

    # Send over BLE if connected. 
    if BLEP.is_connected():
        # Slice the buffer up to the current index (no extra allocation)
        BLEP.send(_debug_buf[:idx])
        


def on_rx(v):
    print("RX", v)

BLEP.on_write(on_rx)
data = 5800
i = 0
while True:
    writeDebug("Hello World", 55555, "Some more text", data)
    print("Free RAM:", gc.mem_free()) # type: ignore
    time.sleep_ms(1000) # type: ignore
