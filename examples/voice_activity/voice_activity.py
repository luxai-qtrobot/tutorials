from tuning import Tuning
import usb.core
import usb.util
import time

if __name__ == '__main__':
    dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
    if dev:
        Mic_tuning = Tuning(dev)
        while True:
            try:
                print("Voice activity %d" % Mic_tuning.is_voice())
                time.sleep(0.2)
            except KeyboardInterrupt:
                break
