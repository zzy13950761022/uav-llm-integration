USEFULL COMMANDS

```bash
sudo chmod 666 /dev/ttyUSB0
```

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

```bash
sudo chmod 666 /dev/bus/usb
```

```bash
sudo evtest
```

```bash
sudo chmod 666 /dev/input/event*
```

```bash
sudo ip addr add 192.168.0.100/24 dev enp89s0
```
