# Joystick control
![](../../img/rastreator_joystick_options.png)

### Configuration

Check the authority of rplidar's serial-port :

```
ls -l /dev |grep ttyUSB
```

Add the authority of write: (such as /dev/ttyUSB0)

```
sudo chmod 666 /dev/ttyUSB0
```

