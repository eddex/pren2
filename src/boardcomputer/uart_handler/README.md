# Setting up the UARTHandler

## Preparing UART Interface

### Raspberry Pi 3B+
By default, the UART Interface on the Raspberry Pi is configured as serial console.
To use the relevant GPIO Pins, we need to free them up first. 


##### Backup the existing configuration
First we need to make a backup of the files **cmdline.txt** as **cmdline_backup.txt** containing the kernel parameters.
```
sudo cp /boot/cmdline.txt /boot/cmdline_backup.txt 
```

##### Edit /boot/cmdline.txt
Edit the file cmdline.txt by removing the parameter containing 'ttyAMA0'

```
sudo nano /boot/cmdline.txt
```

Remove the following line:

``console=ttyAMA0,115200’ and ‘kgdboc=ttyAMA0,115200``

the remaining file should look like this:

```
dwc_otg.lpm_enable=0 console=tty1 root=/dev/mmcblk0p6 rootfstype=ext4 elevator=deadline rootwait
```

- Save the file (Ctrl + O)
- Close the editor (Ctrl + X)


##### Edit /etc/inittab

Open the File with nano
```
sudo nano /etc/inittab
```

Comment out the line:

``
2:23:respawn:/sbin/getty -L ttyAMA0 115200 vt100
``

- Save the file (Ctrl + O)
- Close the editor (Ctrl + X)
- **Reboot the Raspberry Pi by using the commmand:** `sudo reboot`

 #### Testing the UART Interface
 You can verify whether the Pi is sending and receiving UART data by installing the tool Minicom.
 
 - Short the Rx and Tx pins on Pi (GPIO 14 and 15), such that it will receive the same data as it transmits.
 - Install minicom `sudp apt-get install minicom`
 - Launche Minicom
 ``minicom -b 115200 -o -D /dev/ttyAMA0``
