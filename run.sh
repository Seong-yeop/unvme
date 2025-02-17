#!/bin/bash



echo "0000:af:00.0" | sudo tee /sys/bus/pci/drivers/nvme/unbind
sudo setpci -s 0000:af:00.0 COMMAND=0x0007

gcc nvme_driver.c -o unvme



