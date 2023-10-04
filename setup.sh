#!/bin/bash

# Check if vectorsecrets.txt exists. This is used to copy over the relevant vectornav files
if [ ! -f vectorsecrets.txt ]; then
    echo "vectorsecrets.txt does not exist. Creating it now:"
    echo -n "Please enter your login: "
    read username
    echo -n "Please enter your password: "
    read -s password
    echo -n "machine cdn.soonerrobotics.org login $username password $password" > vectorsecrets.txt
fi

# Update packages
sudo apt update

# Install wget and unzip
sudo apt install wget unzip -y

## VECTORNAV

### Download and unzip vectornav files
curl --output vectornav.zip --netrc-file vectorsecrets.txt https://cdn.soonerrobotics.org/protected/vectornav.zip
sudo mkdir /usr/local/vectornav
sudo chown -R "$USER":"$USER" /usr/local/vectornav
unzip -o vectornav.zip -d /usr/local/vectornav
rm vectornav.zip

### Install vectornav
cd /usr/local/vectornav/python
sudo python3 setup.py install -q

## RULES
sudo cp local/autonav.rules /etc/udev/rules.d/autonav.rules
sudo service udev reload
sleep 2
sudo service udev restart

## SERVICES
sudo cp autonav.service /etc/systemd/system/autonav.service
sudo cp autonav_service.sh /usr/bin/autonav_service.sh
sudo chmod +x /usr/bin/autonav_service.sh
sudo chmod 644 /etc/systemd/system/autonav.service

echo "Autonav 2024 Setup Complete - To enable self boot, run the following command:"
echo "sudo systemctl enable autonav"