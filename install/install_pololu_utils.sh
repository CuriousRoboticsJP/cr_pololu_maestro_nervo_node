# requirements
apt-get install -y \
    libusb-1.0-0-dev \
    mono-runtime \
    libmono-system-windows-forms4.0-cil

# unpack components components
tar -xzvf ./maestro*.tar.gz
cd ./maestro-linux

# device permisions
cp ./99-pololu.rules /etc/udev/rules.d/
udevadm control --reload-rules
