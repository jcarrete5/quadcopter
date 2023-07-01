# Tests

## I2C Device Test

### Setup
Instructions assume starting from project root.
```bash
cd $(git rev-parse --show-toplevel)
```

#### Build & load external kernel module
```bash
pushd i2c-stub-plain
make
sudo insmod ./i2c-stub-plain.ko bus_number=1 chip_addr=0x10
popd
```

#### Add user to i2c group (optional)
```bash
sudo usermod -aG i2c
```
This requires the user to log out and log in / possibly reboot.

Check for i2c group in
```bash
groups
```

### Build project
```bash
cmake -S . -B build
cmake --build build
```

### Test
```bash
ctest --test-dir=build
```
Note: If the user is not part of the `i2c` group, this requires sudo.
