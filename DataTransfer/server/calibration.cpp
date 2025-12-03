#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <complex>
#include <cstdint>
#include <cstring>
#include <algorithm>

const char* SERIAL_PORT = "/dev/ttyACM0";
const int BAUDRATE = B115200;
const int POINTS = 201;
const int BLOCK_SIZE = 32;
const uint64_t START_FREQ = 500000000;
const uint64_t STOP_FREQ = 3000000000;
const uint64_t STEP_FREQ = (STOP_FREQ - START_FREQ) / (POINTS - 1);

int openSerial(const char* device) {
    int fd = open(device, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios options{};
    tcgetattr(fd, &options);
    cfsetspeed(&options, BAUDRATE);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CRTSCTS;
    options.c_iflag = IGNPAR;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

void write_reg_8(int fd, uint8_t addr, uint64_t value) {
    uint8_t buf[10];
    buf[0] = 0x23;
    buf[1] = addr;
    for (int i = 0; i < 8; ++i)
        buf[2 + i] = (value >> (i * 8)) & 0xFF;
    write(fd, buf, 10);
}

void write_reg_2(int fd, uint8_t addr, uint16_t value) {
    uint8_t buf[4] = {0x21, addr, (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF)};
    write(fd, buf, 4);
}

void flush_fifo(int fd) {
    uint8_t cmd[2] = {0x30, 0};
    write(fd, cmd, 2);
    usleep(50000);
    write(fd, cmd, 2);
    usleep(50000);
}

void trigger_sweep(int fd) {
    uint8_t cmd[2] = {0x27, 1};
    write(fd, cmd, 2);
}

std::vector<uint8_t> readSweep(int fd, int total_bytes) {
    std::vector<uint8_t> buffer(total_bytes);
    int bytes_read = 0;
    while (bytes_read < total_bytes) {
        int r = read(fd, buffer.data() + bytes_read, total_bytes - bytes_read);
        if (r <= 0) break;
        bytes_read += r;
    }
    buffer.resize(bytes_read);
    return buffer;
}

void parseAndPrint(const std::vector<uint8_t>& data) {
    int total = std::min((int)data.size() / BLOCK_SIZE, POINTS);
    std::cout << "Total valid blocks: " << total << std::endl;
    for (int i = 0; i < total; ++i) {
        const uint8_t* block = &data[i * BLOCK_SIZE];
        int32_t fwd_re = *reinterpret_cast<const int32_t*>(&block[0]);
        int32_t fwd_im = *reinterpret_cast<const int32_t*>(&block[4]);
        int32_t rev_re = *reinterpret_cast<const int32_t*>(&block[8]);
        int32_t rev_im = *reinterpret_cast<const int32_t*>(&block[12]);
        uint16_t idx = *reinterpret_cast<const uint16_t*>(&block[24]);
        std::complex<float> fwd(fwd_re, fwd_im);
        std::complex<float> rev(rev_re, rev_im);
        std::complex<float> s11 = (std::abs(fwd) > 0) ? rev / fwd : std::complex<float>(0, 0);
        double freq = START_FREQ + idx * STEP_FREQ;
        std::cout << freq / 1e6 << " MHz: " << s11.real() << " + j" << s11.imag() << std::endl;
    }
}

void sweepAndShow(int fd, const char* label) {
    std::string prompt;
    std::cout << "Connect " << label << " to CH0, then press ENTER..." << std::endl;
    std::getline(std::cin, prompt);
    flush_fifo(fd);
    trigger_sweep(fd);
    usleep(400000);
    auto data = readSweep(fd, POINTS * BLOCK_SIZE);
    std::cout << "\n[" << label << "] Sweep Result:" << std::endl;
    parseAndPrint(data);
}

int main() {
    int fd = openSerial(SERIAL_PORT);
    if (fd < 0) {
        std::cerr << "Error opening serial port." << std::endl;
        return 1;
    }

    write_reg_8(fd, 0x00, START_FREQ);
    write_reg_8(fd, 0x10, STEP_FREQ);
    write_reg_2(fd, 0x20, POINTS);

    sweepAndShow(fd, "LOAD");
    sweepAndShow(fd, "OPEN");
    sweepAndShow(fd, "SHORT");

    close(fd);
    return 0;
}
