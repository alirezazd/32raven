#include "i2c.hpp"

#include "panic.hpp"

namespace {

template <I2cInstance Inst>
constexpr i2c_port_num_t ToPort() {
  if constexpr (Inst == I2cInstance::kDisplay) {
    return I2C_NUM_0;
  } else {
    static_assert(Inst == I2cInstance::kDisplay, "Invalid I2C instance");
  }
}

}  // namespace

template <I2cInstance Inst>
void I2c<Inst>::Init(const I2cConfig &cfg) {
  if (cfg.pins.sda_gpio == GPIO_NUM_NC || cfg.pins.scl_gpio == GPIO_NUM_NC ||
      cfg.bus.transfer_timeout_ms == 0) {
    Panic(ErrorCode::kI2cParamConfigFailed);
  }

  cfg_ = cfg;

  i2c_master_bus_config_t bus_cfg{};
  bus_cfg.i2c_port = ToPort<Inst>();
  bus_cfg.sda_io_num = cfg_.pins.sda_gpio;
  bus_cfg.scl_io_num = cfg_.pins.scl_gpio;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = cfg_.bus.glitch_ignore_cnt;
  bus_cfg.intr_priority = 0;
  // Use synchronous transfers. The display stack issues a burst of setup writes
  // during boot and does not use the callback-based async path.
  bus_cfg.trans_queue_depth = 0;
  bus_cfg.flags.enable_internal_pullup = cfg_.bus.enable_internal_pullup;
  bus_cfg.flags.allow_pd = 0;

  if (i2c_new_master_bus(&bus_cfg, &bus_) != ESP_OK) {
    Panic(ErrorCode::kI2cInitFailed);
  }
}

template <I2cInstance Inst>
i2c_master_dev_handle_t I2c<Inst>::AddDevice(const I2cDeviceConfig &cfg) {
  if (cfg.address > 0x7F || cfg.timing.clock_hz == 0) {
    Panic(ErrorCode::kI2cParamConfigFailed);
  }

  i2c_device_config_t dev_cfg{};
  dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  dev_cfg.device_address = cfg.address;
  dev_cfg.scl_speed_hz = cfg.timing.clock_hz;
  dev_cfg.scl_wait_us = cfg.timing.scl_wait_us;
  dev_cfg.flags.disable_ack_check = cfg.disable_ack_check;

  i2c_master_dev_handle_t handle = nullptr;
  if (i2c_master_bus_add_device(bus_, &dev_cfg, &handle) != ESP_OK) {
    Panic(ErrorCode::kI2cInitFailed);
  }
  return handle;
}

template <I2cInstance Inst>
bool I2c<Inst>::Probe(uint16_t address) const {
  if (address > 0x7F) {
    Panic(ErrorCode::kI2cInvalidArg);
  }

  return i2c_master_probe(bus_, address,
                          static_cast<int>(cfg_.bus.transfer_timeout_ms)) == ESP_OK;
}

template <I2cInstance Inst>
void I2c<Inst>::Transmit(i2c_master_dev_handle_t device, const uint8_t *data,
                         size_t size) const {
  if (data == nullptr || size == 0) {
    Panic(ErrorCode::kI2cInvalidArg);
  }

  if (i2c_master_transmit(device, data, size,
                          static_cast<int>(cfg_.bus.transfer_timeout_ms)) != ESP_OK) {
    Panic(ErrorCode::kI2cOperationFailed);
  }
}

template <I2cInstance Inst>
void I2c<Inst>::MultiBufferTransmit(
    i2c_master_dev_handle_t device,
    i2c_master_transmit_multi_buffer_info_t *buffers,
    size_t buffer_count) const {
  if (buffers == nullptr || buffer_count == 0) {
    Panic(ErrorCode::kI2cInvalidArg);
  }

  if (i2c_master_multi_buffer_transmit(
          device, buffers, buffer_count,
          static_cast<int>(cfg_.bus.transfer_timeout_ms)) != ESP_OK) {
    Panic(ErrorCode::kI2cOperationFailed);
  }
}

template <I2cInstance Inst>
void I2c<Inst>::TransmitReceive(i2c_master_dev_handle_t device,
                                const uint8_t *write_buffer, size_t write_size,
                                uint8_t *read_buffer, size_t read_size) const {
  if (write_buffer == nullptr || write_size == 0 || read_buffer == nullptr ||
      read_size == 0) {
    Panic(ErrorCode::kI2cInvalidArg);
  }

  if (i2c_master_transmit_receive(
          device, write_buffer, write_size, read_buffer, read_size,
          static_cast<int>(cfg_.bus.transfer_timeout_ms)) != ESP_OK) {
    Panic(ErrorCode::kI2cOperationFailed);
  }
}

template class I2c<I2cInstance::kDisplay>;
