#include "shadower/source/source.h"
#include "shadower/utils/constants.h"
#include "srsran/radio/radio.h"
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>

class UHDSource final : public Source
{
public:
  /* Initialize the radio object and apply the configurations */
  UHDSource(ShadowerConfig& config) : config(config), rf(std::make_unique<srsran_rf_t>())
  {
    set_num_channels(config.nof_channels);
    // Set clock source
    if (config.clock_source != "internal" && config.source_params.find("clock=") == std::string::npos) {
      config.source_params += ",clock=" + config.clock_source;
    }
    // Set sync source
    if (config.sync_source != "internal" && config.source_params.find("sync=") == std::string::npos) {
      config.source_params += ",sync=" + config.sync_source;
    }

    // Configure the master_clock_rate for b210 device
    if (config.source_params.find("type=b200") != std::string::npos &&
        config.source_params.find("master_clock_rate=") == std::string::npos) {
      config.source_params += ",master_clock_rate=" + std::to_string(config.sample_rate);
      config.source_params += ",sampling_rate=" + std::to_string(config.sample_rate);
    }

    /* Initialize srsran rf multi */
    if (srsran_rf_open_multi(rf.get(), (char*)config.source_params.c_str(), nof_channels)) {
      throw std::runtime_error("Failed to open radio");
    }

    /* setup the rf interface */
    srsran_rf_set_tx_srate(rf.get(), config.sample_rate);
    srsran_rf_set_rx_srate(rf.get(), config.sample_rate);
    for (uint32_t ch = 0; ch < nof_channels; ch++) {
      ChannelConfig& channelCfg = config.channels[ch];
      srsran_rf_set_rx_freq(rf.get(), ch, channelCfg.rx_frequency + channelCfg.rx_offset);
      srsran_rf_set_rx_gain_ch(rf.get(), ch, channelCfg.rx_gain);

      srsran_rf_set_tx_freq(rf.get(), ch, channelCfg.tx_frequency + channelCfg.tx_offset);
      srsran_rf_set_tx_gain_ch(rf.get(), ch, channelCfg.tx_gain);
    }
    srsran_rf_start_rx_stream(rf.get(), false);
  }

  ~UHDSource() override { close(); }

  bool is_sdr() const override { return true; }

  int send(cf_t** buffer, uint32_t nof_samples, srsran_timestamp_t& ts, uint32_t slot = 0) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    try {
      int samples_sent = srsran_rf_send_timed_multi(
          rf.get(), (void**)buffer, nof_samples, ts.full_secs, ts.frac_secs, true, true, true);
      return samples_sent;
    } catch (const std::exception& e) {
      return -1;
    }
  }

  // int recv(cf_t** buffer, uint32_t nof_samples, srsran_timestamp_t* ts) override
  // {
  //   /* Start the rx stream */
  //   try {
  //     int samples_recv =
  //         srsran_rf_recv_with_time_multi(rf.get(), (void**)buffer, nof_samples, false, &ts->full_secs, &ts->frac_secs);
  //     if (samples_recv == SRSRAN_ERROR) {
  //       return -1;
  //     }
  //     return samples_recv;
  //   } catch (const std::exception& e) {
  //     return -1;
  //   }
  // }

  int recv(cf_t** buffer, uint32_t nof_samples, srsran_timestamp_t* ts) override
  {
    static uint64_t rx_count = 0;
    static bool have_prev = false;

    static uint64_t prev_samples = 0;
    static double prev_timestamp = 0.0;

    constexpr double sample_rate = config.sample_rate;

    try {
      int samples_recv =
          srsran_rf_recv_with_time_multi(
              rf.get(),
              (void**)buffer,
              nof_samples,
              false,
              &ts->full_secs,
              &ts->frac_secs);

      if (samples_recv == SRSRAN_ERROR) {
        std::cerr << "[UHD-RX] ERROR: srsran_rf_recv_with_time_multi() returned SRSRAN_ERROR\n";
        return -1;
      }

      ++rx_count;

      // Timestamp of the first sample in this received buffer.
      const double current_timestamp =
          static_cast<double>(ts->full_secs) +
          static_cast<double>(ts->frac_secs);

      // Expected timestamp based on the previous buffer.
      double expected_timestamp = 0.0;
      double timestamp_gap = 0.0;
      double timestamp_gap_samples = 0.0;

      if (have_prev) {
        expected_timestamp =
            prev_timestamp +
            static_cast<double>(prev_samples) / sample_rate;

        timestamp_gap =
            current_timestamp - expected_timestamp;

        timestamp_gap_samples =
            timestamp_gap * sample_rate;
      }

      /*
      * Print:
      *   - the first 20 buffers
      *   - any buffer whose timestamp is discontinuous by > 1 sample
      */
      if (rx_count <= 20 ||
          (have_prev && std::abs(timestamp_gap_samples) > 1.0)) {

        std::cout << std::fixed << std::setprecision(12);

        std::cout
            << "[UHD-RX]"
            << " count=" << rx_count
            << " requested=" << nof_samples
            << " received=" << samples_recv
            << " timestamp=" << current_timestamp;

        if (have_prev) {
          std::cout
              << " expected=" << expected_timestamp
              << " gap=" << timestamp_gap * 1e6
              << " us"
              << " (" << timestamp_gap_samples
              << " samples)";
        }

        std::cout << std::endl;
      }

      prev_timestamp = current_timestamp;
      prev_samples = static_cast<uint64_t>(samples_recv);
      have_prev = true;

      return samples_recv;
    }
    catch (const std::exception& e) {
      std::cerr
          << "[UHD-RX] Exception in recv(): "
          << e.what()
          << std::endl;

      return -1;
    }
  }

  void close() override
  {
    if (!rf) {
      return;
    }
    /* Stop the rx stream */
    srsran_rf_close(rf.get());
    rf.reset();
  }
  void set_tx_gain(double gain) override
  {
    for (uint32_t i = 0; i < nof_channels; i++) {
      srsran_rf_set_tx_gain_ch(rf.get(), i, gain);
    }
  }
  void set_rx_gain(double gain) override
  {
    for (uint32_t i = 0; i < nof_channels; i++) {
      srsran_rf_set_rx_gain_ch(rf.get(), i, gain);
    }
  }
  void set_tx_srate(double sample_rate) override { srsran_rf_set_tx_srate(rf.get(), sample_rate); }
  void set_rx_srate(double sample_rate) override { srsran_rf_set_rx_srate(rf.get(), sample_rate); }
  void set_tx_freq(double freq) override
  {
    for (uint32_t i = 0; i < nof_channels; i++) {
      srsran_rf_set_tx_freq(rf.get(), i, freq);
    }
  }
  void set_rx_freq(double freq) override
  {
    for (uint32_t i = 0; i < nof_channels; i++) {
      srsran_rf_set_rx_freq(rf.get(), i, freq);
    }
  }

private:
  std::unique_ptr<srsran_rf_t> rf;
  std::mutex                   mutex;
  ShadowerConfig               config = {};
};

extern "C" {
__attribute__((visibility("default"))) Source* create_source(ShadowerConfig& config)
{
  return new UHDSource(config);
}
}
