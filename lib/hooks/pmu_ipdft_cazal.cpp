/* ipDFT PMU hook with Cazal's method
 *
 * Author: Leonardo Carreras <leonardo.carreras@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Institute for Automation of Complex Power Systems, RWTH Aachen University
 * SPDX-License-Identifier: Apache-2.0
 */

#include <villas/hooks/pmu.hpp>
#include <cmath>
#include <vector>

#define DELTA_INF_COMPARISON_THRESHOLD 0.000001 // Threshold for negligible delta

namespace villas {
namespace node {

class IpDftPmuCazalHook : public PmuHook {

protected:
  std::vector<std::vector<double>> twiddleReal; // Twiddle factor (real part)
  std::vector<std::vector<double>> twiddleImag; // Twiddle factor (imaginary part)
  std::vector<double> hanning;                 // Hanning window
  std::vector<double> wXkr, wXki, wXka;        // Windowed real, imaginary, magnitude

  unsigned frequencyCount; // Number of frequency bins calculated
  double estimationRange;  // Range around nominalFreq used for estimation

  double Kmin, Kmax, frequencyResolution;
  double b, B; // Normalization factors

public:
  IpDftPmuCazalHook(Path *p, Node *n, int fl, int prio, bool en = true)
      : PmuHook(p, n, fl, prio, en), frequencyCount(0), estimationRange(0) {}

  void prepare() {
    PmuHook::prepare();

    const double startFrequency = nominalFreq - estimationRange;
    const double endFrequency = nominalFreq + estimationRange;
    frequencyResolution = (double)sampleRate / windowSize;

    frequencyCount =
        ceil((endFrequency - startFrequency) / frequencyResolution);

    // Compute Kmin, Kmax, and normalization factors
    Kmin = nominalFreq / frequencyResolution - (frequencyCount / 2);
    Kmax = Kmin + frequencyCount;
    B = 0.5 * windowSize; // Integration of the Hanning window
    b = 1.0 / B;          // Scaling factor

    // Initialize Hanning window
    hanning.resize(windowSize);
    for (unsigned i = 0; i < windowSize; i++) {
      hanning[i] = 0.5 - 0.5 * std::cos((2.0 * M_PI * i) / (double)(windowSize - 1));
    }

    // Precompute Twiddle Factors
    twiddleReal.resize(frequencyCount, std::vector<double>(windowSize, 0.0));
    twiddleImag.resize(frequencyCount, std::vector<double>(windowSize, 0.0));

    for (unsigned i = 0; i < frequencyCount; i++) {
      for (unsigned j = 0; j < windowSize; j++) {
        double angle = -2.0 * M_PI * (i + Kmin) * j / windowSize;
        twiddleReal[i][j] = std::cos(angle);
        twiddleImag[i][j] = std::sin(angle);
      }
    }

    // Initialize DFT arrays
    wXkr.resize(frequencyCount, 0.0);
    wXki.resize(frequencyCount, 0.0);
    wXka.resize(frequencyCount, 0.0);
  }

  void parse(json_t *json) {
    PmuHook::parse(json);
    int ret;
    json_error_t err;

    assert(state != State::STARTED);

    Hook::parse(json);

    ret = json_unpack_ex(json, &err, 0, "{ s?: F }", "estimation_range",
                         &estimationRange);

    if (ret)
      throw ConfigError(json, err, "node-config-hook-ip-dft-pmu");

    if (estimationRange <= 0)
      throw ConfigError(json, "node-config-hook-ip-dft-pmu-estimation_range",
                        "Estimation range cannot be less or equal to 0");
  }

  PmuHook::Phasor estimatePhasor(dsp::CosineWindow<double> *window,
                                 PmuHook::Phasor lastPhasor) {
    PmuHook::Phasor phasor = {0};

    // Apply Hanning window and calculate DFT using precomputed Twiddle Factors
    for (unsigned i = 0; i < frequencyCount; i++) {
      wXkr[i] = 0.0;
      wXki[i] = 0.0;
      for (unsigned j = 0; j < windowSize; j++) {
        double windowedValue = hanning[j] * (*window)[j];
        wXkr[i] += windowedValue * twiddleReal[i][j];
        wXki[i] += windowedValue * twiddleImag[i][j];
      }
      wXka[i] = std::sqrt(wXkr[i] * wXkr[i] + wXki[i] * wXki[i]);
    }

    // Find maximum frequency bin
    unsigned maxBin = 0;
    double absAmplitude = 0.0;
    for (unsigned i = 0; i < frequencyCount; i++) {
      if (absAmplitude < wXka[i]) {
        absAmplitude = wXka[i];
        maxBin = i;
      }
    }

    // Handle boundary conditions
    if (maxBin == 0 || maxBin == (frequencyCount - 1)) {
      logger->warn("Maximum frequency bin lies on window boundary. Using "
                   "non-estimated results!");
      return lastPhasor; // Return previous phasor to avoid invalid results
    }

    // Interpolated DFT (IpDFT) Calculations
    double delta = 0.0;
    if (wXka[maxBin + 1] > wXka[maxBin - 1])
      delta = 1.0 * (2.0 * wXka[maxBin + 1] - wXka[maxBin]) /
              (wXka[maxBin] + wXka[maxBin + 1]);
    else
      delta = -1.0 * (2.0 * wXka[maxBin - 1] - wXka[maxBin]) /
              (wXka[maxBin] + wXka[maxBin - 1]);

    // Amplitude Estimation
    if (std::abs(delta) < DELTA_INF_COMPARISON_THRESHOLD)
      phasor.amplitude = 2.0 * wXka[maxBin];
    else
      phasor.amplitude = 2.0 * wXka[maxBin] *
                         std::abs((M_PI * delta * (1 - delta * delta)) /
                                  std::sin(M_PI * delta));

    // Frequency Estimation
    phasor.frequency = nominalFreq +
                       ((double)maxBin + delta) * frequencyResolution;

    // Phase Estimation
    phasor.phase = std::atan2(wXki[maxBin], wXkr[maxBin]) - M_PI * delta;

    // ROCOF Calculation
    phasor.rocof = (phasor.frequency - lastPhasor.frequency) * phasorRate;

    if (lastPhasor.frequency != 0)
      phasor.valid = Status::VALID;

    return phasor;
  }
};

// Register hook
static char n[] = "ip-dft-pmu";
static char d[] = "This hook calculates a phasor based on ipDFT";
static HookPlugin<IpDftPmuCazalHook, n, d,
                  (int)Hook::Flags::NODE_READ | (int)Hook::Flags::NODE_WRITE |
                      (int)Hook::Flags::PATH>
    p;

} // namespace node
} // namespace villas
