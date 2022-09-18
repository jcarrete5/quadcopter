#include "kalman_filter.h"

#include <Eigen/Core>
#include <vector>

#include "gtest/gtest.h"

#define LENGTH(x) (sizeof(x)/sizeof(x[0]))

// example from https://www.kalmanfilter.net/multiExamples.html
class RocketAltitudeFixture : public ::testing::Test {
protected:
    static constexpr size_t n_states = 2;
    static constexpr size_t n_measurements = 1;
    static constexpr size_t n_inputs = 1;
    using KalmanFilter = DiscreteKalmanFilter<n_states, n_measurements, n_inputs>;

    const KalmanFilter::StateTransitionMatrix F
        {{1, 0.25},
         {0, 1}};
    const KalmanFilter::ControlMatrix G
        {{0.0313},
         {0.25}};
    const KalmanFilter::ObservationMatrix H
        {{1.0, 0.0}};
    const KalmanFilter::ProcessNoiseUncertaintyMatrix Q
        {{1.0/102400, 1.0/12800},
         {1.0/12800, 1.0/16}};
    const KalmanFilter::MeasurementUncertaintyMatrix R
        {{400.0}};

    KalmanFilter::StateEstimateVector initial_state
        {{0.0, 0.0}};
    KalmanFilter::EstimateUncertaintyMatrix initial_estimate_uncertainty
        {{500.0, 0.0},
         {0.0, 500.0}};
    KalmanFilter::InputVector initial_input
        {9.8};

    KalmanFilter kf;
    std::vector<KalmanFilter::StateMeasurementVector> measurements;
    std::vector<KalmanFilter::InputVector> inputs;

    RocketAltitudeFixture() :
        kf{F, G, H, Q, R}
    {
        const size_t n_samples = LENGTH(measurement_data_);
        for (size_t i = 0; i < n_samples; i++)
        {
            measurements.push_back(KalmanFilter::VectorZ(measurement_data_[i]));
            inputs.push_back(KalmanFilter::VectorU(input_data_[i]-9.8));
        }
    }

private:
    static constexpr double measurement_data_[] = 
        {-32.4, -11.1, 18, 22.9, 19.5, 28.5, 46.5, 68.9, 48.2, 56.1, 90.5, 104.9, 140.9, 148, 187.6, 209.2, 244.6, 276.4, 323.5, 357.3, 357.4, 398.3, 446.7, 465.1, 529.4, 570.4, 636.8, 693.3, 707.3, 748.5};
    static constexpr double input_data_[] =
        {39.72, 40.02, 39.97, 39.81, 39.75, 39.6, 39.77, 39.83, 39.73, 39.87, 39.81, 39.92, 39.78, 39.98, 39.76, 39.86, 39.61, 39.86, 39.74, 39.87, 39.63, 39.67, 39.96, 39.8, 39.89, 39.85, 39.9, 39.81, 39.81, 39.68};
};

TEST_F(RocketAltitudeFixture, Filter)
{
    kf.initialize(initial_state, initial_estimate_uncertainty, initial_input);

    std::tuple<KalmanFilter::StateEstimateVector, KalmanFilter::EstimateUncertaintyMatrix> estimate;
    for (size_t sample_i = 0; sample_i < measurements.size(); sample_i++)
    {
        std::cout << "measurement: " << measurements.at(sample_i) << '\n';
        std::cout << "input: " << inputs.at(sample_i) << '\n';

        kf.update(measurements.at(sample_i));
        estimate = kf.current_estimate();

        std::cout << "estimate: " << std::get<0>(estimate)  << '\n';
        std::cout << "uncertainty: " << std::get<1>(estimate)  << '\n';

        kf.predict(inputs.at(sample_i));
    }
    
    EXPECT_NEAR(std::get<0>(estimate)(0), 831.5, 0.15*831.5);
    EXPECT_NEAR(std::get<0>(estimate)(1), 222.94, 0.15*222.94);
}
