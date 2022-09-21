#include <Eigen/Core>
#include <Eigen/LU>
#include <cstddef>
#include <tuple>

template<size_t n_states, size_t n_measurements, size_t n_control_inputs>
class DiscreteKalmanFilter {
public:
    using MatrixXX = Eigen::Matrix<double, n_states, n_states>;
    using MatrixZX = Eigen::Matrix<double, n_measurements, n_states>;
    using MatrixXZ = Eigen::Matrix<double, n_states, n_measurements>;
    using MatrixZZ = Eigen::Matrix<double, n_measurements, n_measurements>;
    using MatrixXU = Eigen::Matrix<double, n_states, n_control_inputs>;
    using VectorX = Eigen::Vector<double, n_states>;
    using VectorZ = Eigen::Vector<double, n_measurements>;
    using VectorU = Eigen::Vector<double, n_control_inputs>;

    using StateEstimateVector = VectorX;
    using StateMeasurementVector = VectorZ;
    using StateTransitionMatrix = MatrixXX;
    using ObservationMatrix = MatrixZX;
    using ControlMatrix = MatrixXU;
    using InputVector = VectorU;
    using EstimateUncertaintyMatrix = MatrixXX;
    using ProcessNoiseUncertaintyMatrix = MatrixXX;
    using MeasurementUncertaintyMatrix = MatrixZZ;
    using KalmanGainMatrix = MatrixXZ;

    DiscreteKalmanFilter(
            const StateTransitionMatrix& F,
            const ControlMatrix& G,
            const ProcessNoiseUncertaintyMatrix& Q,
            const StateEstimateVector& x0,
            const EstimateUncertaintyMatrix& P0
    )
        : F_(F), G_(G), Q_(Q), x_(x0), P_(P0)
    {}

    void predict(const InputVector& u)
    {
        x_ = F_ * x_ + G_ * u;
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    void update(const StateMeasurementVector& z, const ObservationMatrix& H, const MeasurementUncertaintyMatrix& R)
    {
        const MatrixXX I = MatrixXX::Identity();
        KalmanGainMatrix K = (P_ * H.transpose()) * (H * P_ * H.transpose() + R).inverse();
        x_ = x_ + K * (z - H * x_);
        P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
    }

    std::tuple<StateEstimateVector, EstimateUncertaintyMatrix> current_estimate() const
    {
        return {x_, P_};
    }

private:
    StateEstimateVector x_;
    EstimateUncertaintyMatrix P_;

    StateTransitionMatrix F_;
    ControlMatrix G_;
    ProcessNoiseUncertaintyMatrix Q_;
};
