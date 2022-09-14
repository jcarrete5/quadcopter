#include <Eigen/Core>
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
    using StateMeasurement = VectorZ;
    using StateTransitionMatrix = MatrixXX;
    using ObservationMatrix = MatrixZX;
    using ControlMatrix = MatrixXU;
    using InputVector = VectorU;
    using EstimateUncertaintyVector = MatrixXX;
    using ProcessNoiseUncertaintyMatrix = MatrixXX;
    using MeasurementUncertaintyMatrix = MatrixZZ;
    using KalmanGain = MatrixXZ;

    DiscreteKalmanFilter(
            const StateTransitionMatrix& F,
            const ControlMatrix& G,
            const ObservationMatrix& H,
            const ProcessNoiseUncertaintyMatrix& Q,
            const MeasurementUncertaintyMatrix& R
    )
        : F_(F), G_(G), H_(H), Q_(Q), R_(R)
    {}

    void initialize(const StateEstimateVector& X0, const EstimateUncertaintyVector& P0)
    {
        X_ = X0;
        P_ = P0;
    }

    std::tuple<StateEstimateVector, EstimateUncertaintyVector> update(const StateMeasurement& Z)
    {
        const MatrixXX I = MatrixXX::Identity();
        KalmanGain K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
        X_ = X_ + K * (Z - H_ * X_);
        P_ = (I - K * H_) * P_ * (I - K * H_).transpose() + K * R_ * K.transpose();
        return {X_, P_};
    }

    std::tuple<StateEstimateVector, EstimateUncertaintyVector> predict(const InputVector& u)
    {
        StateEstimateVector X_prediction = F_ * X_ + G_ * u;
        EstimateUncertaintyVector P_prediction = F_ * P_ * F_.transpose() + Q_;
        return {X_prediction, P_prediction};
    }

private:
    StateEstimateVector X_;
    EstimateUncertaintyVector P_;

    StateTransitionMatrix F_;
    ControlMatrix G_;
    ObservationMatrix H_;
    ProcessNoiseUncertaintyMatrix Q_;
    MeasurementUncertaintyMatrix R_;
};
