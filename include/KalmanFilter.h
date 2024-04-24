

class KalmanFilter {
public:
    KalmanFilter(float q, float r) {
        Q_angle = q;
        Q_bias = q;
        R_measure = r;
        angle = 0.0;
        bias = 0.0;
        P[0][0] = 0.0;
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 0.0;
    }

    float update(float newAngle, float newRate, float dt) {
        // 预测
        float rate = newRate - bias;
        angle += dt * rate;

        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // 更新
        float y = newAngle - angle;
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    }

private:
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
};

