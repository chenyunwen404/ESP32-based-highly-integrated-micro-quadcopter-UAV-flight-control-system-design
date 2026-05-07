#pragma once

template <typename T>
class FirstOrderFilter {
public:
    /**
     * @brief 默认构造函数
     */
    FirstOrderFilter() : alpha_(1.0f), prev_output_(0.0f), initialized_(false) {}

    /**
     * @brief 构造函数，初始化滤波器参数
     * @param cutoff_freq_hz 截止频率 (Hz)
     * @param sampling_time_s 采样周期 (秒)
     */
    FirstOrderFilter(T cutoff_freq_hz, T sampling_time_s) {
        setParameters(cutoff_freq_hz, sampling_time_s);
        prev_output_ = 0.0f;
        initialized_ = false;
    }

    /**
     * @brief 设置滤波器参数 (可在运行时动态调整)
     * @param cutoff_freq_hz 截止频率 (Hz)
     * @param sampling_time_s 采样周期 (秒)
     */
    void setParameters(T cutoff_freq_hz, T sampling_time_s) {
        if (cutoff_freq_hz <= 0.0f || sampling_time_s <= 0.0f) {
            // 错误处理：如果参数无效，设置为直通模式 (alpha = 1)
            alpha_ = 1.0f;
            return;
        }
        
        // 计算时间常数 tau = 1 / (2 * pi * fc)
        T tau = 1.0f / (2.0f * 3.14159265359f * cutoff_freq_hz);
        
        // 计算 alpha = Ts / (tau + Ts)
        alpha_ = sampling_time_s / (tau + sampling_time_s);
    }

    /**
     * @brief 设置平滑系数 alpha (直接控制方式)
     * @param alpha 范围 [0, 1]
     */
    void setAlpha(T alpha) {
        if (alpha < 0.0f) alpha_ = 0.0f;
        else if (alpha > 1.0f) alpha_ = 1.0f;
        else alpha_ = alpha;
    }

    /**
     * @brief 更新滤波器状态
     * @param input 当前输入值
     * @return T 滤波后的输出值
     */
    T IRAM_ATTR update(T input) {
        // 如果是第一次运行，直接将输入作为输出，避免从0开始爬升的滞后
        if (!initialized_) {
            prev_output_ = input;
            initialized_ = true;
            return input;
        }

        // 核心公式: y[n] = alpha * x[n] + (1 - alpha) * y[n-1]
        T output = alpha_ * input + (1.0f - alpha_) * prev_output_;
        
        // 更新状态
        prev_output_ = output;
        
        return output;
    }

    /**
     * @brief 重置滤波器状态
     * @param value 重置后的初始值 (默认0)
     */
    void reset(T value = 0.0f) {
        prev_output_ = value;
        initialized_ = false; // 重置初始化标志，下一次update将直接使用输入值
    }

    /**
     * @brief 获取当前输出值
     */
    T getValue() const {
        return prev_output_;
    }

private:
    T alpha_;          // 平滑系数 [0, 1]
    T prev_output_;    // 上一次的输出值 (状态变量)
    bool initialized_; // 初始化标志位
};


// //作用:跳变检测函数
// void IRAM_ATTR jumpDetect(float& lastValue, float& currentValue, float threshold) {
   
// }