#include <chrono>
namespace robotnik_charge
{
class Timer
{
public:
    Timer(const double duration) :
        duration_(duration),
        init_time_(std::chrono::steady_clock::now()),
        global_init_time_(init_time_),
        time_elapsed_(0.0)
    {
    }

    ~Timer()
    {
    }

    bool is_timedout() const
    {   
        return std::chrono::steady_clock::now() - init_time_ >= std::chrono::duration<double>(duration_);
    }

    void reset()
    {
        init_time_ = std::chrono::steady_clock::now();
    }

    double get_elapsed_time() const
    {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - global_init_time_).count();
    }

private:
    double duration_;
    std::chrono::steady_clock::time_point init_time_;
    std::chrono::steady_clock::time_point global_init_time_;
    double time_elapsed_;
};
} // namespace robotnik_charge