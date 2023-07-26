#pragma once
#include <thread>
#include <string>
#include "ImprovedTypes.hpp"
#include <mutex>
#include <condition_variable>
#include <future>
#include <functional>
#include <memory>
#include <exception>
#include <stdexcept>
#include <deque>
#include <utility>

namespace MORB_SLAM{


template<typename T>
class ManagedFuture{
protected:
    std::shared_ptr<std::promise<T>> promise;
    std::shared_future<T> future;

   ManagedFuture(): promise{std::make_shared<std::promise<T>>()}, future{promise->get_future()} {} // make default constructor protected
public:
    virtual ~ManagedFuture(){}

    // functions from std::shared_future<T>

    const T& get() const { return future.get(); }
    void wait() const { future.wait(); }

    template<class Rep, class Period>
    std::future_status wait_for( const std::chrono::duration<Rep,Period>& timeout_duration ) const { return future.wait_for(timeout_duration); }

    template<class Clock, class Duration>
    std::future_status wait_until( const std::chrono::time_point<Clock,Duration>& timeout_time ) const { return future.wait_until(timeout_time); }
};

template<typename T>
class ManagedPromise: public ManagedFuture<T>{
    std::shared_ptr<std::mutex> mutex;
public:
    ManagedPromise(): mutex{std::make_shared<std::mutex>()} {}
    virtual ~ManagedPromise(){}
    
    // functions from std::promise<T>

    void set_value_at_thread_exit(const T& value){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value_at_thread_exit(value); }
    void set_value_at_thread_exit(T&& value){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value_at_thread_exit(value); }
    void set_value_at_thread_exit(T& value){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value_at_thread_exit(value); }
    void set_value_at_thread_exit(){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value_at_thread_exit(); }
    void set_exception_at_thread_exit(const std::exception_ptr &p ){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_exception_at_thread_exit(p); }
    void set_exception(const std::exception_ptr &p ){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_exception(p); }
    void set_value(const T& value){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value(value); }
    void set_value(T&& value){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value(value); }
    void set_value(T& value){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value(value); }
    void set_value(){ std::scoped_lock<std::mutex> lock(*mutex); ManagedFuture<T>::promise->set_value(); }
};

class Camera{
    std::deque<std::pair<ManagedPromise<bool>, std::function<void(void)>>> ljobs;
    std::deque<std::pair<ManagedPromise<bool>, std::function<void(void)>>> rjobs;

    std::string name;
    CameraType type;
    std::mutex camMutex;
    
    std::condition_variable camCV;

    bool shouldStop;
    
    std::thread lthread;
    std::thread rthread;

    void threadExec(std::deque<std::pair<ManagedPromise<bool>, std::function<void(void)>>> *jobs);
public:
    Camera(CameraType type, const std::string &name="camera");
    virtual ~Camera();

    ManagedFuture<bool> queue(std::function<void(void)> func, bool isLeft=true);
    ManagedFuture<bool> queueLeft(const std::function<void(void)> &func);
    ManagedFuture<bool> queueRight(const std::function<void(void)> &func);

    CameraType getType() const;
    const std::string &getName() const;
};

typedef std::shared_ptr<Camera> Camera_ptr;
typedef std::weak_ptr<Camera> Camera_wptr;

} // namespace MORB_SLAM