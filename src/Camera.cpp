#include "MORB_SLAM/Camera.hpp"

namespace MORB_SLAM{

Camera::Camera(CameraType::eSensor type, const std::string &name): ljobs{}, rjobs{}, name{name}, type{type},
    shouldStop{false}, lthread{&Camera::threadExec, this, &ljobs}, rthread{&Camera::threadExec, this, &rjobs} {

}

Camera::~Camera(){
    {
        std::scoped_lock<std::mutex> lock(camMutex);
        shouldStop = true;
        camCV.notify_all();
    }

    if(lthread.joinable()) lthread.join();
    if(rthread.joinable()) rthread.join();
}

void Camera::threadExec(std::deque<std::pair<ManagedPromise<bool>, std::function<void(void)>>> *jobs){
    while(true){
        ManagedPromise<bool> promise;
        std::function<void(void)> func;
        {
            std::unique_lock<std::mutex> lock(camMutex);
            if(shouldStop) break;
            while(jobs->empty()){
                camCV.wait(lock);
                if(shouldStop) break;
                if(jobs->empty()) continue;
            }
            promise = jobs->front().first;
            func = jobs->front().second;
            jobs->pop_front();
        }

        try{
            func();
        }catch(const std::exception &e){
            promise.set_exception(std::current_exception());
        }
        promise.set_value(true);
    }
}

ManagedFuture<bool> Camera::queue(std::function<void(void)> func, bool isLeft){
    ManagedPromise<bool> promise;
    std::deque<std::pair<ManagedPromise<bool>, std::function<void(void)>>> &jobs = isLeft ? ljobs : rjobs;
    {
        std::scoped_lock<std::mutex> lock(camMutex);

        if(!jobs.empty()){ // only the newest! // TODO add a policy variable to enable or disable this!
            jobs.front().first.set_value(false);
            jobs.clear();
        }
        if(!shouldStop) jobs.emplace_back(promise,func);
        camCV.notify_all();
    }

    return promise;
}
ManagedFuture<bool> Camera::queueLeft(const std::function<void(void)> &func){ return queue(func, true); }
ManagedFuture<bool> Camera::queueRight(const std::function<void(void)> &func){ return queue(func, false); }

CameraType::eSensor Camera::getType() const { return type; }
const std::string &Camera::getName() const { return name; }

} // namespace MORB_SLAM