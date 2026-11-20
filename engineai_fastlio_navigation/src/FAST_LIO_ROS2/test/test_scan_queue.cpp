#include "scan_queue.hpp"
#include <deque>
#include <stdexcept>

struct Frame {double stamp, received_steady;};
void require(bool value) {if (!value) throw std::runtime_error("scan queue invariant failed");}
int main() {
    std::deque<Frame> scans;
    std::deque<double> imu;
    for (int i=0; i<=100; ++i) imu.push_back(1.+i*.005);
    uint64_t skipped=0;
    for (int i=0; i<5; ++i) {
        scans.push_back({1.+i*.1, i*.1});
        fast_lio_queue::trim(scans,3,skipped);
    }
    require(scans.size()==3 && skipped==2);
    fast_lio_queue::select(scans,.41,.25,1.5,[](const auto & f){return f.stamp+.1;},skipped);
    require(scans.size()==3);  // Fresh queues are processed in order.
    fast_lio_queue::select(scans,1.,.25,1.41,[](const auto & f){return f.stamp+.1;},skipped);
    require(scans.size()==2 && scans.front().stamp==1.3 && skipped==3);
    require(imu.size()==101);  // No integration samples discarded with lidar.
    auto time=[](double value){return value;};
    using fast_lio_queue::Coverage;
    require(fast_lio_queue::coverage(imu,1.,1.3,1.4,.05,time)==Coverage::complete);
    require(fast_lio_queue::coverage(imu,1.,1.3,1.6,.05,time)==Coverage::waiting);
    imu.erase(imu.begin()+20,imu.begin()+40);
    require(fast_lio_queue::coverage(imu,1.,1.3,1.4,.05,time)==Coverage::missing);
    imu={2.,2.005,2.01};
    require(fast_lio_queue::coverage(imu,1.5,2.,2.01,.05,time)==Coverage::missing);
    require(fast_lio_queue::coverage(imu,-1.,1.9,2.01,.05,time)==Coverage::waiting);
    // A historic gap cannot permanently block a fully covered new scan.
    imu={1.,1.005,2.,2.005,2.01,2.015,2.02};
    require(fast_lio_queue::coverage(imu,1.,2.,2.02,.1,time)==Coverage::missing);
    auto start=fast_lio_queue::covered_window(imu,2.001,2.02,.1,time);
    require(start && *start==2);
    require(!fast_lio_queue::covered_window(imu,1.001,2.02,.1,time));
    require(!fast_lio_queue::covered_window(imu,2.001,2.03,.1,time));
    imu={2.};
    require(fast_lio_queue::covered_window(imu,2.,2.,.1,time)==0);
    imu={2.,2.005,2.004,2.02};
    require(!fast_lio_queue::covered_window(imu,2.,2.02,.1,time));
    imu={2.,2.005,2.065,2.1};
    require(fast_lio_queue::coverage(imu,2.,2.,2.1,.1,time)==Coverage::complete);
}
